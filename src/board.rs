use core::cell::RefCell;

use rtt_target::debug_rprintln;
use stm32g0xx_hal::analog::adc::{Adc, AdcExt, OversamplingRatio, Precision, SampleTime};
use stm32g0xx_hal::exti::{Event, ExtiExt};
use stm32g0xx_hal::gpio::gpioa::{PA0, PA1, PA10, PA11, PA12, PA4, PA5, PA6, PA7, PA9};
use stm32g0xx_hal::gpio::{
    Analog, GpioExt, Input, OpenDrain, Output, PullUp, PushPull, SignalEdge,
};
use stm32g0xx_hal::i2c::{self, I2c};
use stm32g0xx_hal::pac::{self, interrupt, EXTI, TIM3};
use stm32g0xx_hal::power::{self, PowerExt};
use stm32g0xx_hal::prelude::_embedded_hal_PwmPin;
use stm32g0xx_hal::rcc::{self, RccExt};
use stm32g0xx_hal::time::RateExtU32;
use stm32g0xx_hal::timer::pwm::{PwmExt, PwmPin};
use stm32g0xx_hal::timer::{Channel1, Channel2, Channel3};

use crate::error::Error;
use crate::hal_i2c::I2cBus;
use crate::system_time::Ticker;

type I2cSda = PA10<Output<OpenDrain>>;
type I2cScl = PA9<Output<OpenDrain>>;
type HalI2c1 = I2c<pac::I2C1, I2cSda, I2cScl>;
pub type BoardI2c = I2cBus<HalI2c1>;

pub struct Joystick {
    pub up: PA12<Input<PullUp>>,
    pub down: PA7<Input<PullUp>>,
    pub left: PA6<Input<PullUp>>,
    pub right: PA11<Input<PullUp>>,
    pub select: PA5<Input<PullUp>>,
    pub button: PA4<Input<PullUp>>,
}

pub struct Backlight {
    red: PwmPin<TIM3, Channel1>,
    green: PwmPin<TIM3, Channel2>,
    blue: PwmPin<TIM3, Channel3>,
}

impl Backlight {
    // TIM3 is 16-bit timer but HAL provides u32 as duty type.
    fn set_duty(pin: &mut impl _embedded_hal_PwmPin<Duty = u32>, duty: u8) {
        debug_assert!(duty <= 100);
        if duty == 0 {
            pin.set_duty(0);
            pin.disable();
        } else {
            let scaled_duty = pin.get_max_duty() * duty as u32 / 100;
            pin.enable();
            pin.set_duty(scaled_duty);
            debug_rprintln!("duty {}", scaled_duty);
        }
    }

    pub fn set(&mut self, red: u8, green: u8, blue: u8) {
        Self::set_duty(&mut self.red, red);
        Self::set_duty(&mut self.green, green);
        Self::set_duty(&mut self.blue, blue);
    }
}

pub struct VBat {
    adc: Adc,
    vbat: PA0<Analog>,
}

impl VBat {
    const VBAT_MULTIPLIER: f32 = 1.343;
    pub fn read_battery_volts(&mut self) -> f32 {
        let battery_mv = self.adc.read_voltage(&mut self.vbat).unwrap();
        battery_mv as f32 / 1000.0 * Self::VBAT_MULTIPLIER
    }
}

pub struct Peripherals {
    pub joystick: Joystick,
    pub vbat: VBat,
    pub backlight: Backlight,
    pub display_power: PA1<Output<PushPull>>,
}

pub struct Board {
    pub ticker: Ticker,
    pub i2c: RefCell<BoardI2c>,
    pub peripherals: Peripherals,
}

impl Board {
    pub fn new() -> Result<Self, Error> {
        let _cp = pac::CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = pac::Peripherals::take().ok_or(Error::AlreadyTaken)?;

        // Enable debug while stopped to keep probe-rs happy while WFI
        // Enabling DMA resolves another instability issue:
        // https://github.com/probe-rs/probe-rs/issues/350
        #[cfg(debug_assertions)]
        {
            dp.DBG.cr.modify(|_, w| w.dbg_stop().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.dmaen().set_bit());
        }

        // Set clock to 4MHz (HSI speed is 16MHz).
        // Check I2C clock requirements (RM0444 32.4.4) before lowering.
        // TODO: consider using Low Power Run mode.
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi(rcc::Prescaler::Div4));
        let mut pwr = dp.PWR.constrain(&mut rcc);
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let mut adc = dp.ADC.constrain(&mut rcc);

        let backlight_pwm = dp.TIM3.pwm(10.kHz(), &mut rcc);
        let backlight_red = backlight_pwm.bind_pin(gpiob.pb4);
        let backlight_green = backlight_pwm.bind_pin(gpiob.pb5);
        let backlight_blue = backlight_pwm.bind_pin(gpiob.pb0);
        debug_rprintln!("backlight pwm freq {}", backlight_pwm.freq());
        debug_rprintln!("backlight max_duty {}", backlight_red.get_max_duty());

        adc.set_sample_time(SampleTime::T_160);
        adc.set_precision(Precision::B_12);
        adc.set_oversampling_ratio(OversamplingRatio::X_16);
        adc.set_oversampling_shift(16);
        adc.oversampling_enable(true);

        // RM0444 15.3.3 Calibration can only be initiated when the ADC voltage regulator is
        // enabled (ADVREGEN = 1 and tADCVREG_SETUP has elapsed) and the ADC is disabled
        // (when ADEN = 0).
        // tADCVREG_SETUP = 20us, at 80MHz it's 80*20 = 1600 cycles. Round up for  a safety
        // margin.
        cortex_m::asm::delay(2000);
        adc.calibrate();

        let joystick = Joystick {
            up: gpioa.pa12.into_pull_up_input(),
            down: gpioa.pa7.into_pull_up_input(),
            left: gpioa.pa6.into_pull_up_input(),
            right: gpioa.pa11.into_pull_up_input(),
            select: gpioa.pa5.into_pull_up_input(),
            button: gpioa.pa4.into_pull_up_input(),
        };

        let i2c_sda = gpioa.pa10.into_open_drain_output();
        let i2c_scl = gpioa.pa9.into_open_drain_output();
        let i2c = I2c::i2c1(
            dp.I2C1,
            i2c_sda,
            i2c_scl,
            i2c::Config::new(100_u32.kHz()),
            &mut rcc,
        );

        pwr.set_mode(power::PowerMode::LowPower(power::LowPowerMode::StopMode2));

        let ticker = Ticker::new(dp.LPTIM2, &mut rcc);

        let exti = dp.EXTI;
        exti.wakeup(Event::LPTIM2);
        exti.listen(Event::GPIO4, SignalEdge::Falling);
        exti.listen(Event::GPIO5, SignalEdge::Falling);
        exti.listen(Event::GPIO6, SignalEdge::Falling);
        exti.listen(Event::GPIO7, SignalEdge::Falling);
        exti.listen(Event::GPIO11, SignalEdge::Falling);
        exti.listen(Event::GPIO12, SignalEdge::Falling);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIM7_LPTIM2);
            pac::NVIC::unmask(pac::Interrupt::EXTI4_15);
        }

        Ok(Self {
            ticker,
            i2c: RefCell::new(I2cBus::new(i2c)),
            peripherals: Peripherals {
                joystick,
                vbat: VBat {
                    adc,
                    vbat: gpioa.pa0.into_analog(),
                },
                backlight: Backlight {
                    red: backlight_red,
                    green: backlight_green,
                    blue: backlight_blue,
                },
                display_power: gpioa.pa1.into_push_pull_output(),
            },
        })
    }
}

pub static JOYSTICK_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
unsafe fn EXTI4_15() {
    let exti = &(*EXTI::ptr());

    debug_rprintln!("button interrupt {:b}", exti.fpr1.read().bits());
    JOYSTICK_EVENT.post(());

    // Clear interrupt for joystick GPIO lines
    exti.fpr1.write(|w| {
        w.fpif4()
            .set_bit()
            .fpif5()
            .set_bit()
            .fpif6()
            .set_bit()
            .fpif7()
            .set_bit()
            .fpif11()
            .set_bit()
            .fpif12()
            .set_bit()
    });
}
