use core::cell::RefCell;

use rtt_target::debug_rprintln;
use stm32g0::stm32g071::LPTIM2;
use stm32g0xx_hal::analog::adc::{Adc, AdcExt, OversamplingRatio, Precision, SampleTime};
use stm32g0xx_hal::exti::{Event, ExtiExt};
use stm32g0xx_hal::gpio::gpioa::{PA10, PA9};
use stm32g0xx_hal::gpio::gpiob::PB2;
use stm32g0xx_hal::gpio::{Analog, GpioExt, OpenDrain, Output, SignalEdge};
use stm32g0xx_hal::i2c::{self, I2c};
use stm32g0xx_hal::pac::{self, interrupt, EXTI, TIM3};
use stm32g0xx_hal::power::{self, PowerExt};
use stm32g0xx_hal::rcc::{self, RccExt};
use stm32g0xx_hal::time::RateExtU32;

use crate::error::Error;
use crate::hal_compat::I2cBus;
use crate::microhal::gpio::gpiob::{PB0, PB10, PB11, PB12, PB13, PB14, PB15, PB4, PB5, PB6};
use crate::microhal::gpio::{Alternate, Input, PullUp, PushPull};
use crate::microhal::rcc::config::Prescaler;
use crate::microhal::timer::{LowPowerTimer, Pwm, Timer};
use crate::system_time::Ticker;

type I2cSda = PA10<Output<OpenDrain>>; // TODO: PB4
type I2cScl = PA9<Output<OpenDrain>>; // TODO: PB3
type HalI2c1 = I2c<pac::I2C1, I2cSda, I2cScl>;
pub type BoardI2c = I2cBus<HalI2c1>;

pub struct Joystick {
    pub up: PB14<Input<PullUp>>,
    pub down: PB12<Input<PullUp>>,
    pub left: PB15<Input<PullUp>>,
    pub right: PB10<Input<PullUp>>,
    pub select: PB11<Input<PullUp>>,
    pub button: PB13<Input<PullUp>>,
}

pub struct VBat {
    adc: Adc,
    vbat: PB2<Analog>,
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
    pub display_power: PB6<crate::microhal::gpio::Output<PushPull>>,
}

pub struct Backlight {
    pub pwm: Pwm<TIM3>,
    pub red: PB4<Alternate<1>>,
    pub green: PB5<Alternate<1>>,
    pub blue: PB0<Alternate<1>>,
}

pub struct Board {
    pub ticker: Ticker,
    pub i2c: RefCell<BoardI2c>,
    pub peripherals: Peripherals,
    pub backlight: Backlight,
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

        let clocks = crate::microhal::rcc::config::Config::use_hsi(Prescaler::Div4).enable_lsi();
        let microhal_rcc = unsafe { stm32g0::stm32g071::Peripherals::steal().RCC };
        let rcc_control = crate::microhal::rcc::RccExt::constrain(microhal_rcc).freeze(clocks);
        let microhal_gpiob = crate::microhal::gpio::GpioExt::split(
            unsafe { stm32g0::stm32g071::Peripherals::steal().GPIOB },
            &rcc_control,
        );

        // Set clock to 4MHz (HSI speed is 16MHz).
        // Check I2C clock requirements (RM0444 32.4.4) before lowering.
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi(rcc::Prescaler::Div4));
        let mut pwr = dp.PWR.constrain(&mut rcc);
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let mut adc = dp.ADC.constrain(&mut rcc);

        let backlight_pwm = Timer::<TIM3>::new(dp.TIM3).pwm(0, u16::MAX, &rcc_control);

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
            up: microhal_gpiob.pb14.into_pullup_input(),
            down: microhal_gpiob.pb12.into_pullup_input(),
            left: microhal_gpiob.pb15.into_pullup_input(),
            right: microhal_gpiob.pb10.into_pullup_input(),
            select: microhal_gpiob.pb11.into_pullup_input(),
            button: microhal_gpiob.pb13.into_pullup_input(),
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

        let system_timer = LowPowerTimer::<LPTIM2>::new(dp.LPTIM2);
        let ticker = Ticker::new(system_timer, &rcc_control);

        // Upon reset, a pull-down resistor might be present on PB15, PA8, PD0, or PD2,
        // depending on the voltage level on PB0, PA9, PC6, PA10, PD1, and PD3. In order
        // to disable this resistor, strobe the UCPDx_STROBE bit of the SYSCFG_CFGR1
        // register during start-up sequence.
        let syscfg = dp.SYSCFG_VREFBUF;
        syscfg
            .cfgr1
            .modify(|_, w| w.ucpd1_strobe().set_bit().ucpd2_strobe().set_bit());

        let exti = dp.EXTI;
        exti.exticr3
            .modify(|_, w| w.exti16_23().pb().exti24_31().pb());
        exti.exticr4.modify(|_, w| {
            w.exti0_7()
                .pb()
                .exti8_15()
                .pb()
                .exti16_23()
                .pb()
                .exti24_31()
                .pb()
        });
        exti.listen(Event::GPIO10, SignalEdge::Falling);
        exti.listen(Event::GPIO11, SignalEdge::Falling);
        exti.listen(Event::GPIO12, SignalEdge::Falling);
        exti.listen(Event::GPIO13, SignalEdge::Falling);
        exti.listen(Event::GPIO14, SignalEdge::Falling);
        exti.listen(Event::GPIO15, SignalEdge::Falling);
        exti.wakeup(Event::LPTIM2);

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
                    vbat: gpiob.pb2.into_analog(),
                },
                display_power: microhal_gpiob.pb6.into_push_pull_output(),
            },
            backlight: Backlight {
                pwm: backlight_pwm,
                red: microhal_gpiob.pb4.into_alternate_function(),
                green: microhal_gpiob.pb5.into_alternate_function(),
                blue: microhal_gpiob.pb0.into_alternate_function(),
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
        w.fpif10()
            .set_bit()
            .fpif11()
            .set_bit()
            .fpif12()
            .set_bit()
            .fpif13()
            .set_bit()
            .fpif14()
            .set_bit()
            .fpif15()
            .set_bit()
    });
}
