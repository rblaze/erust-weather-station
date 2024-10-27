use core::cell::RefCell;

use rtt_target::debug_rprintln;
use stm32g0::stm32g071::{I2C1, LPTIM2};
use stm32g0xx_hal::exti::{Event, ExtiExt};
use stm32g0xx_hal::gpio::SignalEdge;
use stm32g0xx_hal::pac::{self, interrupt, EXTI, TIM3};
use stm32g0xx_hal::power::{self, PowerExt};
use stm32g0xx_hal::rcc::{self, RccExt};

use crate::error::Error;
use crate::microhal::adc::Adc;
use crate::microhal::gpio::gpiob::{PB0, PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB4, PB5, PB6};
use crate::microhal::gpio::{Alternate, Analog, GpioExt, Input, PullUp, PushPull};
use crate::microhal::i2c::{self, I2c, I2cExt};
use crate::microhal::rcc::config::Prescaler;
use crate::microhal::timer::{LowPowerTimer, Pwm, Timer};
use crate::system_time::Ticker;

// type I2cSda = PA10<Output<OpenDrain>>; // TODO: PB4
// type I2cScl = PA9<Output<OpenDrain>>; // TODO: PB3
pub type BoardI2c = I2c<I2C1>;

#[allow(unused)]
pub struct Joystick {
    pub up: PB13<Input<PullUp>>,
    pub down: PB12<Input<PullUp>>,
    pub left: PB11<Input<PullUp>>,
    pub right: PB10<Input<PullUp>>,
    pub select: PB15<Input<PullUp>>,
    pub button: PB14<Input<PullUp>>,
}

pub struct VBat {
    adc: Adc,
    vbat: PB2<Analog>,
}

impl VBat {
    const VBAT_MULTIPLIER: f32 = 1.622;

    pub fn read_battery_volts(&mut self) -> f32 {
        let battery_mv = self.adc.read_voltage(&mut self.vbat);
        battery_mv as f32 / 1000.0 * Self::VBAT_MULTIPLIER
    }
}

pub struct Backlight {
    pub pwm: Pwm<TIM3>,
    pub red: PB4<Alternate<1>>,
    pub green: PB5<Alternate<1>>,
    pub blue: PB0<Alternate<1>>,
}

pub type DisplayPowerPin = PB6<crate::microhal::gpio::Output<PushPull>>;

pub struct Board {
    pub ticker: Ticker,
    pub i2c: RefCell<BoardI2c>,
    pub backlight: Backlight,
    pub vbat: VBat,
    pub joystick: Joystick,
    pub display_power: DisplayPowerPin,
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

        // Set clock to 4MHz (HSI speed is 16MHz).
        // Check I2C clock requirements (RM0444 32.4.4) before lowering.
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi(rcc::Prescaler::Div4));
        let mut pwr = dp.PWR.constrain(&mut rcc);
        let gpioa = dp.GPIOA.split(&rcc_control);
        let gpiob = dp.GPIOB.split(&rcc_control);

        let backlight_pwm = Timer::<TIM3>::new(dp.TIM3).pwm(0, u16::MAX, &rcc_control);
        let mut adc = Adc::new(dp.ADC, &rcc_control);

        adc.calibrate();

        let joystick = Joystick {
            up: gpiob.pb13.into_pullup_input(),
            down: gpiob.pb12.into_pullup_input(),
            left: gpiob.pb11.into_pullup_input(),
            right: gpiob.pb10.into_pullup_input(),
            select: gpiob.pb15.into_pullup_input(),
            button: gpiob.pb14.into_pullup_input(),
        };

        let i2c_sda = gpioa.pa10.into_open_drain_output();
        let i2c_scl = gpioa.pa9.into_open_drain_output();
        let config = i2c::Config::from_cubemx(true, 0, 0x00100D14);
        let i2c = dp.I2C1.i2c(i2c_sda, i2c_scl, &config, &rcc_control);

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
            i2c: RefCell::new(i2c),
            joystick,
            vbat: VBat {
                adc,
                vbat: gpiob.pb2.into_analog(),
            },
            backlight: Backlight {
                pwm: backlight_pwm,
                red: gpiob.pb4.into_alternate_function(),
                green: gpiob.pb5.into_alternate_function(),
                blue: gpiob.pb0.into_alternate_function(),
            },
            display_power: gpiob.pb6.into_push_pull_output(),
        })
    }
}

pub static JOYSTICK_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
unsafe fn EXTI4_15() {
    let exti = &(*EXTI::ptr());

    debug_rprintln!("button interrupt {:016b}", exti.fpr1.read().bits());
    debug_rprintln!("button mask      5432109876543210");
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
