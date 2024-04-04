use rtt_target::debug_rprintln;
use stm32g0xx_hal::exti::{Event, ExtiExt};
use stm32g0xx_hal::gpio::gpioa::{PA10, PA11, PA12, PA4, PA5, PA6, PA7, PA9};
use stm32g0xx_hal::gpio::{GpioExt, Input, OpenDrain, Output, PullUp, SignalEdge};
use stm32g0xx_hal::i2c::{self, I2c};
use stm32g0xx_hal::pac::{self, interrupt, EXTI};
use stm32g0xx_hal::power::{self, PowerExt};
use stm32g0xx_hal::rcc::{self, RccExt};
use stm32g0xx_hal::time::RateExtU32;

use crate::error::Error;
use crate::hal_i2c::I2cBus;
use crate::system_time::Ticker;

type I2cSda = PA10<Output<OpenDrain>>;
type I2cScl = PA9<Output<OpenDrain>>;
type HalI2c1 = I2c<pac::I2C1, I2cSda, I2cScl>;

pub struct Joystick {
    pub up: PA12<Input<PullUp>>,
    pub down: PA7<Input<PullUp>>,
    pub left: PA6<Input<PullUp>>,
    pub right: PA11<Input<PullUp>>,
    pub select: PA5<Input<PullUp>>,
    pub button: PA4<Input<PullUp>>,
}

pub struct Peripherals {
    pub i2c: I2cBus<HalI2c1>,
    pub joystick: Joystick,
}

pub struct Board {
    pub ticker: Ticker,
    pub peripherals: Peripherals,
}

impl Board {
    pub fn new() -> Result<Self, Error> {
        let _cp = pac::CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = pac::Peripherals::take().ok_or(Error::AlreadyTaken)?;

        // Enable debug while stopped to keep probe-rs happy while WFI
        // Enabling DMA resolves another instability issue: https://github.com/probe-rs/probe-rs/issues/350
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
            peripherals: Peripherals {
                i2c: I2cBus::new(i2c),
                joystick,
            },
        })
    }
}

#[interrupt]
unsafe fn EXTI4_15() {
    let exti = &(*EXTI::ptr());
    debug_rprintln!("button interrupt {:b}", exti.fpr1.read().bits());
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
