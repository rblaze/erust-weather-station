use rtt_target::debug_rprintln;
use stm32g0xx_hal::exti::{Event, ExtiExt};
use stm32g0xx_hal::gpio::gpioa::{PA10, PA9};
use stm32g0xx_hal::gpio::{GpioExt, OpenDrain, Output, SignalEdge};
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

pub struct Peripherals {
    pub i2c: I2cBus<HalI2c1>,
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

        let _button = gpioa.pa1.into_pull_up_input();

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
        exti.listen(Event::GPIO1, SignalEdge::Falling);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIM7_LPTIM2);
            pac::NVIC::unmask(pac::Interrupt::EXTI0_1);
        }

        Ok(Self {
            ticker,
            peripherals: Peripherals {
                i2c: I2cBus::new(i2c),
            },
        })
    }
}

#[interrupt]
unsafe fn EXTI0_1() {
    debug_rprintln!("button interrupt");
    // Clear interrupt for GPIO1 line
    (*EXTI::ptr()).fpr1.write(|w| w.fpif1().set_bit());
}
