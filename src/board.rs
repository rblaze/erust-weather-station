use stm32l0xx_hal::exti::{DirectLine, Exti};
use stm32l0xx_hal::lptim::{ClockSrc, LpTimer};
use stm32l0xx_hal::pac;
use stm32l0xx_hal::pwr::PWR;
use stm32l0xx_hal::rcc;
use stm32l0xx_hal::rcc::RccExt;

use crate::error::Error;
use crate::system_time::Ticker;

pub struct Board {
    pub ticker: Ticker,
}

impl Board {
    pub fn new(_cp: pac::CorePeripherals, dp: pac::Peripherals) -> Result<Self, Error> {
        // Enable debug while sleeping to keep probe-rs happy while WFI
        #[cfg(debug_assertions)]
        #[rustfmt::skip]
        dp.DBG.cr.modify(|_, w| { w
            .dbg_sleep().enabled()
            .dbg_standby().enabled()
            .dbg_stop().enabled()
        });

        let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range5));
        let mut pwr = PWR::new(dp.PWR, &mut rcc);
        let mut exti = Exti::new(dp.EXTI);

        let lptimer = LpTimer::init_periodic(dp.LPTIM, &mut pwr, &mut rcc, ClockSrc::Lsi);
        let exti_line = DirectLine::Lptim1;
        exti.listen_direct(exti_line);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::LPTIM1);
        }

        Ok(Self {
            ticker: Ticker::new(lptimer),
        })
    }
}
