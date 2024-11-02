pub mod config;
pub mod lptim;
mod reset_enable;

use self::config::Config;
use super::pac::RCC;

use fugit::KilohertzU32;

/// HSI frequency
pub const HSI_FREQ: KilohertzU32 = KilohertzU32::MHz(16);

/// Extension trait for RCC
pub trait RccExt {
    /// Constrain the peripheral and configure clocks.
    fn constrain(self, config: Config) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self, config: Config) -> Rcc {
        // HSI is a default clock source.
        // It is enabled and ready by default.

        let sysclk = match config.hsisys_prescaler {
            config::Prescaler::Div1 => HSI_FREQ,
            config::Prescaler::Div2 => HSI_FREQ / 2,
            // TODO: fix when Div3 renamed to Div4
            config::Prescaler::Div3 => HSI_FREQ / 4,
            config::Prescaler::Div8 => HSI_FREQ / 8,
            config::Prescaler::Div16 => HSI_FREQ / 16,
            config::Prescaler::Div32 => HSI_FREQ / 32,
            config::Prescaler::Div64 => HSI_FREQ / 64,
            config::Prescaler::Div128 => HSI_FREQ / 128,
        };

        // Set HSI prescaler.
        self.cr()
            .modify(|_, w| w.hsidiv().variant(config.hsisys_prescaler));

        if config.lsi_enabled {
            // Enable LSI and wait for it to be ready.
            self.csr().modify(|_, w| w.lsion().set_bit());
            while self.csr().read().lsirdy().bit_is_clear() {}
        }

        Rcc { rcc: self, sysclk }
    }
}

/// Constrained RCC peripheral
#[derive(Debug)]
pub struct Rcc {
    rcc: RCC,
    #[allow(unused)]
    sysclk: KilohertzU32,
}

pub trait ResetEnable {
    fn enable(rcc: &Rcc);
    #[allow(unused)]
    fn disable(rcc: &Rcc);
    fn reset(rcc: &Rcc);
}
