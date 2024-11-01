pub mod config;
pub mod lptim;
mod reset_enable;

use self::config::Config;

use fugit::KilohertzU32;
use stm32g0::stm32g071::RCC;

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

        let (sysclk, bits) = match config.hsisys_prescaler {
            config::Prescaler::NotDivided => (HSI_FREQ, 0b000),
            config::Prescaler::Div2 => (HSI_FREQ / 2, 0b001),
            config::Prescaler::Div4 => (HSI_FREQ / 4, 0b010),
            config::Prescaler::Div8 => (HSI_FREQ / 8, 0b011),
            config::Prescaler::Div16 => (HSI_FREQ / 16, 0b100),
            config::Prescaler::Div32 => (HSI_FREQ / 32, 0b101),
            config::Prescaler::Div64 => (HSI_FREQ / 64, 0b110),
            config::Prescaler::Div128 => (HSI_FREQ / 128, 0b111),
        };

        // Set HSI prescaler.
        self.cr.modify(|_, w| w.hsidiv().variant(bits));

        if config.lsi_enabled {
            // Enable LSI and wait for it to be ready.
            self.csr.modify(|_, w| w.lsion().set_bit());
            while self.csr.read().lsirdy().bit_is_clear() {}
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
