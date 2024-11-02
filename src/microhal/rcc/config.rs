/// SYSCLK prescaler
pub use crate::microhal::pac::rcc::cr::HSIDIV as Prescaler;

/// MCU clock configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub(super) hsisys_prescaler: Prescaler,
    pub(super) lsi_enabled: bool,
}

impl Config {
    fn defaults() -> Self {
        Self {
            hsisys_prescaler: Prescaler::Div1,
            lsi_enabled: true,
        }
    }

    /// Use HSI as SYSCLK
    pub fn sysclk_hsi(prescaler: Prescaler) -> Self {
        Self {
            hsisys_prescaler: prescaler,
            ..Self::defaults()
        }
    }

    /// Enable LSI
    pub fn enable_lsi(self) -> Self {
        Self {
            lsi_enabled: true,
            ..self
        }
    }
}
