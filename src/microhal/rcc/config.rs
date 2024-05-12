/// SYSCLK prescaler
#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Prescaler {
    NotDivided,
    Div2,
    Div4,
    Div8,
    Div16,
    Div32,
    Div64,
    Div128,
    Div256,
    Div512,
}

/// MCU clock configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub(super) cpu_prescaler: Prescaler,
    pub(super) lsi_enabled: bool,
}

impl Config {
    fn defaults() -> Self {
        Self {
            cpu_prescaler: Prescaler::NotDivided,
            lsi_enabled: true,
        }
    }

    /// Use HSI as SYSCLK
    pub fn use_hsi(prescaler: Prescaler) -> Self {
        Self {
            cpu_prescaler: prescaler,
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
