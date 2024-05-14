pub mod config;
pub mod lptim;
mod reset_enable;

use stm32g0::stm32g071::RCC;

use self::config::Config;

pub trait RccExt {
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc { rcc: self }
    }
}

#[derive(Debug)]
pub struct Rcc {
    rcc: RCC,
}

impl Rcc {
    pub fn freeze(self, config: Config) -> RccControl {
        //TODO: apply config

        if config.lsi_enabled {
            // Enable LSI and wait for it to be ready.
            self.rcc.csr.modify(|_, w| w.lsion().set_bit());
            while self.rcc.csr.read().lsirdy().bit_is_clear() {}
        }

        RccControl { rcc: self.rcc }
    }
}

#[derive(Debug)]
pub struct RccControl {
    rcc: RCC,
}

pub trait ResetEnable {
    fn enable(rcc: &RccControl);
    #[allow(unused)]
    fn disable(rcc: &RccControl);
    fn reset(rcc: &RccControl);
}
