use stm32g0::stm32g071::{LPTIM1, LPTIM2};

use super::{RccControl, ResetEnable};

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LptimClock {
    Pclk = 0b00,
    Lsi = 0b01,
    Hsi16 = 0b10,
    Lse = 0b11,
}

pub trait LptimClockExt {
    fn set_clock(clock: LptimClock, rcc: &RccControl);
}

macro_rules! lptim_rcc {
    ($TIM:ident, $enable:ident, $reset:ident, $clock:ident) => {
        impl ResetEnable for $TIM {
            fn enable(rcc: &RccControl) {
                rcc.rcc.apbenr1.modify(|_, w| w.$enable().set_bit());
            }

            fn disable(rcc: &RccControl) {
                rcc.rcc.apbenr1.modify(|_, w| w.$enable().clear_bit());
            }

            fn reset(rcc: &RccControl) {
                rcc.rcc.apbrstr1.modify(|_, w| w.$reset().set_bit());
                rcc.rcc.apbrstr1.modify(|_, w| w.$reset().clear_bit());
            }
        }

        impl LptimClockExt for $TIM {
            fn set_clock(clock: LptimClock, rcc: &RccControl) {
                rcc.rcc.ccipr.modify(|_, w| w.$clock().variant(clock as u8));
            }
        }
    };
}

lptim_rcc!(LPTIM1, lptim1en, lptim1rst, lptim1sel);
lptim_rcc!(LPTIM2, lptim2en, lptim2rst, lptim2sel);
