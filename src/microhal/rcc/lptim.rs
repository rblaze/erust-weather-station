use crate::microhal::pac::{LPTIM1, LPTIM2};

use super::Rcc;

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LptimClock {
    Pclk = 0b00,
    Lsi = 0b01,
    Hsi16 = 0b10,
    Lse = 0b11,
}

pub trait LptimClockExt {
    fn set_clock(clock: LptimClock, rcc: &Rcc);
}

macro_rules! lptim_rcc {
    ($TIM:ident, $clock:ident) => {
        impl LptimClockExt for $TIM {
            fn set_clock(clock: LptimClock, rcc: &Rcc) {
                rcc.rcc.ccipr.modify(|_, w| w.$clock().variant(clock as u8));
            }
        }
    };
}

lptim_rcc!(LPTIM1, lptim1sel);
lptim_rcc!(LPTIM2, lptim2sel);
