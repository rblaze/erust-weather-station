#![allow(unsafe_code)]
use stm32g0::stm32g071::LPTIM2;

use super::{RccControl, ResetEnable};

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LptimClock {
    Pclk = 0b00,
    Lsi = 0b01,
    Hsi16 = 0b10,
    Lse = 0b11,
}

impl ResetEnable for LPTIM2 {
    fn enable(rcc: &RccControl) {
        rcc.rcc.apbenr1.modify(|_, w| w.lptim2en().set_bit());
    }

    fn disable(rcc: &RccControl) {
        rcc.rcc.apbenr1.modify(|_, w| w.lptim2en().clear_bit());
    }

    fn reset(rcc: &RccControl) {
        rcc.rcc.apbrstr1.write(|w| w.lptim2rst().set_bit());
    }
}

pub trait LptimClockExt {
    fn set_clock(clock: LptimClock, rcc: &RccControl);
}

impl LptimClockExt for LPTIM2 {
    fn set_clock(clock: LptimClock, rcc: &RccControl) {
        rcc.rcc
            .ccipr
            .modify(|_, w| w.lptim2sel().variant(clock as u8));
    }
}
