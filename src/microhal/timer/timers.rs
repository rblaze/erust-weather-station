#![allow(unused)]
use stm32g0::stm32g071::{TIM2, TIM3};

use crate::microhal::rcc::{RccControl, ResetEnable};

/// Wrapper for timer peripheral.
#[derive(Debug)]
pub struct Timer<TIM> {
    timer: TIM,
}

/// Counting timer
#[derive(Debug)]
pub struct Counter<TIM> {
    timer: TIM,
}

/// PWM timer
#[derive(Debug)]
pub struct Pwm<TIM> {
    timer: TIM,
}

macro_rules! general_purpose_timer {
    ($TIM:ident, $REG:tt) => {
        impl Timer<$TIM> {
            pub fn new(timer: $TIM) -> Self {
                Self { timer }
            }

            pub fn upcounter(self, prescaler: u16, limit: $REG, rcc: &RccControl) -> Counter<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc.write(|w| w.psc().bits(prescaler));

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr.write(|w| unsafe { w.bits(limit.into()) });

                Counter { timer: self.timer }
            }

            pub fn pwm(self, prescaler: u16, limit: $REG, rcc: &RccControl) -> Pwm<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc.write(|w| w.psc().bits(prescaler));

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr.write(|w| unsafe { w.bits(limit.into()) });

                // TODO: set PWM mode

                Pwm { timer: self.timer }
            }
        }

        impl Counter<$TIM> {
            pub fn start(&self) {
                self.timer.cr1.modify(|_, w| w.cen().set_bit());
            }

            pub fn stop(&self) {
                self.timer.cr1.modify(|_, w| w.cen().clear_bit());
            }

            basic_timer_impl!($TIM, $REG);
        }
    };
}

macro_rules! basic_timer_impl {
    ($TIM:ident, u16) => {
        pub fn counter(&self) -> u16 {
            self.timer.cnt.read().cnt_l().bits()
        }
    };
    ($TIM:ident, u32) => {
        pub fn counter(&self) -> u32 {
            self.timer.cnt.read().bits()
        }
    };
}

general_purpose_timer!(TIM2, u32);
general_purpose_timer!(TIM3, u16);
