#![allow(unused)]
use stm32g0::stm32g071::{TIM2, TIM3};

use super::TimerExt;
use crate::microhal::rcc::{RccControl, ResetEnable};

/// Basic timers support only one clock
pub enum BasicTimerClock {
    ApbClock,
}

pub struct Counter<TIM> {
    timer: TIM,
}

macro_rules! basic_timer {
    ($TIM:ident, $REG:ty) => {
        impl TimerExt for $TIM {
            type RegisterWord = $REG;
            type Clock = BasicTimerClock;
            type Prescaler = u16;
            type CountingTimer = Counter<$TIM>;

            fn upcounter(
                self,
                _clock: Self::Clock,
                prescaler: Self::Prescaler,
                limit: Self::RegisterWord,
                rcc: &RccControl,
            ) -> Self::CountingTimer {
                Self::enable(rcc);
                Self::reset(rcc);

                self.psc.write(|w| w.psc().bits(prescaler));

                #[allow(unsafe_code)]
                self.arr.write(|w| unsafe { w.bits(limit.into()) });

                Self::CountingTimer { timer: self }
            }
        }

        impl Counter<$TIM> {
            /// Starts counting.
            pub fn start(&self) {
                self.timer.cr1.modify(|_, w| w.cen().set_bit());
            }

            /// Stops counting.
            pub fn stop(&self) {
                self.timer.cr1.modify(|_, w| w.cen().clear_bit());
            }
        }
    };
}

basic_timer!(TIM2, u32);
basic_timer!(TIM3, u16);
