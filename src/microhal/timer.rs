mod lptim;
mod timers;

use super::rcc::RccControl;

#[allow(unused)]
pub use lptim::{Disabled, Enabled, LptimCounter, LptimEvent, LptimPrescaler};

pub trait TimerExt {
    type RegisterWord;
    type Clock;
    type Prescaler;
    type CountingTimer;

    fn upcounter(
        self,
        clock: Self::Clock,
        prescaler: Self::Prescaler,
        limit: Self::RegisterWord,
        rcc: &RccControl,
    ) -> Self::CountingTimer;
}
