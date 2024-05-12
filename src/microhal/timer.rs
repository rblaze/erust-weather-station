use super::rcc::RccControl;

pub mod lptim;

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
