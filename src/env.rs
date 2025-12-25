use portable_atomic::{AtomicBool, Ordering};

use async_scheduler::executor::Environment;
use cortex_m::peripheral::{SCB, scb::VectActive};
use rtt_target::debug_rprintln;

use crate::system_time::{Instant, Ticker};

fn in_thread_mode() -> bool {
    match SCB::vect_active() {
        VectActive::ThreadMode => true,
        VectActive::Exception(_) => false,
        VectActive::Interrupt { .. } => false,
    }
}

#[derive(Debug)]
pub struct Env {
    ticker: Ticker,
}

impl Env {
    pub const fn new(ticker: Ticker) -> Self {
        Self { ticker }
    }
}

impl Environment for Env {
    fn wait_for_event_with_deadline(
        &self,
        event: &AtomicBool,
        tick: Option<async_scheduler::time::Instant>,
    ) {
        debug_rprintln!("waiting for event, timeout {:?}", tick);
        debug_assert!(
            in_thread_mode(),
            "calling wait_for_event_with_timeout() in interrupt handler"
        );

        critical_section::with(|cs| {
            if !event.load(Ordering::Acquire) {
                // Critical section prevents interrupt handler from updating 'event' here.
                // Pending interrupt will wake up CPU and exit critical section.
                self.ticker.sleep_until(
                    cs,
                    tick.map(|t| Instant::from_ticks(t.ticks().clamp(0, i64::MAX) as u64)),
                );
            }
        });
    }

    fn ticks(&self) -> async_scheduler::time::Instant {
        // It is highly unlikely tick count will reach 2^63.
        async_scheduler::time::Instant::new(self.ticker.ticks_now() as i64)
    }
}
