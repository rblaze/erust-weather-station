use core::cell::Cell;
use portable_atomic::{AtomicU32, Ordering};

use async_scheduler::executor::{set_environment, Environment, Executor};
use cortex_m::peripheral::{scb::VectActive, SCB};
use once_cell::sync::OnceCell;
use rtt_target::debug_rprintln;

use crate::error::Error;
use crate::system_time::{Instant, Ticker};

fn in_thread_mode() -> bool {
    match SCB::vect_active() {
        VectActive::ThreadMode => true,
        VectActive::Exception(_) => false,
        VectActive::Interrupt { .. } => false,
    }
}

struct Env {
    ticker: Ticker,
    current_executor: Cell<Option<&'static dyn Executor>>,
}

impl Env {
    const fn new(ticker: Ticker) -> Self {
        Self {
            ticker,
            current_executor: Cell::new(None),
        }
    }
}

impl Environment for Env {
    fn wait_for_event_with_deadline(
        &self,
        mask: &AtomicU32,
        tick: Option<async_scheduler::time::Instant>,
    ) {
        debug_rprintln!("waiting for event, timeout {:?}", tick);
        assert!(
            in_thread_mode(),
            "calling wait_for_event_with_timeout() in interrupt handler"
        );

        critical_section::with(|cs| {
            if mask.load(Ordering::Acquire) == 0 {
                // Critical section prevents interrupt handler from updating 'mask' here.
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

    fn enter_executor(&self, executor: &dyn Executor) {
        assert!(
            in_thread_mode(),
            "calling enter_executor() in interrupt handler"
        );
        assert!(
            self.current_executor.get().is_none(),
            "double-entering executor"
        );

        let r = unsafe { core::mem::transmute::<&dyn Executor, &'static dyn Executor>(executor) };
        self.current_executor.set(Some(r));
    }

    fn leave_executor(&self) {
        assert!(
            in_thread_mode(),
            "calling leave_executor() in interrupt handler"
        );

        self.current_executor
            .replace(None)
            .expect("leaving executor without entering");
    }

    fn current_executor(&self) -> Option<&dyn Executor> {
        assert!(
            in_thread_mode(),
            "calling current_executor() in interrupt handler"
        );

        self.current_executor.get()
    }
}

// App is running single thread and interrupt handlers never use environment.
unsafe impl Sync for Env {}
unsafe impl Send for Env {}

static ENV: OnceCell<Env> = OnceCell::new();

pub fn init_env(ticker: Ticker) -> Result<(), Error> {
    set_environment(ENV.get_or_init(|| Env::new(ticker)))?;

    Ok(())
}
