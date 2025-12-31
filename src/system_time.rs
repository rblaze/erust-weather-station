#![allow(unsafe_code)]

use core::cell::Cell;

use critical_section::{CriticalSection, Mutex};
use embedded_hal::digital::OutputPin;
use fugit::{HertzU32, TimerDuration, TimerInstant};
use futures::{Future, FutureExt, select_biased};
use once_cell::sync::OnceCell;
use rtt_target::debug_rprintln;

use stm32g0_hal::gpio::gpioa::PA15;
use stm32g0_hal::gpio::{Output, PushPull};
use stm32g0_hal::pac::{LPTIM2, interrupt};
use stm32g0_hal::rcc::lptim::LptimClock;
use stm32g0_hal::rcc::{LSI_FREQ, Rcc};
use stm32g0_hal::timer::{Enabled, LowPowerTimer, LptimCounter, LptimEvent, LptimPrescaler};

const TIMER_FREQ: u32 = LSI_FREQ.to_Hz() / 128;

type TimerTicks = u64;
pub type Instant = TimerInstant<TimerTicks, TIMER_FREQ>;
pub type Duration = TimerDuration<TimerTicks, TIMER_FREQ>;

#[derive(Debug)]
pub struct Ticker {
    timer: &'static SystemTimer,
}

impl Ticker {
    const MAX_COUNTER: u16 = u16::MAX;
    const CYCLE_LENGTH: TimerTicks = (Self::MAX_COUNTER as TimerTicks) + 1;

    pub fn new(timer: LowPowerTimer<LPTIM2>, rcc: &Rcc) -> Self {
        debug_assert!(TIMER.get().is_none());

        // Set largest prescaler and longest reload cycle.
        // With LSI running at ~32KHz, this gives ~250 ticks per second and
        // a cycle every 260 seconds.
        let counter = timer
            .upcounter(
                LptimClock::Lsi,
                LptimPrescaler::Div128,
                Self::MAX_COUNTER,
                rcc,
            )
            .disable();

        // Enable compare-match interrupt.
        counter.listen(LptimEvent::CmpMatch);

        let systimer = TIMER.get_or_init(|| SystemTimer {
            timer: counter.enable(),
        });

        systimer.timer.start();

        Ticker { timer: systimer }
    }

    fn unreliable_ticks_now(&self) -> (u32, u16) {
        let current_cycle_ticks = self.timer.timer.counter();
        let cycles = critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);

        (cycles, current_cycle_ticks)
    }

    /// Gets current tick count in LPTIM ticks.
    pub fn ticks_now(&self) -> TimerTicks {
        // It is possible to for `num_full_cycles` to increment and counter wrap to zero
        // while getting ticks count. Reading CNT is also unreliable unless two reads in
        // the row return the same result. See RM0444 26.7.8
        let (cycles, ticks) = loop {
            let (first_cycles, first_ticks) = self.unreliable_ticks_now();
            let (second_cycles, second_ticks) = self.unreliable_ticks_now();

            if first_cycles == second_cycles && first_ticks == second_ticks {
                break (first_cycles, first_ticks);
            }
            //            debug_rprintln!(
            //                "LPTIM CNT read retry, {}:{} != {}:{}",
            //                first_cycles,
            //                first_ticks,
            //                second_cycles,
            //                second_ticks
            //            );
        };

        cycles as u64 * Self::CYCLE_LENGTH + ticks as u64
    }

    /// Waits for the specified tick or next interrupt.
    pub fn sleep_until(&self, cs: CriticalSection, tick: Option<Instant>) {
        let timer = &self.timer.timer;

        // Precondition: timer is enabled and CMP is set to the beginning of the next cycle.
        debug_assert_eq!(timer.cmp(), 0);

        let target_counter = if let Some(tick) = tick {
            // This function is called from critical section, with interrupts disabled.
            // If timer rolls over while in this function, interrupt is pending and will wakeup
            // CPU immediatelly upon entering WFI. Next call to sleep_until() will calculate
            // correct tick.
            let target_tick = tick.ticks();
            let target_cycle = target_tick / Self::CYCLE_LENGTH;
            let current_cycle = TICKS.borrow(cs).get().num_full_cycles;
            match target_cycle.cmp(&current_cycle.into()) {
                core::cmp::Ordering::Less => {
                    // Sleep requested to the moment in the past.
                    debug_rprintln!("Wakeup time already passed");
                    return;
                }
                core::cmp::Ordering::Equal => {
                    // Wakeup during current cycle.
                    // Set CMP to wakeup at the right moment.
                    (target_tick % Self::CYCLE_LENGTH) as u16
                }
                core::cmp::Ordering::Greater => {
                    // Wait until end of cycle, wakeup and retry.
                    Self::MAX_COUNTER
                }
            }
        } else {
            // Sleep until event.
            Self::MAX_COUNTER
        };

        // If target counter is MAX_COUNTER, we can't set it because CMP must
        // be less than ARR, so add one and get 0.
        // If target counter is 0, it's already set properly.
        if target_counter != 0 && target_counter != Self::MAX_COUNTER {
            timer.set_cmp(target_counter);
        }

        // Check if the counter didn't run over our target already.
        if timer.counter() < target_counter {
            cortex_m::asm::wfi();
        } else {
            // It did run over, don't wait for interrupt.
            // debug_rprintln!("overrun");
        }

        // Either we have interrupt pending or never called WFI.
        // If there is an LPTIM event, it will be processed right
        // after exiting critical section.
        // Otherwise, reset CMP to zero.
        if !timer.is_pending(LptimEvent::CmpMatch) && timer.cmp() != 0 {
            timer.set_cmp(0);
        }
    }
}

/// Runs provided future with timeout.
/// Returns `Some(x)` if future completes, None if timeout occurs.
pub async fn timeout<T, E, F>(duration: Duration, f: F) -> Result<Option<T>, E>
where
    F: Future<Output = Result<T, E>>,
{
    select_biased! {
        ret = f.fuse() => ret.map(|v| Some(v)),
        _ = sleep(duration).fuse() => Ok(None),
    }
}

/// Sleeps for the specified duration.
/// Clamps at Duration::MAX for u64->i64 conversion.
pub async fn sleep(duration: Duration) {
    async_scheduler::sleep(duration.ticks().try_into().map_or(
        async_scheduler::time::Duration::MAX,
        async_scheduler::time::Duration::new,
    ))
    .await;
}

/// Returns current time.
pub async fn now() -> Instant {
    Instant::from_ticks(async_scheduler::now().await.ticks().clamp(0, i64::MAX) as u64)
}

/// HAL interface to sync and async sleep.
#[derive(Clone, Copy, Debug)]
pub struct Delay {
    sysclk: HertzU32,
}

impl Delay {
    pub fn new(sysclk: HertzU32) -> Self {
        Self { sysclk }
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let ticks = self.sysclk / HertzU32::nanos(ns);
        cortex_m::asm::delay(ticks);
    }
}

impl embedded_hal_async::delay::DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        let duration = Duration::nanos(ns.into());
        if duration.is_zero() {
            // Wait time is shorter than one LPTIM tick.
            // Fallback to busyloop.
            embedded_hal::delay::DelayNs::delay_ns(self, 0);
        } else {
            // Use normal sleep
            sleep(duration).await;
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct TickerState {
    /// Number of times counter wrapped.
    num_full_cycles: u32,
}

static TICKS: Mutex<Cell<TickerState>> = Mutex::new(Cell::new(TickerState { num_full_cycles: 0 }));

#[derive(Debug)]
struct SystemTimer {
    timer: LptimCounter<LPTIM2, Enabled>,
}
// CNT register is read-only after timer initialization.
// Other registers are accessed either in the interrupt handler or in sleep_until()
// during critical section.
unsafe impl Sync for SystemTimer {}

static TIMER: OnceCell<SystemTimer> = OnceCell::new();

#[interrupt]
fn TIM7() {
    // Create debug LED from thin air.
    let mut led =
        unsafe { core::mem::MaybeUninit::<PA15<Output<PushPull>>>::uninit().assume_init() };
    led.set_high().unwrap();

    let timer = &TIMER
        .get()
        .expect("Interrupt from uninitialized timer")
        .timer;

    if timer.is_pending(LptimEvent::CmpMatch) {
        debug_assert!(!timer.is_pending(LptimEvent::CmpOk));

        // Clear compare interrupt.
        timer.unpend(LptimEvent::CmpMatch);

        // Set next cmp event to the start of the loop.
        // sleep_until() will set it to the correct value.
        if timer.cmp() != 0 {
            timer.set_cmp(0);
        }

        // Check ARR last in the handler. Either it has happened and will be
        // processed now, or the next interrupt will be generated when cnt
        // reaches 0, even if it happens right now in the handler.
        if timer.is_pending(LptimEvent::ArrMatch) {
            debug_rprintln!("update event");

            critical_section::with(|cs| {
                let state = TICKS.borrow(cs).get();
                TICKS.borrow(cs).set(TickerState {
                    num_full_cycles: state.num_full_cycles + 1,
                });
            });

            // Clear update event
            timer.unpend(LptimEvent::ArrMatch);
        }
    } else {
        debug_rprintln!("LPTIM interrupt without compare event");
    }
    led.set_low().unwrap();
}
