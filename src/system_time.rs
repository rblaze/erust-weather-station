use core::cell::Cell;

use critical_section::{CriticalSection, Mutex};
use fugit::{TimerDuration, TimerInstant};
use futures::{select_biased, Future, FutureExt};
use rtt_target::debug_rprintln;
use stm32g0xx_hal::pac::lptim1::RegisterBlock;
use stm32g0xx_hal::pac::{interrupt, LPTIM2};
use stm32g0xx_hal::rcc::{Enable, Rcc, Reset};

const LSI_FREQ: u32 = 32000 / 128;

type TimerTicks = u64;
pub type Instant = TimerInstant<TimerTicks, LSI_FREQ>;
pub type Duration = TimerDuration<TimerTicks, LSI_FREQ>;

pub struct Ticker {
    timer: LPTIM2,
}

impl Ticker {
    const MAX_COUNTER: u16 = u16::MAX;
    const CYCLE_LENGTH: TimerTicks = (Self::MAX_COUNTER as TimerTicks) + 1;

    pub fn new(timer: LPTIM2, rcc: &mut Rcc) -> Self {
        // Enable LSI and wait for it to be ready.
        rcc.csr.modify(|_, w| w.lsion().set_bit());
        while rcc.csr.read().lsirdy().bit_is_clear() {}

        // Use LSI as LPTIM2 clock source.
        rcc.ccipr.modify(|_, w| unsafe { w.lptim2sel().bits(0b01) });
        LPTIM2::enable(rcc);
        LPTIM2::reset(rcc);

        // Enable largest prescaler and longest reload cycle.
        // With LSI running at ~32KHz, this gives ~250 ticks per second and
        // a cycle every 260 seconds.

        // Copied from HAL LpTimer::configure()
        // Disable the timer. The prescaler can only be changed while it's disabled.
        timer.cr.modify(|_, w| w.enable().clear_bit());
        timer.cfgr.modify(|_, w| unsafe { w.presc().bits(0b111) });
        // Enable compare-match interrupt.
        timer.ier.write(|w| w.cmpmie().set_bit());

        timer.cr.modify(|_, w| w.enable().set_bit());

        // "After setting the ENABLE bit, a delay of two counter clock is needed before the LPTIM is
        // actually enabled."
        // The slowest LPTIM clock source is LSI at 32000 Hz, the fastest CPU clock is ~80 MHz. At
        // these conditions, one cycle of the LPTIM clock takes 2500 CPU cycles, so sleep for 5000.
        cortex_m::asm::delay(5000);

        // CMP is set to 0 by reset.
        // ARR can only be changed while the timer is *en*abled.
        timer
            .arr
            .write(|w| unsafe { w.arr().bits(Self::MAX_COUNTER) });
        while timer.isr.read().arrok().bit_is_clear() {}
        timer.icr.write(|w| w.arrokcf().set_bit());

        // Start counting
        timer.cr.modify(|_, w| w.cntstrt().set_bit());

        Ticker { timer }
    }

    fn unreliable_ticks_now(&self) -> (u32, u16) {
        let current_cycle_ticks = self.timer.cnt.read().cnt().bits();
        let cycles = critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);

        (cycles, current_cycle_ticks)
    }

    /// Gets current tick count in LPTIM ticks.
    pub fn ticks_now(&self) -> TimerTicks {
        // It is possible to for `num_full_cycles` to increment and counter
        // wrap to zero between their reads. So we try reading the cycles before
        // and after the counter. If number of cycles doesn't change, the result is valid.
        // Reading CNT is also unreliable unless two reads in the row return the same result.
        // See RM0444 26.7.8
        let (cycles, ticks) = loop {
            let (first_cycles, first_ticks) = self.unreliable_ticks_now();
            let (second_cycles, second_ticks) = self.unreliable_ticks_now();

            if first_cycles == second_cycles && first_ticks == second_ticks {
                break (first_cycles, first_ticks);
            }
            debug_rprintln!(
                "LPTIM CNT read retry, {}:{} != {}:{}",
                first_cycles,
                first_ticks,
                second_cycles,
                second_ticks
            );
        };

        cycles as u64 * Self::CYCLE_LENGTH + ticks as u64
    }

    /// Waits for the specified tick or next interrupt.
    pub fn sleep_until(&self, cs: CriticalSection, tick: Option<Instant>) {
        // Precondition: timer is enabled and CMP is set to the beginning of the next cycle.
        debug_assert_eq!(self.timer.cmp.read().cmp().bits(), 0);
        debug_assert!(self.timer.isr.read().cmpok().bit_is_clear());
        debug_assert!(self.timer.cfgr.read().preload().bit_is_clear());

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
            set_cmp(&self.timer, target_counter);
        }

        // Check if the counter didn't run over our target already.
        if self.timer.cnt.read().cnt().bits() < target_counter {
            cortex_m::asm::wfi();
        } else {
            // It did run over, don't wait for interrupt.
            debug_rprintln!("overrun");
        }

        // Either we have interrupt pending or never called WFI.
        // If there is an LPTIM event, it will be processed right
        // after exiting critical section.
        // Otherwise, reset CMP to zero.
        if self.timer.isr.read().cmpm().bit_is_clear() && self.timer.cmp.read().cmp().bits() != 0 {
            set_cmp(&self.timer, 0);
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
    async_scheduler::executor::sleep(duration.ticks().try_into().map_or(
        async_scheduler::time::Duration::MAX,
        async_scheduler::time::Duration::new,
    ))
    .await;
}

/// Returns current time.
pub fn now() -> Instant {
    Instant::from_ticks(async_scheduler::executor::now().ticks().clamp(0, i64::MAX) as u64)
}

/// Writes to CMP and waits for CMPOK to be set again to make sure there is
/// no races later.
fn set_cmp(lptim: &RegisterBlock, value: u16) {
    debug_assert!(lptim.isr.read().cmpok().bit_is_clear());

    lptim.cmp.write(|w| unsafe { w.cmp().bits(value) });
    while lptim.isr.read().cmpok().bit_is_clear() {}
    lptim.icr.write(|w| w.cmpokcf().set_bit());
    while lptim.isr.read().cmpok().bit_is_set() {}
}

fn handle_arr_event(lptim: &RegisterBlock) {
    if lptim.isr.read().arrm().bit_is_set() {
        debug_rprintln!("update event");

        critical_section::with(|cs| {
            let state = TICKS.borrow(cs).get();
            TICKS.borrow(cs).set(TickerState {
                num_full_cycles: state.num_full_cycles + 1,
            });
        });

        // Clear update event
        lptim.icr.write(|w| w.arrmcf().set_bit());
    }
}

#[derive(Clone, Copy, Debug)]
struct TickerState {
    /// Number of times counter wrapped.
    num_full_cycles: u32,
}

static TICKS: Mutex<Cell<TickerState>> = Mutex::new(Cell::new(TickerState { num_full_cycles: 0 }));

#[interrupt]
unsafe fn TIM7_LPTIM2() {
    let lptim = &(*LPTIM2::ptr());

    if lptim.isr.read().cmpm().bit_is_set() {
        debug_assert!(lptim.isr.read().cmpok().bit_is_clear());

        // Clear compare interrupt.
        lptim.icr.write(|w| w.cmpmcf().set_bit());
        while lptim.isr.read().cmpm().bit_is_set() {}

        // Set next cmp event to the start of the loop.
        // sleep_until() will set it to the correct value.
        let cmp = lptim.cmp.read().cmp().bits();
        if cmp != 0 {
            set_cmp(lptim, 0);
        }

        // Check ARR last in the handler. Either it is happened and will be
        // processed now, or next interrupt will be generated when cnt
        // reaches 0, even it happens right now in the handler.
        handle_arr_event(lptim);
    } else {
        debug_rprintln!("LPTIM interrupt without compare event");
    }
}
