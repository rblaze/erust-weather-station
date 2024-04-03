use core::cell::Cell;

use critical_section::{CriticalSection, Mutex};
use fugit::{TimerDuration, TimerInstant};
use rtt_target::debug_rprintln;
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
        // Enable autoreload and compare-match interrupts.
        timer.ier.write(|w| w.arrmie().set_bit().cmpmie().set_bit());

        timer.cr.modify(|_, w| w.enable().set_bit());

        // "After setting the ENABLE bit, a delay of two counter clock is needed before the LPTIM is
        // actually enabled."
        // The slowest LPTIM clock source is LSI at 32000 Hz, the fastest CPU clock is ~80 MHz. At
        // these conditions, one cycle of the LPTIM clock takes 2500 CPU cycles, so sleep for 5000.
        cortex_m::asm::delay(5000);

        // ARR can only be changed while the timer is *en*abled
        timer
            .arr
            .write(|w| unsafe { w.arr().bits(Self::MAX_COUNTER) });
        // When no wakeup needed, set CMP equal to ARR so interrupt will merge with Update interrupt.
        timer
            .cmp
            .write(|w| unsafe { w.cmp().bits(Self::MAX_COUNTER) });

        // Wait for register update operation to complete
        while timer.isr.read().cmpok().bit_is_clear() {}

        // Start counting
        timer.cr.modify(|_, w| w.cntstrt().set_bit());

        Ticker { timer }
    }

    /// Gets current tick count in LPTIM ticks.
    fn lptim_ticks(&self) -> TimerTicks {
        // It is possible to for `num_full_cycles` to increment and counter
        // wrap to zero between their reads. So we try reading the cycles before
        // and after the counter. If number of cycles doesn't change, the result is valid.
        loop {
            let cycles = critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);
            let current_cycle_ticks: TimerTicks = self.timer.cnt.read().cnt().bits().into();
            let control_cycles =
                critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);

            if cycles == control_cycles {
                return (cycles as TimerTicks * Self::CYCLE_LENGTH) + current_cycle_ticks;
            } else {
                debug_rprintln!("LPTIM read retry, {} != {}", cycles, control_cycles);
            }
        }
    }

    /// Gets current time.
    pub fn now(&self) -> Instant {
        Instant::from_ticks(self.lptim_ticks())
    }

    /// Waits for the specified tick or next interrupt.
    pub fn sleep_until(&self, cs: CriticalSection, tick: Option<Instant>) {
        let mut target_counter = Self::MAX_COUNTER;

        if let Some(tick) = tick {
            // This method is called from critical section, with interrupts disabled.
            // Therefore, missing cycles value update is irrelevant: update interrupt
            // will wakeup CPU, end current wait and we will redo the calculation.
            let lptim_tick = tick.ticks();
            let target_cycle = lptim_tick / Self::CYCLE_LENGTH;
            let current_cycle = TICKS.borrow(cs).get().num_full_cycles;
            if target_cycle == current_cycle.into() {
                // Set CMP to wakeup at the right moment.
                target_counter = (lptim_tick % Self::CYCLE_LENGTH) as u16;
            }
        }
        // Clear CMPOK bit, write to CMP and wait for CMPOK to be set again
        // to make sure there is no races later.
        self.timer.icr.write(|w| w.cmpokcf().set_bit());
        self.timer
            .cmp
            .write(|w| unsafe { w.cmp().bits(target_counter) });
        while self.timer.isr.read().cmpok().bit_is_clear() {}

        // Check if the counter didn't run over our target already.
        if self.timer.cnt.read().cnt().bits() < target_counter {
            cortex_m::asm::wfi();
        } else {
            // It did run over, disable CMP and don't wait for interrupt.
            self.timer
                .cmp
                .write(|w| unsafe { w.cmp().bits(Self::MAX_COUNTER) });
        }
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

#[derive(Clone, Copy, Debug)]
struct TickerState {
    /// Number of times counter wrapped.
    num_full_cycles: u32,
}

static TICKS: Mutex<Cell<TickerState>> = Mutex::new(Cell::new(TickerState { num_full_cycles: 0 }));

#[interrupt]
unsafe fn TIM7_LPTIM2() {
    let lptim = &(*LPTIM2::ptr());
    let flags = lptim.isr.read();

    if flags.arrm().bit_is_set() {
        debug_rprintln!("update event");

        critical_section::with(|cs| {
            let state = TICKS.borrow(cs).get();
            TICKS.borrow(cs).set(TickerState {
                num_full_cycles: state.num_full_cycles + 1,
            });
        });

        // Clear both interrupts: this one will wake up the CPU anyway.
        lptim.icr.write(|w| w.arrmcf().set_bit().cmpmcf().set_bit());
    } else if flags.cmpm().bit_is_set() {
        debug_rprintln!("compare event");

        // Set CMP to 0xFFFF so interrupt will merge with Update interrupt.
        lptim.cmp.write(|w| w.cmp().bits(Ticker::MAX_COUNTER));

        // Clear compare interrupt only so we don't lose update event.
        lptim.icr.write(|w| w.cmpmcf().set_bit());
    }
}
