use core::cell::Cell;

use critical_section::{CriticalSection, Mutex};
use embedded_time::duration::Milliseconds;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::{Fraction, Hertz};
use embedded_time::{Clock, Instant};
use rtt_target::debug_rprintln;
use stm32l0xx_hal::pac::{self, interrupt, LPTIM};
use stm32l0xx_hal::rcc::{Enable, Rcc, Reset};

pub type TimeUnit = Milliseconds;

#[derive(Debug)]
pub struct Ticker {
    timer: pac::LPTIM,
    conversion_factor: Fraction,
}

impl Ticker {
    const MAX_COUNTER: u16 = 0xffff;
    const CYCLE_LENGTH: u32 = (Self::MAX_COUNTER as u32) + 1;

    pub fn new(timer: pac::LPTIM, rcc: &mut Rcc, lsi_freq: Hertz) -> Self {
        // Use LSI as LPTIM clock source.
        rcc.ccipr.modify(|_, w| w.lptim1sel().lsi());
        pac::LPTIM::enable(rcc);
        pac::LPTIM::reset(rcc);

        // Enable largest prescaler and longest reload cycle.
        // With LSI running at ~40KHz, this gives ~300 ticks per second and
        // a cycle every 210 seconds.

        // Copied from HAL LpTimer::configure()
        // Disable the timer. The prescaler can only be changed while it's disabled.
        timer.cr.modify(|_, w| w.enable().clear_bit());
        timer.cfgr.modify(|_, w| w.presc().div128());
        // Enable autoreload and compare-match interrupts.
        timer.ier.write(|w| w.arrmie().enabled().cmpmie().enabled());

        timer.cr.modify(|_, w| w.enable().enabled());

        // "After setting the ENABLE bit, a delay of two counter clock is needed before the LPTIM is
        // actually enabled."
        // The slowest LPTIM clock source is LSE at 32768 Hz, the fastest CPU clock is ~80 MHz. At
        // these conditions, one cycle of the LPTIM clock takes 2500 CPU cycles, so sleep for 5000.
        cortex_m::asm::delay(5000);

        // ARR can only be changed while the timer is *en*abled
        timer.arr.write(|w| w.arr().bits(Self::MAX_COUNTER));
        // When no wakeup needed, set CMP equal to ARR so interrupt will merge with Update interrupt.
        timer.cmp.write(|w| w.cmp().bits(Self::MAX_COUNTER));

        // Wait for register update operation to complete
        while timer.isr.read().cmpok().bit_is_clear() {}

        // Start counting
        timer.cr.modify(|_, w| w.cntstrt().start());

        // Minimize fraction by rounding lsi_freq to the multiple of 128 (prescaler).
        let conversion_factor =
            (Self::SCALING_FACTOR * Fraction::new((lsi_freq / 128).integer(), 1)).recip();

        debug_rprintln!(
            "Conversion factor from LPTIM ticks to API ticks (ms): {:?}",
            conversion_factor
        );

        Ticker {
            timer,
            conversion_factor,
        }
    }

    /// Gets current tick count in LPTIM ticks.
    /// Public API is in 1KHz timer so this function is private.
    fn lptim_ticks(&self) -> u32 {
        // It is possible to for `num_full_cycles` to increment and counter
        // wrap to zero between their reads. So we try reading the cycles before
        // and after the counter. If cycles number don't change, the result is valid.
        loop {
            let cycles = critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);
            let current_cycle_ticks: u32 = self.timer.cnt.read().cnt().bits().into();
            let control_cycles =
                critical_section::with(|cs| TICKS.borrow(cs).get().num_full_cycles);

            if cycles == control_cycles {
                // At 300Hz, 32-bit clock will wrap at ~165 days.
                // TODO: support clock wraping or switch to 64-bit ticks.
                return (cycles * Self::CYCLE_LENGTH) + current_cycle_ticks;
            } else {
                debug_rprintln!("LPTIM read retry, {} != {}", cycles, control_cycles);
            }
        }
    }

    /// Gets current tick count in API ticks.
    pub fn ticks(&self) -> u32 {
        // FIXME: with conversion factor like 1000/293, u32 will overflow at ~2^22 ticks
        // or in 4 hours.
        let ticks = Fraction::new(self.lptim_ticks(), 1) * self.conversion_factor;
        ticks.to_integer()
    }

    /// Waits for the specified tick or next interrupt.
    pub fn sleep_until(&self, cs: CriticalSection, tick: Option<u32>) {
        let mut target_counter = Self::MAX_COUNTER;

        if let Some(tick) = tick {
            // This method is called from critical section, with interrupts disabled.
            // Therefore, missing cycles value update is irrelevant: update interrupt
            // will wakeup CPU anyway and we will redo the calculation.
            let lptim_tick = (Fraction::new(tick, 1) / self.conversion_factor).to_integer();
            let target_cycle = lptim_tick / Self::CYCLE_LENGTH;
            let current_cycle = TICKS.borrow(cs).get().num_full_cycles;
            if target_cycle == current_cycle {
                // Set CMP to wakeup at the right moment.
                target_counter = (lptim_tick % Self::CYCLE_LENGTH) as u16;
            }
        }
        // Clear CMPOK bit, write to CMP and wait for CMPOK to be set again
        // to make sure there is no races later.
        self.timer.icr.write(|w| w.cmpokcf().set_bit());
        self.timer.cmp.write(|w| w.cmp().bits(target_counter));
        while self.timer.isr.read().cmpok().bit_is_clear() {}

        // Check if the counter didn't run over our target already.
        if self.timer.cnt.read().cnt().bits() < target_counter {
            cortex_m::asm::wfi();
        } else {
            // It did run over, disable CMP and don't wait for interrupt.
            self.timer.cmp.write(|w| w.cmp().bits(Self::MAX_COUNTER));
        }
    }
}

// Since clock rate is expected to be known at compile time, set it to 1KHz
// and convert to/from LPTIM ticks.
impl Clock for Ticker {
    type T = u32;

    const SCALING_FACTOR: Fraction = TimeUnit::SCALING_FACTOR;

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.ticks()))
    }
}

pub async fn sleep(duration: TimeUnit) {
    async_scheduler::executor::sleep(duration.integer()).await;
}

#[derive(Clone, Copy, Debug)]
struct TickerState {
    /// Number of times counter wrapped.
    num_full_cycles: u32,
}

static TICKS: Mutex<Cell<TickerState>> = Mutex::new(Cell::new(TickerState { num_full_cycles: 0 }));

#[interrupt]
unsafe fn LPTIM1() {
    let lptim = &(*LPTIM::ptr());
    let flags = lptim.isr.read();

    if flags.arrm().is_set() {
        debug_rprintln!("update event");

        critical_section::with(|cs| {
            let state = TICKS.borrow(cs).get();
            TICKS.borrow(cs).set(TickerState {
                num_full_cycles: state.num_full_cycles + 1,
            });
        });

        // Clear both interrupts: this one will wake up the CPU anyway.
        lptim.icr.write(|w| w.arrmcf().clear().cmpmcf().clear());
    } else if flags.cmpm().is_set() {
        debug_rprintln!("compare event");

        // Set CMP to 0xFFFF so interrupt will merge with Update interrupt.
        lptim.cmp.write(|w| w.cmp().bits(Ticker::MAX_COUNTER));

        // Clear compare interrupt only so we don't lose update event.
        lptim.icr.write(|w| w.cmpmcf().clear());
    }
}
