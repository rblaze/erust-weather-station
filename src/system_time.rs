use core::cell::{Cell, RefCell};

use async_scheduler::time::Ticks;
use critical_section::{CriticalSection, Mutex};
use embedded_time::duration::Milliseconds;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::{Fraction, Hertz};
use embedded_time::{Clock, Instant};
use rtt_target::debug_rprintln;
use stm32l0xx_hal::pac::{self, interrupt, LPTIM};
use stm32l0xx_hal::pwr::PWR;
#[cfg(not(debug_assertions))]
use stm32l0xx_hal::pwr::{PowerMode, StopModeConfig};
use stm32l0xx_hal::rcc::{Enable, Rcc, Reset};

type TimerTicks = u64;
pub type TimeUnit = Milliseconds<u64>;

#[cfg_attr(debug_assertions, allow(unused))]
pub struct Sleeper {
    pwr: PWR,
    scb: pac::SCB,
    rcc: Rcc,
}

pub struct Ticker {
    timer: pac::LPTIM,
    conversion_factor: Fraction,
    #[cfg_attr(debug_assertions, allow(unused))]
    sleeper: RefCell<Sleeper>,
}

impl Ticker {
    const MAX_COUNTER: u16 = 0xffff;
    const CYCLE_LENGTH: TimerTicks = (Self::MAX_COUNTER as TimerTicks) + 1;

    pub fn new(timer: pac::LPTIM, mut rcc: Rcc, pwr: PWR, scb: pac::SCB, lsi_freq: Hertz) -> Self {
        // Use LSI as LPTIM clock source.
        rcc.ccipr.modify(|_, w| w.lptim1sel().lsi());
        pac::LPTIM::enable(&mut rcc);
        pac::LPTIM::reset(&mut rcc);

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
            sleeper: RefCell::new(Sleeper { pwr, scb, rcc }),
        }
    }

    /// Gets current tick count in LPTIM ticks.
    /// Public API is in 1KHz timer so this function is private.
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

    /// Gets current tick count in API ticks.
    pub fn ticks(&self) -> Ticks {
        let ticks = self.lptim_ticks() * self.conversion_factor;
        Ticks::new(ticks)
    }

    /// Waits for the specified tick or next interrupt.
    pub fn sleep_until(&self, cs: CriticalSection, tick: Option<Ticks>) {
        let mut target_counter = Self::MAX_COUNTER;

        if let Some(tick) = tick {
            // This method is called from critical section, with interrupts disabled.
            // Therefore, missing cycles value update is irrelevant: update interrupt
            // will wakeup CPU anyway and we will redo the calculation.
            let lptim_tick = tick.ticks() / self.conversion_factor;
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
        self.timer.cmp.write(|w| w.cmp().bits(target_counter));
        while self.timer.isr.read().cmpok().bit_is_clear() {}

        // Check if the counter didn't run over our target already.
        if self.timer.cnt.read().cnt().bits() < target_counter {
            // Probe disconnects when entering STOP mode so limit
            // to SLEEP for debug builds.
            #[cfg(debug_assertions)]
            cortex_m::asm::wfi();
            #[cfg(not(debug_assertions))]
            {
                let sleeper = &mut *self.sleeper.borrow_mut();
                sleeper
                    .pwr
                    .stop_mode(
                        &mut sleeper.scb,
                        &mut sleeper.rcc,
                        StopModeConfig {
                            ultra_low_power: true,
                        },
                    )
                    .enter();
            }
        } else {
            // It did run over, disable CMP and don't wait for interrupt.
            self.timer.cmp.write(|w| w.cmp().bits(Self::MAX_COUNTER));
        }
    }
}

// Since clock rate is expected to be known at compile time, set it to 1KHz
// and convert to/from LPTIM ticks.
impl Clock for Ticker {
    type T = u64;

    const SCALING_FACTOR: Fraction = TimeUnit::SCALING_FACTOR;

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.ticks().ticks()))
    }
}

pub async fn sleep(duration: TimeUnit) {
    async_scheduler::executor::sleep(async_scheduler::time::Ticks::new(duration.integer())).await;
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
