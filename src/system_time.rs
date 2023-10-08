use core::cell::Cell;
use critical_section::Mutex;
use embedded_hal::timer::CountDown;
use embedded_time::duration::Milliseconds;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::{Extensions, Fraction};
use embedded_time::{Clock, Instant};
use stm32l0xx_hal::lptim::{Interrupts, LpTimer, Periodic};
use stm32l0xx_hal::pac::{interrupt, LPTIM};

const HERTZ: u32 = 100;

static TICKS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[derive(Clone, Copy, Debug)]
pub struct Ticker {}

impl Ticker {
    // Setup LpTimer to tick at 100Hz
    pub fn new(mut timer: LpTimer<Periodic>) -> Self {
        timer.enable_interrupts(Interrupts {
            autoreload_match: true,
            ..Default::default()
        });
        timer.start(HERTZ.Hz());

        Ticker {}
    }

    // Get current tick count.
    pub fn ticks(&self) -> u32 {
        critical_section::with(|cs| TICKS.borrow(cs).get())
    }

    // Wait for the next tick.
    // Makes sure the ticker is enabled.
    pub fn wait_for_tick(&self) {
        cortex_m::asm::wfi();
    }
}

impl Clock for Ticker {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, HERTZ);

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.ticks()))
    }
}

pub async fn sleep(duration: Milliseconds) {
    // Tick is 10 ms
    let ticks = duration.integer() / 10;
    async_scheduler::executor::sleep(ticks).await;
}

#[interrupt]
unsafe fn LPTIM1() {
    critical_section::with(|cs| {
        let ticks = TICKS.borrow(cs).get();
        TICKS.borrow(cs).set(ticks + 1);
    });

    // Clear the interrupt
    (*LPTIM::ptr()).icr.write(|w| w.arrmcf().clear());
}
