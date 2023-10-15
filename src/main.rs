#![no_std]
#![no_main]

mod board;
mod env;
mod error;
mod screen;
mod system_time;

use core::fmt::Write;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use cortex_m_rt::entry;
use embedded_time::duration::Extensions;
use futures::task::LocalFutureObj;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use stm32l0xx_hal::pac;

use crate::error::Error;
use crate::screen::Lcd;

use panic_probe as _;
// use panic_halt as _;

async fn panic_if_exited<F: core::future::Future<Output = Result<(), Error>>>(f: F) {
    f.await.expect("error in task");
    unreachable!()
}

#[entry]
fn main() -> ! {
    move || -> Result<(), Error> {
        #[cfg(debug_assertions)]
        rtt_init_print!();

        debug_rprintln!("starting");

        let cp = pac::CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = pac::Peripherals::take().ok_or(Error::AlreadyTaken)?;
        let board = board::Board::new(cp, dp)?;

        env::init_env(board.ticker)?;

        let task = pin!(panic_if_exited(async {
            // LCD initialization wait
            system_time::sleep(100.milliseconds()).await;

            let mut display = Lcd::new(board.i2c)?;

            let mut i = 0u32;
            loop {
                display.cls()?;
                display.write_fmt(format_args!("loop {}", i))?;

                debug_rprintln!("loop {}", i);
                i += 1;
                system_time::sleep(2000.milliseconds()).await;
            }
        }));

        LocalExecutor::new().run([LocalFutureObj::new(task)]);
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
