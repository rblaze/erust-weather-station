#![no_std]
#![no_main]

mod board;
mod env;
mod error;
mod hal_i2c;
mod screen;
mod system_time;

use core::fmt::Write;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use cortex_m_rt::entry;
use futures::task::LocalFutureObj;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use system_time::Duration;

use crate::error::Error;
use crate::screen::Lcd;

use panic_probe as _;
// use panic_halt as _;

async fn async_main(board: board::Peripherals) -> Result<(), Error> {
    // LCD initialization wait
    system_time::sleep(Duration::millis(100)).await;

    let mut display = Lcd::new(board.i2c)?;
    display.cls()?;

    let mut i = 0u32;
    loop {
        display.set_output_line(0)?;
        display.write_fmt(format_args!("loop {}", i))?;
        display.set_output_line(1)?;
        display.write("demo mode")?;

        debug_rprintln!("loop {}", i);
        i += 1;
        system_time::sleep(Duration::secs(2)).await;
    }
}

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

        let board = board::Board::new()?;

        env::init_env(board.ticker)?;

        let task = pin!(panic_if_exited(async_main(board.peripherals)));

        LocalExecutor::new().run([LocalFutureObj::new(task)]);
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
