#![no_std]
#![no_main]

mod board;
mod display;
mod env;
mod error;
mod hal_compat;
mod screen;
mod system_time;

use core::cell::{Cell, RefCell};
use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use async_scheduler::mailbox::Mailbox;
use board::{Peripherals, JOYSTICK_EVENT};
use bq24259::BQ24259;
use cortex_m_rt::entry;
use embedded_hal::digital::InputPin;
use embedded_hal_bus::i2c::RefCellDevice;
use futures::task::LocalFutureObj;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use system_time::Duration;

use crate::display::DisplayPage;
use crate::error::Error;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    debug_rprintln!("{}", info);

    cortex_m::asm::bkpt();
    cortex_m::asm::udf();
}

async fn panic_if_exited<F: core::future::Future<Output = Result<(), Error>>>(f: F) {
    panic!("future exited with {:?}", f.await)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum DisplayBacklight {
    Off,
    Half,
    Full,
}

impl DisplayBacklight {
    fn next(self) -> Self {
        use DisplayBacklight::*;
        match self {
            Off => Half,
            Half => Full,
            Full => Off,
        }
    }
}

async fn navigation(
    display_event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    backlight_event: &Mailbox<()>,
    backlight: &Cell<DisplayBacklight>,
    board: &RefCell<Peripherals>,
) -> Result<(), Error> {
    let mut last_visible_page = None;

    loop {
        // Wait for button press
        JOYSTICK_EVENT.read().await?;

        let current_page = page.get();
        let joystick = &mut board.borrow_mut().joystick;
        if joystick.up.is_low()? {
            page.set(current_page.prev());
        } else if joystick.down.is_low()? {
            page.set(current_page.next());
        } else if joystick.button.is_low()? {
            match last_visible_page {
                Some(last_page) => {
                    last_visible_page = None;
                    page.set(last_page);
                }
                None => {
                    last_visible_page = Some(current_page);
                    backlight.set(DisplayBacklight::Off);
                    backlight_event.post(());
                    page.set(DisplayPage::Off);
                }
            }
        } else if page.get() != DisplayPage::Off && joystick.select.is_low()? {
            backlight.set(backlight.get().next());
            backlight_event.post(());
        }

        display_event.post(());
    }
}

#[entry]
fn main() -> ! {
    move || -> Result<(), Error> {
        #[cfg(debug_assertions)]
        rtt_init_print!();

        debug_rprintln!("starting");

        let board = board::Board::new()?;

        env::init_env(board.ticker)?;

        let peripherals = RefCell::new(board.peripherals);
        let backlight_event = Mailbox::<()>::new();
        let backlight = Cell::new(DisplayBacklight::Off);
        let display_refresh_event = Mailbox::<()>::new();
        let display_page = Cell::new(DisplayPage::BatteryStatus);
        let display_bus = RefCellDevice::new(&board.i2c);
        let charger = RefCell::new(BQ24259::new(RefCellDevice::new(&board.i2c)));

        let charger_watchdog = pin!(panic_if_exited(async {
            loop {
                debug_rprintln!("watchdog");
                // Application is single-threaded and charger can't be borrowed
                // by another coroutine.
                charger.borrow_mut().reset_watchdog()?;
                system_time::sleep(Duration::secs(11)).await;
            }
        }));
        let backlight_handler = pin!(panic_if_exited(async {
            loop {
                debug_rprintln!("backlight {:?}", backlight.get());
                match backlight.get() {
                    DisplayBacklight::Off => peripherals.borrow_mut().backlight.set(0, 0, 0),
                    DisplayBacklight::Half => peripherals.borrow_mut().backlight.set(50, 50, 50),
                    DisplayBacklight::Full => peripherals.borrow_mut().backlight.set(100, 100, 100),
                };

                backlight_event.read().await?;
            }
        }));
        let display_handler = pin!(panic_if_exited(display::task(
            display_bus,
            &display_refresh_event,
            &display_page,
            &charger,
            &peripherals,
        )));
        let navigation = pin!(panic_if_exited(navigation(
            &display_refresh_event,
            &display_page,
            &backlight_event,
            &backlight,
            &peripherals
        )));

        LocalExecutor::new().run([
            LocalFutureObj::new(charger_watchdog),
            LocalFutureObj::new(navigation),
            LocalFutureObj::new(display_handler),
            LocalFutureObj::new(backlight_handler),
        ]);
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
