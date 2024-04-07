#![no_std]
#![no_main]

mod board;
mod env;
mod error;
mod hal_i2c;
mod screen;
mod system_time;

use core::cell::{Cell, RefCell};
use core::fmt::Write;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use async_scheduler::mailbox::Mailbox;
use board::{Peripherals, JOYSTICK_EVENT};
use bq24259::BQ24259;
use cortex_m_rt::entry;
use embedded_hal_bus::i2c::RefCellDevice;
use futures::future::try_select;
use futures::task::LocalFutureObj;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use stm32g0xx_hal::hal::digital::v2::InputPin;
use system_time::Duration;

use crate::error::Error;
use crate::screen::Lcd;

use panic_probe as _;
// use panic_halt as _;

async fn panic_if_exited<F: core::future::Future<Output = Result<(), Error>>>(f: F) {
    f.await.expect("error in task");
    unreachable!()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum DisplayPage {
    BatteryStatus,
    ChargerRegisters,
}

async fn display_handler<Bus>(
    mut display: Lcd<Bus>,
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    charger: &RefCell<BQ24259<Bus>>,
    board: &RefCell<Peripherals>,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    loop {
        match page.get() {
            DisplayPage::BatteryStatus => {
                let status = charger.borrow_mut().status()?;
                let battery_volts = board.borrow_mut().vbat.read_battery_volts();

                display.cls()?;
                display.set_output_line(0)?;
                write!(&mut display, "{:?}", status.chrg())?;
                display.set_output_line(1)?;
                write!(&mut display, "vbat {:.2}", battery_volts)?;
            }
            DisplayPage::ChargerRegisters => {
                let mut ch = charger.borrow_mut();
                let status = ch.status()?;
                let faults = ch.new_fault()?;

                display.cls()?;
                display.set_output_line(0)?;
                write!(&mut display, "status {:08b}", u8::from(status))?;
                display.set_output_line(1)?;
                write!(&mut display, "faults {:08b}", u8::from(faults))?;
            }
        }

        // Wait for either sleep or read() to complete and propagate error.
        try_select(
            pin!(async {
                system_time::sleep(Duration::secs(2)).await;
                Ok(())
            }),
            pin!(event.read()),
        )
        .await
        .map_err(|e| e.factor_first().0)?;
    }
}

async fn navigation(
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    board: &RefCell<Peripherals>,
) -> Result<(), Error> {
    loop {
        // Wait for button press
        JOYSTICK_EVENT.read().await?;

        let p = board.borrow_mut();
        let joystick = &p.joystick;
        if joystick.up.is_high()? {
            page.set(DisplayPage::BatteryStatus);
        } else if joystick.down.is_high()? {
            page.set(DisplayPage::ChargerRegisters);
        }

        event.post(());
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

        // Wait 100ms for the LCD to power up.
        cortex_m::asm::delay(600000);

        let peripherals = RefCell::new(board.peripherals);
        let display_refresh_event = Mailbox::<()>::new();
        let display_page = Cell::new(DisplayPage::BatteryStatus);
        let display = Lcd::new(RefCellDevice::new(&board.i2c))?;
        let charger = RefCell::new(BQ24259::new(RefCellDevice::new(&board.i2c)));

        let charger_watchdog = pin!(panic_if_exited(async {
            loop {
                // Application is single-threaded and charger can't be borrowed
                // by another coroutine.
                charger.borrow_mut().reset_watchdog()?;
                system_time::sleep(Duration::secs(11)).await;
            }
        }));
        let display_handler = pin!(panic_if_exited(display_handler(
            display,
            &display_refresh_event,
            &display_page,
            &charger,
            &peripherals,
        )));
        let navigation = pin!(panic_if_exited(navigation(
            &display_refresh_event,
            &display_page,
            &peripherals
        )));

        LocalExecutor::new().run([
            LocalFutureObj::new(charger_watchdog),
            LocalFutureObj::new(display_handler),
            LocalFutureObj::new(navigation),
        ]);
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
