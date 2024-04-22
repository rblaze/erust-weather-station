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
use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use async_scheduler::mailbox::Mailbox;
use board::{Peripherals, JOYSTICK_EVENT};
use bq24259::BQ24259;
use cortex_m_rt::entry;
use embedded_hal_bus::i2c::RefCellDevice;
use fugit::SecsDurationU64;
use futures::future::try_select;
use futures::task::LocalFutureObj;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use stm32g0xx_hal::hal::digital::v2::{InputPin, OutputPin};
use system_time::{Duration, Instant};

use crate::error::Error;
use crate::screen::Lcd;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum DisplayPage {
    BatteryStatus,
    ChargerRegisters,
    SystemStatus,
    Off,
}

impl DisplayPage {
    fn next(self) -> Self {
        use DisplayPage::*;
        match self {
            BatteryStatus => ChargerRegisters,
            ChargerRegisters => SystemStatus,
            SystemStatus => BatteryStatus,
            Off => Off,
        }
    }

    fn prev(self) -> Self {
        use DisplayPage::*;
        match self {
            BatteryStatus => SystemStatus,
            ChargerRegisters => BatteryStatus,
            SystemStatus => ChargerRegisters,
            Off => Off,
        }
    }
}

async fn show_page<Bus>(
    page: DisplayPage,
    start_time: Instant,
    display: &mut Lcd<'_, Bus>,
    charger: &RefCell<BQ24259<Bus>>,
    board: &RefCell<Peripherals>,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    debug_rprintln!("display loop");
    match page {
        DisplayPage::BatteryStatus => {
            let status = charger.borrow_mut().status()?;
            let battery_volts = board.borrow_mut().vbat.read_battery_volts();

            display.cls()?;
            display.set_output_line(0)?;
            write!(display, "{:?}", status.chrg())?;
            display.set_output_line(1)?;
            write!(display, "vbat {:.2}", battery_volts)?;
        }
        DisplayPage::ChargerRegisters => {
            let mut ch = charger.borrow_mut();
            let status = ch.status()?;
            let faults = ch.new_fault()?;

            debug_rprintln!("status {:?}", status);
            debug_rprintln!("faults {:?}", faults);

            display.cls()?;
            display.set_output_line(0)?;
            write!(display, "status {:08b}", u8::from(status))?;
            display.set_output_line(1)?;
            write!(display, "faults {:08b}", u8::from(faults))?;
        }
        DisplayPage::SystemStatus => {
            let uptime: SecsDurationU64 = (system_time::now() - start_time).convert();

            display.cls()?;
            display.set_output_line(0)?;
            write!(display, "uptime {}", uptime)?;
        }
        DisplayPage::Off => {
            unimplemented!();
        }
    }

    Ok(())
}

async fn display_handler<Bus>(
    mut display_bus: Bus,
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    charger: &RefCell<BQ24259<Bus>>,
    board: &RefCell<Peripherals>,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    let start_time = system_time::now();

    loop {
        {
            board.borrow_mut().display_power.set_low()?;
            // Give display time to initialize.
            system_time::sleep(Duration::millis(200)).await;

            let mut display = Lcd::new(&mut display_bus)?;
            while page.get() != DisplayPage::Off {
                show_page(page.get(), start_time, &mut display, charger, board).await?;

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

        // If we fell out of previous loop, it means LCD was turned off.
        board.borrow_mut().display_power.set_high()?;

        while page.get() == DisplayPage::Off {
            event.read().await?;
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
        let joystick = &board.borrow_mut().joystick;
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
        let display_handler = pin!(panic_if_exited(display_handler(
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
