#![deny(unsafe_code)]

use core::cell::{Cell, RefCell};
use core::fmt::Write;
use core::pin::pin;

use async_scheduler::mailbox::Mailbox;
use bq24259::BQ24259;
use embedded_hal::digital::OutputPin;
use fugit::SecsDurationU64;
use futures::future::try_select;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;

use crate::board::Peripherals;
use crate::error::Error;
use crate::screen::Lcd;
use crate::system_time::{self, Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DisplayPage {
    BatteryStatus,
    ChargerRegisters,
    SystemStatus,
    Off,
}

impl DisplayPage {
    pub fn next(self) -> Self {
        use DisplayPage::*;
        match self {
            BatteryStatus => ChargerRegisters,
            ChargerRegisters => SystemStatus,
            SystemStatus => BatteryStatus,
            Off => Off,
        }
    }

    pub fn prev(self) -> Self {
        use DisplayPage::*;
        match self {
            BatteryStatus => SystemStatus,
            ChargerRegisters => BatteryStatus,
            SystemStatus => ChargerRegisters,
            Off => Off,
        }
    }
}

// Update page once.
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

// Update page every 2 seconds until display is turned off or error occurs.
async fn page_loop<Bus>(
    display_bus: &mut Bus,
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    charger: &RefCell<BQ24259<Bus>>,
    board: &RefCell<Peripherals>,
    start_time: Instant,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    board.borrow_mut().display_power.set_low()?;
    // Give display time to initialize.
    system_time::sleep(Duration::millis(200)).await;

    let mut display = Lcd::new(display_bus)?;
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

    Ok(())
}

// Task responsible for updating display.
pub async fn task<Bus>(
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
        let result = page_loop(&mut display_bus, event, page, charger, board, start_time).await;

        // If we fell out of previous loop, it means LCD was turned off.
        board.borrow_mut().display_power.set_high()?;

        match result {
            Ok(()) => {
                while page.get() == DisplayPage::Off {
                    event.read().await?;
                }
            }
            Err(e) => {
                // On error, power down display, wait a bit and try again.
                debug_rprintln!("page loop error: {:?}", e);
                system_time::sleep(Duration::millis(500)).await;
            }
        }
    }
}
