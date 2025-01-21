#![deny(unsafe_code)]

use core::cell::{Cell, RefCell};
use core::fmt::Write;

use async_scheduler::mailbox::Mailbox;
use bq24259::BQ24259;
use embedded_hal::digital::OutputPin;
use fugit::SecsDurationU64;
use lcd::screen::Screen;
use rtt_target::debug_rprintln;

use crate::board::{DisplayPowerPin, VBat};
use crate::error::Error;
use crate::screen::Lcd;
use crate::system_time::{self, sleep, Duration, Instant};

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
    vbat: &mut VBat,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    debug_rprintln!("display loop");
    match page {
        DisplayPage::BatteryStatus => {
            let status = charger.borrow_mut().status()?;
            let battery_volts = vbat.read_battery_volts();

            display.cls()?;
            sleep(Duration::millis(5)).await;

            display.set_output_line(0)?;
            write!(display, "{:?}", status.chrg())?;
            display.set_output_line(1)?;
            write!(display, "vbat {:.2}", battery_volts)?;
        }
        DisplayPage::ChargerRegisters => {
            let (status, faults) = {
                let mut ch = charger.borrow_mut();

                // In order to read the current fault status, the host has to read REG09 two
                // times consecutively. The 1st reads fault register status from
                // the last read and the 2nd reads the current fault register status.
                ch.new_fault()?;

                (ch.status()?, ch.new_fault()?)
            };

            debug_rprintln!("status {:?}", status);
            debug_rprintln!("faults {:?}", faults);

            display.cls()?;
            sleep(Duration::millis(5)).await;

            display.set_output_line(0)?;
            write!(display, "status {:08b}", u8::from(status))?;
            display.set_output_line(1)?;
            write!(display, "faults {:08b}", u8::from(faults))?;
        }
        DisplayPage::SystemStatus => {
            let uptime: SecsDurationU64 = (system_time::now() - start_time).convert();

            display.cls()?;
            sleep(Duration::millis(5)).await;

            display.set_output_line(0)?;
            write!(display, "uptime {}", uptime)?;
        }
        DisplayPage::Off => {
            unimplemented!();
        }
    }

    Ok(())
}

// Update page every 2 seconds or on event until display is turned off or error occurs.
async fn page_loop<Bus>(
    display_bus: &mut Bus,
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    charger: &RefCell<BQ24259<Bus>>,
    vbat: &mut VBat,
    display_power: &mut DisplayPowerPin,
    start_time: Instant,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    display_power.set_low()?;
    // Give display time to initialize.
    system_time::sleep(Duration::millis(200)).await;

    let mut display = Lcd::new(display_bus)?;
    while page.get() != DisplayPage::Off {
        show_page(page.get(), start_time, &mut display, charger, vbat).await?;

        if page.get() == DisplayPage::ChargerRegisters {
            // On this page, update only by charger interrupt or page change.
            event.read().await?;
        } else {
            system_time::timeout(Duration::secs(2), event.read()).await?;
        }
    }

    Ok(())
}

// Task responsible for updating display.
pub async fn task<Bus>(
    mut display_bus: Bus,
    event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    charger: &RefCell<BQ24259<Bus>>,
    vbat: &mut VBat,
    display_power: &mut DisplayPowerPin,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    let start_time = system_time::now();

    loop {
        let result = page_loop(
            &mut display_bus,
            event,
            page,
            charger,
            vbat,
            display_power,
            start_time,
        )
        .await;

        // If we fell out of previous loop, it means LCD was turned off.
        display_power.set_high()?;

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
