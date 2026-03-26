use bq24259::BQ24259;
use embedded_hal::i2c::I2c;
use futures::FutureExt;
use rtt_target::debug_rprintln;

use firmware::error::{Error, I2cError};
use firmware::types::{Duration, EventWaiter, OnOff, VoltageReader, Watchdog};
use firmware::station_data::StationData;

use crate::system_time::timeout;

pub struct Charger<'a, I2cBus, ChargerEvent, VBat, UsbPower, Wd> {
    charger: BQ24259<I2cBus>,
    charger_event: ChargerEvent,
    vbat: VBat,
    usb_power: UsbPower,
    watchdog: Wd,
    system_data: &'a StationData,
    power_good: bool,
}

impl<'a, I2cBus, ChargerEvent, VBat, UsbPower, Wd>
    Charger<'a, I2cBus, ChargerEvent, VBat, UsbPower, Wd>
where
    I2cBus: I2c,
    ChargerEvent: EventWaiter,
    VBat: VoltageReader,
    UsbPower: OnOff,
    Wd: Watchdog,
{
    pub fn new(
        charger: BQ24259<I2cBus>,
        charger_event: ChargerEvent,
        vbat: VBat,
        usb_power: UsbPower,
        watchdog: Wd,
        system_data: &'a StationData,
    ) -> Self {
        Self {
            charger,
            charger_event,
            vbat,
            usb_power,
            watchdog,
            system_data,
            power_good: false,
        }
    }

    pub async fn task(&mut self) -> Result<(), Error<I2cBus::Error>> {
        self.watchdog.feed();
        self.charger.reset_watchdog().map_err(I2cError)?;

        self.update_battery_state();
        self.power_good = self.update_charger_state()?;
        self.report_power_state();

        loop {
            // Default charger watchdog timeout is 40 seconds.
            // Watchdog timeout is 32.7 seconds.
            let power_event = timeout(
                Duration::secs(31),
                self.charger_event
                    .wait()
                    .map(Ok::<(), Error<I2cBus::Error>>),
            )
            .await?
            .is_some();

            debug_rprintln!("charger watchdog reset");

            self.watchdog.feed();
            self.charger.reset_watchdog().map_err(I2cError)?;
            self.update_battery_state();

            if power_event {
                let power_good = self.update_charger_state()?;
                if power_good != self.power_good {
                    self.power_good = power_good;
                    self.report_power_state();
                }
            }
        }
    }

    fn report_power_state(&mut self) {
        if self.power_good {
            self.usb_power.on();
        } else {
            self.usb_power.off();
        }
    }

    fn update_charger_state(&mut self) -> Result<bool, I2cError<I2cBus::Error>> {
        let charger_status = self.charger.status()?;
        self.system_data.set_charger_status(charger_status);

        // First read returns fault that triggered the interrupt.
        self.charger.new_fault()?;
        // Second read returns current state.
        self.system_data
            .set_charger_faults(self.charger.new_fault()?);

        Ok(charger_status.pg())
    }

    fn update_battery_state(&mut self) {
        self.system_data
            .set_battery_millivolts(self.vbat.millivolts());
    }
}
