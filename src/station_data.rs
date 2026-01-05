use core::cell::Cell;

use async_scheduler::mailbox::Mailbox;

use crate::error::Error;

#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub co2_ppm: u16,
    pub temp_celsius: f32,
    pub humidity_percent: f32,
    pub battery_millivolts: u16,
    pub charger_status: bq24259::registers::SystemStatus,
    pub charger_faults: bq24259::registers::NewFault,
}

#[derive(Debug)]
pub struct StationData {
    sensor_data: Cell<SensorData>,
    update_event: Mailbox<()>,
}

impl StationData {
    pub fn new() -> Self {
        Self {
            sensor_data: Cell::new(SensorData {
                co2_ppm: 0,
                temp_celsius: 0.0,
                humidity_percent: 0.0,
                battery_millivolts: 0,
                charger_status: bq24259::registers::SystemStatus::from(0),
                charger_faults: bq24259::registers::NewFault::from(0),
            }),
            update_event: Mailbox::new(),
        }
    }

    /// Waits for change in sensor data and returns the new data.
    pub async fn wait_for_update(&self) -> Result<SensorData, Error> {
        self.update_event.read().await?;
        Ok(self.sensor_data.get())
    }

    /// Returns current data.
    pub fn get(&self) -> SensorData {
        self.sensor_data.get()
    }

    pub fn set_charger_status(&self, status: bq24259::registers::SystemStatus) {
        self.sensor_data.update(|data| {
            if data.charger_status != status {
                self.update_event.post(());
            }

            SensorData {
                charger_status: status,
                ..data
            }
        });
    }

    pub fn set_charger_faults(&self, faults: bq24259::registers::NewFault) {
        self.sensor_data.update(|data| {
            if data.charger_faults != faults {
                self.update_event.post(());
            }

            SensorData {
                charger_faults: faults,
                ..data
            }
        });
    }

    pub fn set_sgp4x_data(&self, measurement: &sensirion::scd4x::Measurement) {
        self.sensor_data.update(|data| {
            if data.co2_ppm != measurement.co2_ppm
                || data.temp_celsius != measurement.temp_celsius
                || data.humidity_percent != measurement.humidity_percent
            {
                self.update_event.post(());
            }

            SensorData {
                co2_ppm: measurement.co2_ppm,
                temp_celsius: measurement.temp_celsius,
                humidity_percent: measurement.humidity_percent,
                ..data
            }
        });
    }

    pub fn set_battery_millivolts(&self, millivolts: u16) {
        self.sensor_data.update(|data| {
            if data.battery_millivolts != millivolts {
                self.update_event.post(());
            }

            SensorData {
                battery_millivolts: millivolts,
                ..data
            }
        });
    }
}
