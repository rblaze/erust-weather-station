use core::cell::{Cell, RefCell};

use async_scheduler::mailbox::Mailbox;
use rtt_target::debug_rprintln;

use crate::error::Error;
use crate::system_time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub co2_ppm: u16,
    pub temp_celsius: f32,
    pub humidity_percent: f32,
    pub voc_index: Option<i32>,
    pub battery_millivolts: u16,
    pub charger_status: bq24259::registers::SystemStatus,
    pub charger_faults: bq24259::registers::NewFault,
}

pub const HISTORY_SIZE: usize = 120;

#[derive(Debug, Clone, Copy)]
pub struct HistoryEntry {
    pub timestamp: Instant,
    pub co2_ppm: u16,
    pub temp_celsius: f32,
    pub humidity_percent: f32,
    pub voc_index: Option<i32>,
    pub battery_millivolts: u16,
    pub charger_status: bq24259::registers::SystemStatus,
}

impl Default for HistoryEntry {
    fn default() -> Self {
        Self {
            timestamp: Instant::from_ticks(0),
            co2_ppm: 0,
            temp_celsius: 0.0,
            humidity_percent: 0.0,
            voc_index: None,
            battery_millivolts: 0,
            charger_status: bq24259::registers::SystemStatus::from(0),
        }
    }
}

#[derive(Debug)]
struct HistoryBuffer {
    entries: [HistoryEntry; HISTORY_SIZE],
    head: usize,
    last_recorded: Option<Instant>,
}

impl HistoryBuffer {
    fn new() -> Self {
        Self {
            entries: [HistoryEntry::default(); HISTORY_SIZE],
            head: 0,
            last_recorded: None,
        }
    }

    fn push(&mut self, entry: HistoryEntry) {
        self.entries[self.head] = entry;
        self.head = (self.head + 1) % HISTORY_SIZE;
        self.last_recorded = Some(entry.timestamp);
    }
}

#[derive(Debug)]
pub struct StationData {
    sensor_data: Cell<SensorData>,
    update_event: Mailbox<()>,
    history: RefCell<HistoryBuffer>,
}

impl StationData {
    const HISTORY_INTERVAL: Duration = Duration::secs(300);

    pub fn new() -> Self {
        Self {
            sensor_data: Cell::new(SensorData {
                co2_ppm: 0,
                temp_celsius: 0.0,
                humidity_percent: 0.0,
                voc_index: None,
                battery_millivolts: 0,
                charger_status: bq24259::registers::SystemStatus::from(0),
                charger_faults: bq24259::registers::NewFault::from(0),
            }),
            update_event: Mailbox::new(),
            history: RefCell::new(HistoryBuffer::new()),
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

    pub fn record_history(&self, timestamp: Instant) {
        let mut history = self.history.borrow_mut();

        // No more often than every 5 minutes (300 seconds).
        if let Some(last) = history.last_recorded
            && timestamp - last < Self::HISTORY_INTERVAL
        {
            return;
        }

        let data = self.get();
        let entry = HistoryEntry {
            timestamp,
            co2_ppm: data.co2_ppm,
            temp_celsius: data.temp_celsius,
            humidity_percent: data.humidity_percent,
            voc_index: data.voc_index,
            battery_millivolts: data.battery_millivolts,
            charger_status: data.charger_status,
        };

        history.push(entry);
        debug_rprintln!("recorded history entry at {}s", timestamp);
    }

    /// Returns latest history entry with a timestamp less or equal to the provided one.
    pub fn get_history_at(&self, timestamp: Instant) -> Option<HistoryEntry> {
        let history = self.history.borrow();

        // Search backwards from the most recent entry.
        for i in 0..HISTORY_SIZE {
            let index = (history.head + HISTORY_SIZE - 1 - i) % HISTORY_SIZE;
            let entry = history.entries[index];
            if entry.timestamp.ticks() == 0 {
                // We reached an unused entry, no need to search further.
                break;
            }
            if entry.timestamp <= timestamp {
                return Some(entry);
            }
        }

        None
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

    pub fn set_sgp4x_data(
        &self,
        measurement: &sensirion::scd4x::Measurement,
        voc_index: Option<i32>,
    ) {
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
                voc_index,
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
