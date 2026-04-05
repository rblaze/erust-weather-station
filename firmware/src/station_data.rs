use core::cell::{Cell, RefCell};

use async_scheduler::mailbox::{Error, Mailbox};
use rtt_target::debug_rprintln;

use crate::time::{Duration, Instant};

pub const HISTORY_SIZE: usize = 120;
pub const HISTORY_INTERVAL: Duration = Duration::secs(300);

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

impl Default for SensorData {
    fn default() -> Self {
        Self {
            co2_ppm: 0,
            temp_celsius: 0.0,
            humidity_percent: 0.0,
            voc_index: None,
            battery_millivolts: 0,
            charger_status: bq24259::registers::SystemStatus::from(0),
            charger_faults: bq24259::registers::NewFault::from(0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HistoryEntry {
    pub timestamp: Instant,
    pub snapshot: SensorData,
}

const _: () = {
    // Verify that HistoryEntry doesn't increase in size if more data
    // is added to SensorData. One way to save 4 bytes is to use
    // Option<NonZero<i32>> for voc_index: it is never less than 1 anyway.
    // It doesn't help now because HistoryEntry is aligned to 8 bytes.
    assert!(core::mem::size_of::<HistoryEntry>() == 32);
};

impl Default for HistoryEntry {
    fn default() -> Self {
        Self {
            timestamp: Instant::from_ticks(0),
            snapshot: SensorData::default(),
        }
    }
}

#[derive(Debug)]
struct HistoryBuffer {
    entries: [HistoryEntry; HISTORY_SIZE],
    head: usize,
    last_recorded: Instant,
}

impl HistoryBuffer {
    fn new() -> Self {
        Self {
            entries: [HistoryEntry::default(); HISTORY_SIZE],
            head: 0,
            last_recorded: Instant::from_ticks(0),
        }
    }

    fn push(&mut self, entry: HistoryEntry) {
        self.entries[self.head] = entry;
        self.head = (self.head + 1) % HISTORY_SIZE;
        self.last_recorded = entry.timestamp;
    }

    fn get_at(&self, timestamp: Instant) -> Option<HistoryEntry> {
        (0..HISTORY_SIZE)
            .map(|i| self.entries[(self.head + HISTORY_SIZE - 1 - i) % HISTORY_SIZE])
            .take_while(|entry| entry.timestamp.ticks() != 0)
            .find(|entry| entry.timestamp <= timestamp)
    }
}

#[derive(Debug)]
pub struct StationData {
    sensor_data: Cell<SensorData>,
    update_event: Mailbox<()>,
    history: RefCell<HistoryBuffer>,
}

impl Default for StationData {
    fn default() -> Self {
        Self::new()
    }
}

impl StationData {
    pub fn new() -> Self {
        Self {
            sensor_data: Cell::default(),
            update_event: Mailbox::new(),
            history: RefCell::new(HistoryBuffer::new()),
        }
    }

    /// Waits for change in sensor data and returns the new data.
    pub async fn wait_for_update(&self) -> Result<(), Error> {
        self.update_event.read().await?;
        Ok(())
    }

    /// Returns current data.
    pub fn sensor_data(&self) -> SensorData {
        self.sensor_data.get()
    }

    pub fn maybe_record_history(&self, timestamp: Instant) {
        let mut history = self.history.borrow_mut();

        if timestamp - history.last_recorded < HISTORY_INTERVAL {
            return;
        }

        let entry = HistoryEntry {
            timestamp,
            snapshot: self.sensor_data(),
        };

        history.push(entry);
        debug_rprintln!("recorded history entry at {}", timestamp);
    }

    /// Returns latest history entry with a timestamp less or equal to the provided one.
    pub fn get_history_at(&self, timestamp: Instant) -> Option<HistoryEntry> {
        self.history.borrow().get_at(timestamp)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn create_entry(ticks: u64) -> HistoryEntry {
        HistoryEntry {
            timestamp: Instant::from_ticks(ticks),
            ..HistoryEntry::default()
        }
    }

    #[test]
    fn test_history_buffer_empty() {
        let buffer = HistoryBuffer::new();
        assert!(buffer.get_at(Instant::from_ticks(100)).is_none());
    }

    #[test]
    fn test_history_buffer_push_and_get_exact() {
        let mut buffer = HistoryBuffer::new();
        let entry = create_entry(100);
        buffer.push(entry);

        let retrieved = buffer.get_at(Instant::from_ticks(100)).unwrap();
        assert_eq!(retrieved.timestamp.ticks(), 100);
    }

    #[test]
    fn test_history_buffer_get_latest_before() {
        let mut buffer = HistoryBuffer::new();
        buffer.push(create_entry(100));
        buffer.push(create_entry(200));
        buffer.push(create_entry(300));

        // Exactly at 200
        assert_eq!(
            buffer
                .get_at(Instant::from_ticks(200))
                .unwrap()
                .timestamp
                .ticks(),
            200
        );
        // Between 200 and 300
        assert_eq!(
            buffer
                .get_at(Instant::from_ticks(250))
                .unwrap()
                .timestamp
                .ticks(),
            200
        );
        // Before 100
        assert!(buffer.get_at(Instant::from_ticks(50)).is_none());
        // After 300
        assert_eq!(
            buffer
                .get_at(Instant::from_ticks(400))
                .unwrap()
                .timestamp
                .ticks(),
            300
        );
    }

    #[test]
    fn test_history_buffer_wrap_around() {
        let mut buffer = HistoryBuffer::new();
        // Fill the buffer and wrap around by 10 entries
        for i in 1..=(HISTORY_SIZE + 10) {
            buffer.push(create_entry(i as u64 * 100));
        }

        // Latest
        assert_eq!(
            buffer
                .get_at(Instant::from_ticks(20000))
                .unwrap()
                .timestamp
                .ticks(),
            13000
        );
        // Oldest still in buffer
        assert_eq!(
            buffer
                .get_at(Instant::from_ticks(1100))
                .unwrap()
                .timestamp
                .ticks(),
            1100
        );
        // Just before the oldest
        assert!(buffer.get_at(Instant::from_ticks(1099)).is_none());
    }
}
