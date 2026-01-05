use core::fmt::Write;
use rtt_target::debug_rprintln;
use sensirion::scd4x::SCD4x;

use crate::board::{SharedI2cBus, UsbSerialPort};
use crate::error::Error;
use crate::station_data::StationData;
use crate::system_time::{Duration, sleep};

#[derive(Debug, Clone, Copy)]
struct PrintBuf<const N: usize> {
    buffer: [u8; N],
    offset: usize,
}

impl<const N: usize> PrintBuf<N> {
    fn new() -> Self {
        Self {
            buffer: [0; N],
            offset: 0,
        }
    }

    fn as_slice(&self) -> &[u8] {
        &self.buffer[..self.offset]
    }
}

impl<const N: usize> Write for PrintBuf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.as_bytes() {
            if self.offset >= N {
                break;
            }
            self.buffer[self.offset] = *byte;
            self.offset += 1;
        }

        Ok(())
    }
}

pub async fn task(
    mut sensor: SCD4x<SharedI2cBus<'_>>,
    serial: &UsbSerialPort,
    system_data: &StationData,
) -> Result<(), Error> {
    // Reset sensor.
    sensor.stop_periodic_measurement()?;
    sleep(Duration::secs(1)).await;

    debug_rprintln!("SCD4x serial {}", sensor.get_serial_number()?);
    debug_rprintln!("SCD4x type {}", sensor.get_sensor_variant()?);

    sensor.start_self_test()?;
    sleep(Duration::secs(10)).await;
    let self_test_result = sensor.read_self_test_result()?;
    debug_rprintln!(
        "SCD4x self test {}",
        if self_test_result { "passed" } else { "failed" }
    );

    sensor.start_low_power_periodic_measurement()?;

    loop {
        sleep(Duration::secs(30)).await;
        if sensor.get_data_ready_status()? {
            let measurement = sensor.read_measurement()?;
            debug_rprintln!("{}", measurement);
            system_data.set_sgp4x_data(&measurement);

            // Send data to USB serial port
            let mut buf = PrintBuf::<128>::new();
            write!(buf, "{}\r\n", measurement)?;
            serial.write(buf.as_slice())?;
        }
    }
}
