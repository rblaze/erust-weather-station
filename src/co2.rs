use core::fmt::Write;
use rtt_target::debug_rprintln;
use sensirion::scd4x::SCD4x;
use sensirion::sgp40::SGP40;

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
    mut co2_sensor: SCD4x<SharedI2cBus<'_>>,
    mut voc_sensor: SGP40<SharedI2cBus<'_>>,
    serial: &UsbSerialPort,
    system_data: &StationData,
) -> Result<(), Error> {
    // Reset sensor.
    co2_sensor.stop_periodic_measurement()?;
    sleep(Duration::secs(1)).await;

    debug_rprintln!("SCD4x serial {}", co2_sensor.get_serial_number()?);
    debug_rprintln!("SCD4x type {}", co2_sensor.get_sensor_variant()?);
    debug_rprintln!("SGP40 serial {}", voc_sensor.get_serial_number()?);

    voc_sensor.start_self_test()?;
    co2_sensor.start_self_test()?;

    sleep(Duration::millis(350)).await;
    let voc_self_test_result = voc_sensor.read_self_test_result()?;
    debug_rprintln!(
        "SGP40 self test {}",
        if voc_self_test_result { "passed" } else { "failed" }
    );

    sleep(Duration::secs(10)).await;
    let co2_self_test_result = co2_sensor.read_self_test_result()?;
    debug_rprintln!(
        "SCD4x self test {}",
        if co2_self_test_result { "passed" } else { "failed" }
    );

    co2_sensor.start_low_power_periodic_measurement()?;

    loop {
        sleep(Duration::secs(30)).await;
        if co2_sensor.get_data_ready_status()? {
            let measurement = co2_sensor.read_measurement()?;
            debug_rprintln!("{}", measurement);
            system_data.set_sgp4x_data(&measurement);

            // Send data to USB serial port
            let mut buf = PrintBuf::<128>::new();
            write!(buf, "{}\r\n", measurement)?;
            serial.write(buf.as_slice())?;
        }
    }
}
