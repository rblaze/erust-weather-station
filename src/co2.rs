use core::fmt::Write;
use rtt_target::debug_rprintln;
use sensirion::scd4x::SCD4x;
use sensirion::sgp40::SGP40;
use sensirion_gas_index_algorithm_rs::{AlgorithmType, GasIndexAlgorithm};

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

const SAMPLING_INTERVAL: Duration = Duration::secs(30);

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
        if voc_self_test_result {
            "passed"
        } else {
            "failed"
        }
    );

    sleep(Duration::secs(10)).await;
    let co2_self_test_result = co2_sensor.read_self_test_result()?;
    debug_rprintln!(
        "SCD4x self test {}",
        if co2_self_test_result {
            "passed"
        } else {
            "failed"
        }
    );

    co2_sensor.start_low_power_periodic_measurement()?;

    let mut voc_alg = GasIndexAlgorithm::with_sampling_interval(
        AlgorithmType::Voc,
        SAMPLING_INTERVAL.to_secs() as f32,
    );

    loop {
        sleep(SAMPLING_INTERVAL).await;
        if co2_sensor.get_data_ready_status()? {
            let co2_measurement = co2_sensor.read_measurement()?;
            debug_rprintln!("{:?}", co2_measurement);

            voc_sensor.start_measure_raw_signal_with_ticks(
                co2_measurement.humidity_raw,
                co2_measurement.temp_raw,
            )?;
            sleep(Duration::millis(30)).await;
            let voc_measurement = voc_sensor.read_measure_raw_signal_result()?;
            let voc_index = voc_alg.process(voc_measurement);
            debug_rprintln!("VOC index: {:?}, raw {}", voc_index, voc_measurement);

            system_data.set_sgp4x_data(&co2_measurement, voc_index);

            // Send data to USB serial port
            let mut buf = PrintBuf::<128>::new();
            write!(buf, "{}\r\n", co2_measurement)?;
            serial.write(buf.as_slice())?;
        }
    }
}
