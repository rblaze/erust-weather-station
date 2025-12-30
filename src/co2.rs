use core::cell::Cell;

use rtt_target::debug_rprintln;
use sensirion::scd4x::{Measurement, SCD4x};

use crate::error::Error;
use crate::system_time::{Delay, Duration, sleep};

pub async fn task<Bus>(
    mut sensor: SCD4x<Bus>,
    mut delay: Delay,
    data: &Cell<Measurement>,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<sensirion::Error<Bus::Error>>,
{
    // Reset sensor.
    sensor.stop_periodic_measurement()?;
    sleep(Duration::secs(1)).await;

    debug_rprintln!("SCD4x serial {}", sensor.get_serial_number()?);
    debug_rprintln!("SCD4x type {}", sensor.get_sensor_variant()?);

    let self_test_result = sensor.perform_self_test(&mut delay).await?;
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
            data.set(measurement);
        }
    }
}
