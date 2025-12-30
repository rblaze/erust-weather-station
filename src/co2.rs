use rtt_target::debug_rprintln;
use sensirion::scd4x::SCD4x;

use crate::error::Error;
use crate::system_time::{Delay, Duration, sleep};

pub async fn task<Bus>(mut sensor: SCD4x<Bus>, mut delay: Delay) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<sensirion::Error<Bus::Error>>,
{
    debug_rprintln!("SCD4x serial {}", sensor.get_serial_number()?);
    debug_rprintln!("SCD4x type {}", sensor.get_sensor_variant()?);

    let self_test_result = sensor.perform_self_test(&mut delay).await?;
    debug_rprintln!(
        "SCD4x self test {}",
        if self_test_result { "passed" } else { "failed" }
    );

    loop {
        sleep(Duration::secs(60)).await;
    }
}
