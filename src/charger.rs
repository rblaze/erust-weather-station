use core::cell::RefCell;

use async_scheduler::mailbox::Mailbox;
use bq24259::BQ24259;
use rtt_target::debug_rprintln;

use crate::error::Error;
use crate::system_time::{Duration, timeout};

fn set_power_state(on_external_power: bool) {
    if on_external_power {
        crate::board::Board::on_external_power();
    } else {
        crate::board::Board::on_battery();
    }
}

pub async fn task<Bus>(
    charger: &RefCell<BQ24259<Bus>>,
    power_event: &Mailbox<()>,
) -> Result<(), Error>
where
    Bus: embedded_hal::i2c::I2c,
    Error: From<Bus::Error>,
{
    charger.borrow_mut().reset_watchdog()?;
    let mut last_power_state = charger.borrow_mut().status()?.pg();
    set_power_state(last_power_state);

    loop {
        // Default charger watchdog timeout is 40 seconds.
        if timeout(Duration::secs(37), crate::board::CHARGER_EVENT.read())
            .await?
            .is_some()
        {
            // Charger interrupt occurred, refresh display.
            power_event.post(());
        }

        debug_rprintln!("charger watchdog");
        charger.borrow_mut().reset_watchdog()?;

        let power_state = charger.borrow_mut().status()?.pg();
        if power_state != last_power_state {
            set_power_state(power_state);
            last_power_state = power_state;
        }
    }
}
