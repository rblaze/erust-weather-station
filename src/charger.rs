use bq24259::BQ24259;
use rtt_target::debug_rprintln;

use crate::board::{SharedI2cBus, VBat};
use crate::error::Error;
use crate::station_data::StationData;
use crate::system_time::{Duration, timeout};

struct Charger<'a> {
    charger: BQ24259<SharedI2cBus<'a>>,
    vbat: VBat,
    system_data: &'a StationData,
    power_good: bool,
}

impl<'a> Charger<'a> {
    fn new(
        mut charger: BQ24259<SharedI2cBus<'a>>,
        vbat: VBat,
        system_data: &'a StationData,
    ) -> Result<Self, Error> {
        charger.reset_watchdog()?;

        Ok(Self {
            charger,
            vbat,
            system_data,
            power_good: false,
        })
    }

    fn report_power_state(&self) {
        if self.power_good {
            crate::board::Board::on_external_power();
        } else {
            crate::board::Board::on_battery();
        }
    }

    fn update_charger_state(&mut self) -> Result<bool, Error> {
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
            .set_battery_millivolts(self.vbat.read_battery_millivolts());
    }

    fn do_work(&mut self, power_event: bool) -> Result<(), Error> {
        debug_rprintln!("charger watchdog reset");

        self.charger.reset_watchdog()?;
        self.update_battery_state();

        if power_event {
            let power_good = self.update_charger_state()?;
            if power_good != self.power_good {
                self.power_good = power_good;
                self.report_power_state();
            }
        }

        Ok(())
    }
}

pub async fn task(
    charger: BQ24259<SharedI2cBus<'_>>,
    vbat: VBat,
    system_data: &StationData,
) -> Result<(), Error> {
    let mut ch = Charger::new(charger, vbat, system_data)?;

    ch.update_battery_state();
    ch.power_good = ch.update_charger_state()?;
    ch.report_power_state();

    loop {
        // Default charger watchdog timeout is 40 seconds.
        let power_event = timeout(Duration::secs(37), crate::board::CHARGER_EVENT.read())
            .await?
            .is_some();

        ch.do_work(power_event)?;
    }
}
