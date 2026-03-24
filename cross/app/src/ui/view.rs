use core::fmt::Write;
use core::pin::pin;

use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;
use futures::{FutureExt, select_biased};
use lcd::screen::Screen;
use rtt_target::debug_rprintln;

use firmware::error::Error;
use firmware::types::{Backlight, OnOff};

use super::state::{DisplayData, DisplayPage, DisplayState, Power};
use crate::screen::Lcd;
use crate::station_data::{SensorData, StationData};
use crate::system_time::{Duration, sleep};

pub struct View<'a, I2cBus, DisplayPowerPin, R: SetDutyCycle, G: SetDutyCycle, B: SetDutyCycle> {
    current_state: DisplayState,
    display: Lcd<I2cBus>,
    power_pin: DisplayPowerPin,
    backlight: Backlight<R, G, B>,
    state: &'a DisplayData,
    data: &'a StationData,
}

impl<'a, I2cBus, DisplayPowerPin, R, G, B> View<'a, I2cBus, DisplayPowerPin, R, G, B>
where
    I2cBus: I2c,
    DisplayPowerPin: OnOff,
    R: SetDutyCycle,
    G: SetDutyCycle,
    B: SetDutyCycle,
    Error<I2cBus::Error>: core::convert::From<<R as embedded_hal::pwm::ErrorType>::Error>,
    Error<I2cBus::Error>: core::convert::From<<G as embedded_hal::pwm::ErrorType>::Error>,
    Error<I2cBus::Error>: core::convert::From<<B as embedded_hal::pwm::ErrorType>::Error>,
{
    pub fn new(
        display: Lcd<I2cBus>,
        backlight: Backlight<R, G, B>,
        power_pin: DisplayPowerPin,
        state: &'a DisplayData,
        data: &'a StationData,
    ) -> Self {
        Self {
            current_state: DisplayState::default(),
            display,
            power_pin,
            backlight,
            state,
            data,
        }
    }

    pub async fn task(&mut self) -> Result<(), Error<I2cBus::Error>> {
        loop {
            let state_waiter = pin!(self.state.wait_for_update());
            let data_waiter = pin!(self.data.wait_for_update());

            select_biased! {
                v = state_waiter.fuse() => {
                    match v? {
                        Some(new_state) => self.update_state(&new_state).await?,
                        None => self.update_display(&self.data.get()).await?,
                    }
                },
                v = data_waiter.fuse() => self.update_display(&v?).await?,
            }
        }
    }

    async fn update_state(&mut self, state: &DisplayState) -> Result<(), Error<I2cBus::Error>> {
        let mut update_display = self.current_state.page != state.page;

        if state.power != self.current_state.power {
            if state.power == Power::On {
                // Turn on display
                self.power_pin.on();
                sleep(Duration::millis(200)).await;
                self.display.reset()?;
                update_display = true;
            } else {
                // Turn off display
                self.power_pin.off();
            }
        }

        if state.backlight != self.current_state.backlight {
            debug_rprintln!("backlight {}%", state.backlight.brightness_pct);
            match state.backlight.brightness_pct {
                0 => {
                    self.backlight.red.set_duty_cycle_fully_off()?;
                    self.backlight.green.set_duty_cycle_fully_off()?;
                    self.backlight.blue.set_duty_cycle_fully_off()?;
                }
                100 => {
                    self.backlight.red.set_duty_cycle_fully_on()?;
                    self.backlight.green.set_duty_cycle_fully_on()?;
                    self.backlight.blue.set_duty_cycle_fully_on()?;
                }
                pct => {
                    self.backlight.red.set_duty_cycle_percent(pct)?;
                    self.backlight.green.set_duty_cycle_percent(pct)?;
                    self.backlight.blue.set_duty_cycle_percent(pct)?;
                }
            }
        }

        // Update current_state: update_display() uses it.
        self.current_state = *state;

        if update_display {
            self.update_display(&self.data.get()).await?;
        }

        Ok(())
    }

    async fn update_display(&mut self, data: &SensorData) -> Result<(), Error<I2cBus::Error>> {
        if self.current_state.power == Power::Off {
            // No update needed
            return Ok(());
        }

        self.display.cls()?;
        sleep(Duration::millis(5)).await;

        match self.current_state.page {
            DisplayPage::AirData => self.show_air_data(data).await,
            DisplayPage::BatteryStatus => self.show_battery_status(data).await,
            DisplayPage::ChargerStatus => self.show_charger_status(data).await,
        }
    }

    async fn show_air_data(&mut self, data: &SensorData) -> Result<(), Error<I2cBus::Error>> {
        self.display.set_output_line(0)?;
        write!(
            self.display,
            "{} ppm {:.0}\u{df}C {:.0}% RH",
            data.co2_ppm, data.temp_celsius, data.humidity_percent
        )?;

        self.display.set_output_line(1)?;
        match data.voc_index {
            Some(voc_index) => write!(self.display, "VOC {}", voc_index)?,
            None => write!(self.display, "no VOC")?,
        }

        let uptime_secs = crate::system_time::now()
            .await
            .duration_since_epoch()
            .to_secs();
        write!(self.display, " up {}", uptime_secs)?;

        Ok(())
    }

    async fn show_battery_status(&mut self, data: &SensorData) -> Result<(), Error<I2cBus::Error>> {
        self.display.set_output_line(0)?;
        write!(self.display, "{:?}", data.charger_status.chrg())?;
        self.display.set_output_line(1)?;
        write!(
            self.display,
            "vbat {}.{:02}V",
            data.battery_millivolts / 1000,
            data.battery_millivolts % 1000 / 10
        )?;

        Ok(())
    }

    async fn show_charger_status(&mut self, data: &SensorData) -> Result<(), Error<I2cBus::Error>> {
        self.display.set_output_line(0)?;
        write!(self.display, "status {:08b}", u8::from(data.charger_status))?;

        let fault_bits = u8::from(data.charger_faults);
        self.display.set_output_line(1)?;
        if fault_bits == 0 {
            write!(self.display, "no faults")?;
        } else {
            write!(self.display, "faults {:08b}", fault_bits)?;
        }

        Ok(())
    }
}
