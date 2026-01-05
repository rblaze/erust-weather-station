use core::fmt::Write;
use core::pin::pin;

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use futures::{FutureExt, select_biased};
use lcd::screen::Screen;
use rtt_target::debug_rprintln;
use stm32g0_hal::pac::TIM4;
use stm32g0_hal::timer::{Channel1, Channel2, Channel3, PwmPin};

use super::state::{DisplayData, DisplayPage, DisplayState, Power};
use crate::board::{Backlight, DisplayPowerPin};
use crate::error::Error;
use crate::screen::Lcd;
use crate::station_data::{SensorData, StationData};
use crate::system_time::{Duration, sleep};

pub struct View<'a> {
    current_state: DisplayState,
    display: &'a mut Lcd<'a>,
    power_pin: DisplayPowerPin,
    backlight: Option<Backlight>,
}

struct Leds<'a> {
    red: PwmPin<'a, TIM4, Channel1>,
    green: PwmPin<'a, TIM4, Channel2>,
    blue: PwmPin<'a, TIM4, Channel3>,
}

impl<'a> View<'a> {
    pub fn new(display: &'a mut Lcd<'a>, power_pin: DisplayPowerPin, backlight: Backlight) -> Self {
        Self {
            current_state: DisplayState::default(),
            display,
            power_pin,
            backlight: Some(backlight),
        }
    }

    pub async fn task(&mut self, state: &DisplayData, data: &StationData) -> Result<(), Error> {
        let backlight = self.backlight.take().unwrap();
        let mut leds = Leds {
            red: backlight.pwm.bind_pin(backlight.red),
            green: backlight.pwm.bind_pin(backlight.green),
            blue: backlight.pwm.bind_pin(backlight.blue),
        };

        loop {
            let state_waiter = pin!(state.wait_for_update());
            let data_waiter = pin!(data.wait_for_update());

            select_biased! {
                v = state_waiter.fuse() => {
                    match v? {
                        Some(new_state) => self.update_state(&mut leds, &new_state, &data.get()).await?,
                        None => self.update_display(&data.get()).await?,
                    }
                },
                v = data_waiter.fuse() => self.update_display(&v?).await?,
            }
        }
    }

    async fn update_state(
        &mut self,
        leds: &mut Leds<'_>,
        state: &DisplayState,
        data: &SensorData,
    ) -> Result<(), Error> {
        let mut update_display = self.current_state.page != state.page;

        if state.power != self.current_state.power {
            if state.power == Power::On {
                // Turn on display
                self.power_pin.set_low()?;
                sleep(Duration::millis(200)).await;
                self.display.reset()?;
                update_display = true;
            } else {
                // Turn off display
                self.power_pin.set_high()?;
            }
        }

        if state.backlight != self.current_state.backlight {
            debug_rprintln!("backlight {}%", state.backlight.brightness_pct);
            match state.backlight.brightness_pct {
                0 => {
                    leds.red.set_duty_cycle_fully_off()?;
                    leds.green.set_duty_cycle_fully_off()?;
                    leds.blue.set_duty_cycle_fully_off()?;
                }
                100 => {
                    leds.red.set_duty_cycle_fully_on()?;
                    leds.green.set_duty_cycle_fully_on()?;
                    leds.blue.set_duty_cycle_fully_on()?;
                }
                pct => {
                    leds.red.set_duty_cycle_percent(pct)?;
                    leds.green.set_duty_cycle_percent(pct)?;
                    leds.blue.set_duty_cycle_percent(pct)?;
                }
            }
        }

        // Update current_state: update_display() uses it.
        self.current_state = *state;

        if update_display {
            self.update_display(data).await?;
        }

        Ok(())
    }

    async fn update_display(&mut self, data: &SensorData) -> Result<(), Error> {
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

    async fn show_air_data(&mut self, data: &SensorData) -> Result<(), Error> {
        self.display.set_output_line(0)?;
        write!(
            self.display,
            "{} ppm {:.0}\u{df}C {:.0}% RH",
            data.co2_ppm, data.temp_celsius, data.humidity_percent
        )?;

        let uptime_secs = crate::system_time::now()
            .await
            .duration_since_epoch()
            .to_secs();
        let days = uptime_secs / 86400;
        let hours = (uptime_secs % 86400) / 3600;
        let minutes = (uptime_secs % 3600) / 60;
        let seconds = uptime_secs % 60;

        self.display.set_output_line(1)?;
        write!(self.display, "uptime ")?;
        if days > 0 {
            write!(self.display, "{}d ", days)?;
        }
        if days > 0 || hours > 0 {
            write!(self.display, "{}h ", hours)?;
        }
        write!(self.display, "{}m {}s", minutes, seconds)?;

        Ok(())
    }

    async fn show_battery_status(&mut self, data: &SensorData) -> Result<(), Error> {
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

    async fn show_charger_status(&mut self, data: &SensorData) -> Result<(), Error> {
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
