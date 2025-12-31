use core::cell::Cell;

use async_scheduler::mailbox::Mailbox;
use embedded_hal::pwm::SetDutyCycle;
use rtt_target::debug_rprintln;

use crate::{board::Backlight, error::Error};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DisplayBacklight {
    Off,
    Half,
    Full,
}

impl DisplayBacklight {
    pub fn next(self) -> Self {
        use DisplayBacklight::*;
        match self {
            Off => Half,
            Half => Full,
            Full => Off,
        }
    }
}

pub async fn backlight_handler(
    backlight: Backlight,
    state: &Cell<DisplayBacklight>,
    event: &Mailbox<()>,
) -> Result<(), Error> {
    let mut red = backlight.pwm.bind_pin(backlight.red);
    let mut green = backlight.pwm.bind_pin(backlight.green);
    let mut blue = backlight.pwm.bind_pin(backlight.blue);

    loop {
        debug_rprintln!("backlight {:?}", state.get());
        match state.get() {
            DisplayBacklight::Off => {
                red.set_duty_cycle_fully_off()?;
                green.set_duty_cycle_fully_off()?;
                blue.set_duty_cycle_fully_off()?;
            }
            DisplayBacklight::Half => {
                red.set_duty_cycle_fraction(1, 2)?;
                green.set_duty_cycle_fraction(1, 2)?;
                blue.set_duty_cycle_fraction(1, 2)?;
            }
            DisplayBacklight::Full => {
                red.set_duty_cycle_fully_on()?;
                green.set_duty_cycle_fully_on()?;
                blue.set_duty_cycle_fully_on()?;
            }
        };

        event.read().await?;
    }
}
