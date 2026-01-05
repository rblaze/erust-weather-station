use embedded_hal::digital::InputPin;

use crate::board::{JOYSTICK_EVENT, Joystick};
use crate::error::Error;
use crate::ui::DisplayData;

pub async fn task(mut joystick: Joystick, state: &DisplayData) -> Result<(), Error> {
    loop {
        JOYSTICK_EVENT.read().await?;

        if joystick.button.is_low()? {
            state.flip_display_power();
        } else if joystick.right.is_low()? {
            state.show_next_page();
        } else if joystick.left.is_low()? {
            state.show_prev_page();
        } else if joystick.select.is_low()? {
            state.set_next_backlight_mode();
        }
    }
}
