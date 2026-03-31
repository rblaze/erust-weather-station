use crate::time::now;
use crate::types::{EventWaiter, Joystick};
use crate::ui::DisplayData;

pub async fn task<J: Joystick + EventWaiter>(mut joystick: J, state: &DisplayData) -> () {
    loop {
        joystick.wait().await;

        if joystick.button() {
            state.flip_display_power();
        } else if joystick.right() {
            state.show_next_page(now().await);
        } else if joystick.left() {
            state.show_prev_page(now().await);
        } else if joystick.up() {
            state.scroll_page_up();
        } else if joystick.down() {
            state.scroll_page_down();
        } else if joystick.select() {
            state.set_next_backlight_mode();
        }
    }
}
