use core::cell::Cell;

use async_scheduler::mailbox::{Error, Mailbox};

use crate::station_data::HISTORY_INTERVAL;
use crate::time::{Duration, Instant, timeout};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Power {
    On,
    Off,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DisplayPage {
    AirData,
    BatteryStatus,
    ChargerStatus,
    History(Instant),
}

impl DisplayPage {
    pub fn next(self, time: Instant) -> Self {
        use DisplayPage::*;
        match self {
            AirData => BatteryStatus,
            BatteryStatus => ChargerStatus,
            ChargerStatus => History(time),
            History(_) => AirData,
        }
    }

    pub fn prev(self, time: Instant) -> Self {
        use DisplayPage::*;
        match self {
            AirData => History(time),
            BatteryStatus => AirData,
            ChargerStatus => BatteryStatus,
            History(_) => ChargerStatus,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Backlight {
    pub brightness_pct: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DisplayState {
    pub power: Power,
    pub backlight: Backlight,
    pub page: DisplayPage,
}

impl Default for DisplayState {
    fn default() -> Self {
        Self {
            power: Power::Off,
            backlight: Backlight { brightness_pct: 0 },
            page: DisplayPage::AirData,
        }
    }
}

#[derive(Debug)]
pub struct DisplayData {
    state: Cell<DisplayState>,
    update_event: Mailbox<()>,
}

impl Default for DisplayData {
    fn default() -> Self {
        Self::new()
    }
}

impl DisplayData {
    pub fn new() -> Self {
        Self {
            state: Cell::new(DisplayState::default()),
            update_event: Mailbox::new(),
        }
    }

    /// Waits for change in display state and returns the new state.
    /// Returns Ok(None) early if current state requires periodic updates and
    /// timeout expires.
    pub async fn wait_for_update(&self) -> Result<Option<DisplayState>, Error> {
        let state = self.state.get();

        if state.power == Power::On && state.page == DisplayPage::AirData {
            // AirData page has uptime information
            Ok(timeout(Duration::secs(1), self.update_event.read())
                .await
                .map(|opt| opt.map(|()| self.state.get()))?)
        } else {
            Ok(self
                .update_event
                .read()
                .await
                .map(|()| Some(self.state.get()))?)
        }
    }

    pub fn set_display_power(&self, power: Power) {
        self.state.update(|state| {
            if state.power != power {
                self.update_event.post(());
            }

            DisplayState { power, ..state }
        });
    }

    pub fn flip_display_power(&self) {
        self.state.update(|state| DisplayState {
            power: if state.power == Power::On {
                Power::Off
            } else {
                Power::On
            },
            ..state
        });

        self.update_event.post(());
    }

    pub fn show_next_page(&self, now: Instant) {
        self.state.update(|state| DisplayState {
            page: state.page.next(now),
            ..state
        });

        self.update_event.post(());
    }

    pub fn show_prev_page(&self, now: Instant) {
        self.state.update(|state| DisplayState {
            page: state.page.prev(now),
            ..state
        });

        self.update_event.post(());
    }

    pub fn scroll_page_up(&self, now: Instant) {
        self.state.update(|state| {
            if let DisplayPage::History(timestamp) = state.page {
                self.update_event.post(());

                let new_time = timestamp
                    .checked_add_duration(HISTORY_INTERVAL)
                    .unwrap_or(Instant::from_ticks(0))
                    .min(now);
                DisplayState {
                    page: DisplayPage::History(new_time),
                    ..state
                }
            } else {
                // Do nothing
                state
            }
        })
    }

    pub fn scroll_page_down(&self, now: Instant) {
        self.state.update(|state| {
            if let DisplayPage::History(timestamp) = state.page {
                self.update_event.post(());

                let new_time = timestamp
                    .checked_sub_duration(HISTORY_INTERVAL)
                    .unwrap_or(Instant::from_ticks(0))
                    .min(now);
                DisplayState {
                    page: DisplayPage::History(new_time),
                    ..state
                }
            } else {
                // Do nothing
                state
            }
        })
    }

    pub fn set_next_backlight_mode(&self) {
        self.state.update(|state| {
            let new_pct = if state.backlight.brightness_pct == 0 {
                50
            } else if state.backlight.brightness_pct == 50 {
                100
            } else {
                0
            };

            DisplayState {
                backlight: Backlight {
                    brightness_pct: new_pct,
                },
                ..state
            }
        });

        self.update_event.post(());
    }
}
