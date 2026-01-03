#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod backlight;
mod board;
mod charger;
mod co2;
mod display;
mod env;
mod error;
mod screen;
mod system_time;
mod usb;

use core::cell::{Cell, RefCell};
use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use async_scheduler::mailbox::Mailbox;
use backlight::backlight_handler;
use board::{JOYSTICK_EVENT, Joystick};
use bq24259::BQ24259;
use cortex_m_rt::entry;
use embedded_hal::digital::InputPin;
use embedded_hal_bus::i2c::RefCellDevice;
use futures::task::LocalFutureObj;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use sensirion::scd4x::SCD4x;
use sensirion::sgp40::SGP40;

use crate::backlight::DisplayBacklight;
use crate::display::DisplayPage;
use crate::error::Error;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    debug_rprintln!("{}", info);

    cortex_m::asm::bkpt();
    cortex_m::asm::udf();
}

async fn panic_if_exited<F: core::future::Future<Output = Result<(), Error>>>(f: F) {
    panic!("future exited with {:?}", f.await)
}

async fn navigation(
    display_event: &Mailbox<()>,
    page: &Cell<DisplayPage>,
    backlight_event: &Mailbox<()>,
    backlight: &Cell<DisplayBacklight>,
    joystick: &mut Joystick,
) -> Result<(), Error> {
    let mut last_visible_page = None;

    loop {
        // Wait for button press
        JOYSTICK_EVENT.read().await?;

        let current_page = page.get();
        if joystick.up.is_low()? {
            page.set(current_page.prev());
        } else if joystick.down.is_low()? {
            page.set(current_page.next());
        } else if joystick.button.is_low()? {
            match last_visible_page {
                Some(last_page) => {
                    last_visible_page = None;
                    page.set(last_page);
                }
                None => {
                    last_visible_page = Some(current_page);
                    backlight.set(DisplayBacklight::Off);
                    backlight_event.post(());
                    page.set(DisplayPage::Off);
                }
            }
        } else if page.get() != DisplayPage::Off && joystick.select.is_low()? {
            backlight.set(backlight.get().next());
            backlight_event.post(());
        }

        display_event.post(());
    }
}

#[entry]
fn main() -> ! {
    move || -> Result<(), Error> {
        #[cfg(debug_assertions)]
        rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 4096);

        debug_rprintln!("starting");

        let mut board = board::Board::new()?;

        let backlight_event = Mailbox::<()>::new();
        let backlight = Cell::new(DisplayBacklight::Off);
        let display_refresh_event = Mailbox::<()>::new();
        let display_page = Cell::new(DisplayPage::BatteryStatus);
        let display_bus = RefCellDevice::new(&board.i2c);
        let charger = RefCell::new(BQ24259::new(RefCellDevice::new(&board.i2c)));
        let mut voc_sensor = SGP40::new(RefCellDevice::new(&board.i2c));
        let co2_sensor = SCD4x::new(RefCellDevice::new(&board.i2c));
        let co2_data = Cell::new(sensirion::scd4x::Measurement::default());

        match voc_sensor.get_serial_number() {
            Ok(serial) => debug_rprintln!("SGP40 serial {}", serial),
            Err(error) => debug_rprintln!("SGP40 error: {}", error),
        }

        let charger_watchdog = pin!(panic_if_exited(charger::task(
            &charger,
            &display_refresh_event
        )));
        let backlight_handler = pin!(panic_if_exited(backlight_handler(
            board.backlight,
            &backlight,
            &backlight_event
        )));
        let display_handler = pin!(panic_if_exited(display::task(
            display_bus,
            &display_refresh_event,
            &display_page,
            &co2_data,
            &charger,
            &mut board.vbat,
            &mut board.display_power,
        )));
        let navigation = pin!(panic_if_exited(navigation(
            &display_refresh_event,
            &display_page,
            &backlight_event,
            &backlight,
            &mut board.joystick
        )));
        let usb = pin!(panic_if_exited(usb::task(board.usb_serial)));
        let co2_reader = pin!(panic_if_exited(co2::task(
            co2_sensor,
            board.delay,
            &co2_data
        )));

        let env = env::Env::new(board.ticker);
        LocalExecutor::new(&env).run([
            LocalFutureObj::new(charger_watchdog),
            LocalFutureObj::new(navigation),
            LocalFutureObj::new(display_handler),
            LocalFutureObj::new(backlight_handler),
            LocalFutureObj::new(usb),
            LocalFutureObj::new(co2_reader),
        ]);

        // Tasks are running forever.
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
