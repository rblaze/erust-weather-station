#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod board;
mod system_time;

use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use bq24259::BQ24259;
use cortex_m_rt::entry;
use embedded_hal_bus::i2c::RefCellDevice;
use futures::task::LocalFutureObj;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use sensirion::scd4x::SCD4x;
use sensirion::sgp40::SGP40;

use firmware::buttons;
use firmware::charger::Charger;
use firmware::co2;
use firmware::error::Error;
use firmware::screen::Lcd;
use firmware::station_data::StationData;
use firmware::ui::{DisplayData, Power, View};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    debug_rprintln!("{}", info);

    cortex_m::asm::bkpt();
    cortex_m::asm::udf();
}

async fn panic_if_exited<F: core::future::Future<Output: core::fmt::Debug>>(f: F) {
    panic!("future exited with {:?}", f.await)
}

#[entry]
fn main() -> ! {
    move || -> Result<(), Error<board::I2cError>> {
        #[cfg(debug_assertions)]
        rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 4096);

        debug_rprintln!("starting");

        let (env, board) = board::init()?;

        let station_data = StationData::new();
        let screen_state = DisplayData::new();

        screen_state.set_display_power(Power::On);

        let lcd = Lcd::new(RefCellDevice::new(&board.i2c));
        let mut lcd_view = View::new(
            lcd,
            board.backlight,
            board.display_power,
            &screen_state,
            &station_data,
        );

        let charger_device = BQ24259::new(RefCellDevice::new(&board.i2c));
        let mut charger = Charger::new(
            charger_device,
            board.charger_event,
            board.vbat,
            board.usb_power,
            board.watchdog,
            &station_data,
        );

        let voc_sensor = SGP40::new(RefCellDevice::new(&board.i2c));
        let co2_sensor = SCD4x::new(RefCellDevice::new(&board.i2c));

        LocalExecutor::new(&env).run([
            LocalFutureObj::new(pin!(panic_if_exited(lcd_view.task()))),
            LocalFutureObj::new(pin!(panic_if_exited(charger.task()))),
            LocalFutureObj::new(pin!(panic_if_exited(co2::task(
                co2_sensor,
                voc_sensor,
                &board.usb_serial,
                &station_data
            )))),
            LocalFutureObj::new(pin!(panic_if_exited(buttons::task(
                board.joystick,
                &screen_state
            )))),
        ]);

        // Tasks are running forever.
        unreachable!();
    }()
    .expect("error in main");

    unreachable!();
}
