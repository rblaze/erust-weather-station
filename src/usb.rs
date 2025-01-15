use async_scheduler::mailbox::Mailbox;
use futures::{select_biased, FutureExt};
use rtt_target::debug_rprintln;

use crate::board::{MuxtexWithUsb, UsbSerialPort};
use crate::error::Error;

pub async fn task(
    pg_event: &Mailbox<bool>,
    device: &MuxtexWithUsb,
    serial: UsbSerialPort,
) -> Result<(), Error> {
    let initial_power_state = pg_event.read().await?;

    if !initial_power_state {
        usb_poweroff_loop(pg_event, device).await?;
    }

    loop {
        usb_active_loop(pg_event, &serial).await?;
        usb_poweroff_loop(pg_event, device).await?;
    }
}

async fn usb_poweroff_loop(pg_event: &Mailbox<bool>, device: &MuxtexWithUsb) -> Result<(), Error> {
    debug_rprintln!("USB power off");
    critical_section::with(|cs| {
        device.borrow_ref(cs).bus().disable_interrupts();
        device.borrow_ref(cs).bus().power_down();
    });

    while !pg_event.read().await? {
        // Wait for power to be connected
    }

    critical_section::with(|cs| {
        device.borrow_ref(cs).bus().power_up();
        device.borrow_ref(cs).bus().enable_interrupts();
    });

    Ok(())
}

async fn usb_active_loop(pg_event: &Mailbox<bool>, serial: &UsbSerialPort) -> Result<(), Error> {
    debug_rprintln!("USB power on");

    let mut buf = [0u8; 40];
    let mut total_bytes = 0;

    loop {
        select_biased! {
            pg = pg_event.read().fuse() => if !(pg?) { break },
            read_result = serial.read(&mut buf).fuse() => match read_result {
                Ok(bytes_read) => {
                    total_bytes += bytes_read;
                    debug_rprintln!("serial read ({}): {:?}", total_bytes, &buf[..bytes_read]);
                }
                Err(e) => {
                    debug_rprintln!("serial read error: {:?}", e);
                }
            },
        };
    }

    Ok(())
}
