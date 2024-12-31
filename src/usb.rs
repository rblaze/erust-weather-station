use async_scheduler::mailbox::Mailbox;
use futures::{select_biased, FutureExt};
use rtt_target::debug_rprintln;

use crate::board::{MutexWithSerialPort, MuxtexWithUsb, USB_EVENT};
use crate::error::Error;

pub async fn task(
    pg_event: &Mailbox<bool>,
    device: &MuxtexWithUsb,
    serial: &MutexWithSerialPort,
) -> Result<(), Error> {
    let initial_power_state = pg_event.read().await?;

    if !initial_power_state {
        usb_poweroff_loop(pg_event, device).await?;
    }

    loop {
        usb_active_loop(pg_event, serial).await?;
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

enum UsbEvent {
    PoweredDown,
    DataReady,
    Nothing,
}

async fn usb_active_loop(
    pg_event: &Mailbox<bool>,
    serial: &MutexWithSerialPort,
) -> Result<(), Error> {
    debug_rprintln!("USB power on");
    loop {
        let event = select_biased! {
            pg = pg_event.read().fuse() => if pg? { UsbEvent::Nothing } else { UsbEvent::PoweredDown },
            _ = USB_EVENT.read().fuse() => UsbEvent::DataReady,
        };

        match event {
            UsbEvent::PoweredDown => break,
            UsbEvent::DataReady => read_serial(serial)?,
            UsbEvent::Nothing => {}
        }
    }

    Ok(())
}

fn read_serial(serial: &MutexWithSerialPort) -> Result<(), Error> {
    let mut buf = [0u8; 64];
    let result = critical_section::with(|cs| serial.borrow_ref_mut(cs).read(&mut buf));

    match result {
        Err(usbd_serial::UsbError::WouldBlock) => {}
        Ok(len) => debug_rprintln!("serial read: {:?}", &buf[..len]),
        Err(e) => debug_rprintln!("serial read error: {:?}", e),
    }

    Ok(())
}
