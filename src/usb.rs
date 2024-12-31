use rtt_target::debug_rprintln;

use crate::board::{MutexWithSerialPort, USB_EVENT};
use crate::error::Error;

pub async fn task(serial: &MutexWithSerialPort) -> Result<(), Error> {
    loop {
        USB_EVENT.read().await?;

        let mut buf = [0u8; 64];
        let result = critical_section::with(|cs| serial.borrow_ref_mut(cs).read(&mut buf));

        match result {
            Ok(len) => debug_rprintln!("serial read: {:?}", &buf[..len]),
            Err(e) => debug_rprintln!("serial read error: {:?}", e),
        }
    }
}
