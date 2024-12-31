use async_scheduler::executor::yield_now;
use rtt_target::debug_rprintln;

use crate::board::{Serial, Usb};
use crate::error::Error;

pub async fn task(mut usb: Usb, mut serial: Serial) -> Result<(), Error> {
    let mut state = usb.state();

    debug_rprintln!("USB state {:?}", state);

    loop {
        if usb.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            let out = serial.read(&mut buf);
            match out {
                Ok(num_bytes) => if num_bytes > 0 {
                    debug_rprintln!("Received {} bytes from serial", num_bytes);
                },
                Err(e) => if e != usbd_serial::UsbError::WouldBlock {
                    debug_rprintln!("Error reading from serial: {:?}", e);
                }
            }
        }

        let new_state = usb.state();
        if new_state != state {
            debug_rprintln!("<==========> USB state {:?}", new_state);
            state = new_state;
        }
    
        yield_now().await;
    }
}
