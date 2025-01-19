use rtt_target::debug_rprintln;

use crate::board::UsbSerialPort;
use crate::error::Error;

pub async fn task(serial: UsbSerialPort) -> Result<(), Error> {
    let mut buf = [0u8; 40];
    let mut total_bytes = 0;

    loop {
        let read_result = serial.read(&mut buf).await;
        match read_result {
            Ok(bytes_read) => {
                total_bytes += bytes_read;
                debug_rprintln!("SER DATA ({}): {:?}", total_bytes, &buf[..bytes_read]);
            }
            Err(e) => {
                debug_rprintln!("SER ERROR: {:?}", e);
            }
        };
    }
}
