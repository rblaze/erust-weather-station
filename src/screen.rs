use embedded_hal::i2c::I2c;
use lcd::hd44780::*;
use lcd::screen::Screen;
use lcd::st7036::*;

use crate::board::SharedI2cBus;
use crate::error::Error;

enum TransactionMode {
    Command = 0b0000_0000,
    Data = 0b0100_0000,
}

pub struct Lcd<'a> {
    i2c: SharedI2cBus<'a>,
}

impl<'a> Lcd<'a> {
    const I2C_ADDR: u8 = 0x3C;
    const WIDTH: usize = 20;
    const HEIGHT: usize = 2;

    pub fn new(i2c: SharedI2cBus<'a>) -> Self {
        Self { i2c }
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.send_commands(INIT_SEQUENCE[0])?;
        cortex_m::asm::delay(1000);
        self.send_commands(INIT_SEQUENCE[1])?;
        cortex_m::asm::delay(1000);
        self.send_commands(INIT_SEQUENCE[2])?;
        Ok(())
    }

    pub fn set_output_line(&mut self, line: usize) -> Result<(), Error> {
        if line > 1 {
            return Err(Error::InvalidLcdLine);
        }
        self.send_command(set_ddram_address(line as u8 * 0x40))?;
        Ok(())
    }
}

impl Screen<{ Lcd::WIDTH }, { Lcd::HEIGHT }, crate::error::Error> for Lcd<'_> {
    fn send_command(&mut self, command: u8) -> Result<(), Error> {
        self.i2c
            .write(Self::I2C_ADDR, &[TransactionMode::Command as u8, command])?;
        Ok(())
    }

    fn send_data(&mut self, data: u8) -> Result<(), crate::error::Error> {
        self.i2c
            .write(Self::I2C_ADDR, &[TransactionMode::Data as u8, data])?;
        Ok(())
    }

    fn send_data_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() > 1 && data.len() <= Self::WIDTH {
            // Fast path. Copying a few bytes in the memory is faster
            // than repeating I2C transactions.
            // TODO: consider using uninit after maybe_uninit_write_slice stabilized.
            let mut buf = [0; Self::WIDTH + 1];
            buf[0] = TransactionMode::Data as u8;
            buf[1..data.len() + 1].copy_from_slice(data);
            self.i2c.write(Self::I2C_ADDR, &buf[..data.len() + 1])?;
        } else {
            // Slow path.
            for byte in data {
                self.send_data(*byte)?;
            }
        }
        Ok(())
    }
}

impl core::fmt::Write for Lcd<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s).map_err(|_| core::fmt::Error)
    }
}
