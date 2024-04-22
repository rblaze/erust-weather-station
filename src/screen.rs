#![deny(unsafe_code)]

use crate::error::Error;
use embedded_hal::i2c::{self};
use lcd::hd44780::*;
use lcd::screen::Screen;
use lcd::st7036::*;

const WIDTH: usize = 20;
const HEIGHT: usize = 2;

enum TransactionMode {
    Command = 0b0000_0000,
    Data = 0b0100_0000,
}

pub struct Lcd<'a, I2C> {
    i2c: &'a mut I2C,
}

impl<'a, I2C: i2c::I2c> Lcd<'a, I2C>
where
    Error: From<I2C::Error>,
{
    const I2C_ADDR: u8 = 0x3C;

    pub fn new(i2c: &'a mut I2C) -> Result<Self, Error> {
        let mut screen = Self { i2c };
        screen.init()?;
        Ok(screen)
    }

    fn init(&mut self) -> Result<(), Error> {
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

impl<I2C: i2c::I2c> Screen<WIDTH, HEIGHT, crate::error::Error> for Lcd<'_, I2C>
where
    Error: From<I2C::Error>,
{
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
        if data.len() > 1 && data.len() <= WIDTH {
            // Fast path. Copying a few bytes in the memory is faster
            // than repeating I2C transactions.
            // TODO: consider using uninit after maybe_uninit_write_slice stabilized.
            let mut buf = [0; WIDTH + 1];
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

impl<I2C: i2c::I2c> core::fmt::Write for Lcd<'_, I2C>
where
    Error: From<I2C::Error>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s).map_err(|_| core::fmt::Error)
    }
}
