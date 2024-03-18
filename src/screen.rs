#![deny(unsafe_code)]

use crate::error::{Error, I2cError};
use embedded_hal::i2c::I2c;
use lcd::hd44780::*;
use lcd::screen::Screen;
use lcd::st7036::*;

pub struct Lcd<I2C> {
    i2c: I2C,
}

impl<I2C: I2c<Error = I2cError>> Lcd<I2C> {
    const I2C_ADDR: u8 = 0x3C;

    pub fn new(i2c: I2C) -> Result<Self, Error> {
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

impl<I2C: I2c<Error = I2cError>> Screen<20, 2, crate::error::Error> for Lcd<I2C> {
    fn send_command(&mut self, command: u8) -> Result<(), Error> {
        self.i2c.write(Self::I2C_ADDR, &[0x00, command])?;
        Ok(())
    }

    fn send_data(&mut self, data: u8) -> Result<(), crate::error::Error> {
        self.i2c.write(Self::I2C_ADDR, &[0b0100_0000, data])?;
        Ok(())
    }
}

impl<I2C: I2c<Error = I2cError>> core::fmt::Write for Lcd<I2C> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s).map_err(|_| core::fmt::Error)
    }
}
