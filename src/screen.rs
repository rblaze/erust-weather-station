#![deny(unsafe_code)]

use bitflags::bitflags;
use embedded_hal::blocking::i2c::Write;

use crate::board::I2cBus;
use crate::error::Error;
use crate::hd44780::*;
use crate::st7036;

pub struct Screen {
    i2c: I2cBus,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Off,
    On,
}

impl Screen {
    const ADDR: u8 = 0x3C;

    pub fn new(i2c: I2cBus) -> Result<Self, Error> {
        let mut screen = Self { i2c };
        screen.init()?;
        Ok(screen)
    }

    fn init(&mut self) -> Result<(), Error> {
        self.send_command(st7036::INIT_SEQUENCE[0][0])?;
        cortex_m::asm::delay(1000);

        self.send_command(st7036::INIT_SEQUENCE[1][0])?;
        cortex_m::asm::delay(1000);

        let mut commands = [0; 8];
        commands[1..].copy_from_slice(st7036::INIT_SEQUENCE[2]);
        self.i2c.write(Self::ADDR, &commands)?;

        Ok(())
    }

    fn send_command(&mut self, command: u8) -> Result<(), Error> {
        self.i2c.write(Self::ADDR, &[0x00, command])?;
        Ok(())
    }

    pub fn cls(&mut self) -> Result<(), Error> {
        self.send_command(cls())
    }

    pub fn write(&mut self, s: &str) -> Result<(), Error> {
        const SCREEN_WIDTH: usize = 20;
        let mut string_buf = [0; SCREEN_WIDTH + 1];
        string_buf[0] = Control::DATA.bits();

        // Copy `s` to string buffer, replacing non-ASCII characters with '?'
        let len = s
            .chars()
            .take(SCREEN_WIDTH)
            .map(|c| if c.is_ascii() { c } else { '?' })
            .fold(1, |i, c| {
                string_buf[i] = c as u8;
                i + 1
            });

        self.i2c.write(Self::ADDR, &string_buf[..len])?;
        Ok(())
    }
}

impl core::fmt::Write for Screen {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s).map_err(|_| core::fmt::Error)
    }
}

bitflags! {
    struct Control: u8 {
        // The last control byte is tagged with a cleared most significant
        // bit (i.e. the continuation bit Co). After a control byte with a
        // cleared Co bit, only data bytes will follow.
        const CONT = 0b1000_0000;
        // The state of the RS bit defines whether the data byte is
        // interpreted as a command or as RAM data.
        // If the RS bit is set to logic 1, these display bytes are stored
        // in the display RAM at the address specified by the data pointer.
        // If the RS bit of the last control byte is set to logic 0, these
        // command bytes will be decoded and the setting of the device will
        // be changed according to the received commands.
        const DATA = 0b0100_0000;
    }
}
