#![deny(unsafe_code)]

use bitflags::bitflags;
use embedded_hal::blocking::i2c::Write;
use lcd::hd44780::*;
use lcd::screen::Screen;
use lcd::st7036::*;

use crate::board::I2cBus;
use crate::error::Error;

pub struct Lcd {
    i2c: I2cBus,
}

impl Lcd {
    const I2C_ADDR: u8 = 0x3C;

    pub fn new(i2c: I2cBus) -> Result<Self, Error> {
        let mut screen = Self { i2c };
        screen.init()?;
        Ok(screen)
    }

    fn init(&mut self) -> Result<(), Error> {
        self.send_command(lcd::st7036::function_set(
            BusWidth::EightBits,
            DisplayHeight::TwoLines,
            FontHeight::Normal,
            1,
        ))?;
        cortex_m::asm::delay(1000);

        self.send_commands(&[
            power_icon_contrast_set(IconState::Off, BoosterState::On, 32),
            follower_control(FollowerState::On, 5),
            display_on_off(DisplayState::On, CursorState::Off, BlinkState::Off),
        ])?;
        Ok(())
    }
}

impl Screen<20, 2, crate::error::Error> for Lcd {
    fn send_command(&mut self, command: u8) -> Result<(), Error> {
        self.i2c.write(Self::I2C_ADDR, &[0x00, command])?;
        Ok(())
    }

    fn send_data(&mut self, data: u8) -> Result<(), crate::error::Error> {
        self.i2c
            .write(Self::I2C_ADDR, &[Control::DATA.bits(), data])?;
        Ok(())
    }
}

impl core::fmt::Write for Lcd {
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
