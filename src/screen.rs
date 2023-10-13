#![deny(unsafe_code)]

use bitflags::bitflags;
use embedded_hal::blocking::i2c::Write;

use crate::board::I2cBus;
use crate::error::Error;
use crate::hd44780::{self, *};
use crate::st7036::*;

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
        let command = Self::function_set(
            BusWidth::EightBits,
            DisplayHeight::TwoLines,
            FontHeight::Normal,
            0,
        );
        debug_assert_eq!(command, 0x38);
        self.send_command(command)?;
        cortex_m::asm::delay(1000);

        let command = Self::function_set(
            BusWidth::EightBits,
            DisplayHeight::TwoLines,
            FontHeight::Normal,
            1,
        );
        debug_assert_eq!(command, 0x39);
        self.send_command(command)?;
        cortex_m::asm::delay(1000);

        let contrast = 40; // Taked from initialization example, 0b00101000
        let v0_amplified_ratio = 5;

        let commands = [
            0x00, // write commands
            bias_set(Bias::Set1_5, BiasFixedBit::OtherDisplay),
            contrast_set(contrast),
            power_icon_contrast_set(IconState::On, BoosterState::On, contrast),
            follower_control(FollowerState::On, v0_amplified_ratio),
            display_on_off(DisplayState::On, CursorState::Off, BlinkState::Off),
            cls(),
            entry_mode_set(TextDirection::LeftToRight, ShiftMode::CursorShift),
        ];
        debug_assert_eq!(
            commands,
            [0x00_u8, 0x14, 0x78, 0x5e, 0x6d, 0x0c, 0x01, 0x06]
        );
        self.i2c.write(Self::ADDR, &commands)?;

        Ok(())
    }

    fn send_command(&mut self, command: u8) -> Result<(), Error> {
        self.i2c.write(Self::ADDR, &[0x00, command])?;
        Ok(())
    }

    /// This display has extended functions.
    fn function_set(
        data_width: BusWidth,
        display_height: DisplayHeight,
        font_height: FontHeight,
        instruction_set: u8,
    ) -> u8 {
        hd44780::function_set(data_width, display_height, font_height) | (instruction_set & 0b0011)
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
