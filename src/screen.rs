#![deny(unsafe_code)]

use bitflags::bitflags;
use embedded_hal::blocking::i2c::Write;

use crate::board::I2cBus;
use crate::error::Error;

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

    #[allow(clippy::identity_op)] // zero bits are set for documentation purposes
    fn init(&mut self) -> Result<(), Error> {
        let command = Command::FunctionSet as u8
            | 0b10000 // 8-bit bus
            | 0b01000 // 2-line display
            | 0b00000 // 5x8 font
            | 0b00000 // instruction set 0
            ;
        debug_assert!(command == 0x38);
        self.i2c.write(Self::ADDR, &[0x00, command])?;
        cortex_m::asm::delay(1000);

        let command = Command::FunctionSet as u8
            | 0b10000 // 8-bit bus
            | 0b01000 // 2-line display
            | 0b00000 // 5x8 font
            | 0b00001 // instruction set 1
            ;
        debug_assert!(command == 0x39);
        self.i2c.write(Self::ADDR, &[0x00, command])?;
        cortex_m::asm::delay(1000);

        let commands = [
            0x00, // write commands
            CommandExt1::BiasSet as u8
            | 0b0000 // 1/5 bias
            | 0b0100 // const
            | 0b0000, // const for 2-line displays
            CommandExt1::ContrastSet as u8 | 0b1000, // low bits of contrast setting
            CommandExt1::PowerIconContrastSet as u8
            | 0b1000 // ICON display on
            | 0b0100 // booster circuit on
            | 0b0010, // high bits of contrast setting
            CommandExt1::FollowerControl as u8
            | 0b1000 // follower circuit on
            | 0b0101, // amplifier ratio
            Command::DisplayOnOff as u8
            | 0b0100 // display on
            | 0b0000 // cursor off
            | 0b0000, // blink off
            Command::ClearScreen as u8,
            Command::EntryModeSet as u8
            | 0b0010 // increment
            | 0b0000, // no display shift
        ];
        debug_assert!(commands == [0x00_u8, 0x14, 0x78, 0x5e, 0x6d, 0x0c, 0x01, 0x06]);
        self.i2c.write(Self::ADDR, &commands)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn set_state(
        &mut self,
        display: State,
        cursor: State,
        cursor_blink: State,
    ) -> Result<(), Error> {
        let display_bits = if display == State::On { 0b0100 } else { 0 };
        let cursor_bits = if cursor == State::On { 0b0010 } else { 0 };
        let blink_bits = if cursor_blink == State::On { 0b0001 } else { 0 };
        let command = Command::DisplayOnOff as u8 | display_bits | cursor_bits | blink_bits;

        self.i2c.write(Self::ADDR, &[0x00, command])?;
        Ok(())
    }

    pub fn cls(&mut self) -> Result<(), Error> {
        self.i2c
            .write(Self::ADDR, &[0x00, Command::ClearScreen as u8])?;
        Ok(())
    }

    pub fn write(&mut self, s: &str) -> Result<(), Error> {
        const SCREEN_SIZE: usize = 20;
        let mut string_buf = [0; SCREEN_SIZE + 1];
        string_buf[0] = Control::DATA.bits();

        let len = s
            .chars()
            .take(SCREEN_SIZE)
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

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
enum Command {
    ClearScreen = 0b0000_0001,
    ReturnHome = 0b0000_0010,
    EntryModeSet = 0b0000_0100,
    DisplayOnOff = 0b0000_1000,
    FunctionSet = 0b0010_0000,
    SetDdramAddress = 0b1000_0000,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
enum CommandExt0 {
    CursorDisplayShift = 0b0001_0000,
    SetCgramAddress = 0b0100_0000,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
enum CommandExt1 {
    BiasSet = 0b0001_0000,
    SetIconAddress = 0b0100_0000,
    PowerIconContrastSet = 0b0101_0000,
    FollowerControl = 0b0110_0000,
    ContrastSet = 0b0111_0000,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
enum CommandExt2 {
    DoubleHeightPositionSelect = 0b0001_0000,
}
