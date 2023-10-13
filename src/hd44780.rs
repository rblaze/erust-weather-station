#![deny(unsafe_code)]

/// Generic LCD commands, common for most controllers.
#[derive(Debug, Clone, Copy)]
enum Command {
    ClearScreen = 0b0000_0001,
    ReturnHome = 0b0000_0010,
    EntryModeSet = 0b0000_0100,
    DisplayOnOff = 0b0000_1000,
    CursorDisplayShift = 0b0001_0000,
    FunctionSet = 0b0010_0000,
    SetCgramAddress = 0b0100_0000,
    SetDdramAddress = 0b1000_0000,
}

/// Clears all display and returns the cursor to the home position (Address 0)
pub fn cls() -> u8 {
    Command::ClearScreen as u8
}

/// Returns the cursor to the home position (Address 0).
/// Returns display to its original state if it was shifted.
#[allow(unused)]
pub fn return_home() -> u8 {
    Command::ReturnHome as u8
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum TextDirection {
    LeftToRight = 0b0000_0010,
    RightToLeft = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum ShiftMode {
    /// The display will be shifted to the left (if I/D = 1) or right
    /// (if I/D = 0) on subsequent DD RAM write operations. This makes it
    /// looks as if the cursor stands still and the display moves when each
    /// character is written to the DD RAM.
    DisplayShift = 0b0000_0001,
    /// The display will not shift on subsequent DD RAM write operations.
    CursorShift = 0,
}

/// Sets the effect of subsequent DD RAM read or write operations.
/// Sets the cursor move direction and specifies or not to shift the display.
/// These operations are performed during data read and write.
pub fn entry_mode_set(text_direction: TextDirection, shift_mode: ShiftMode) -> u8 {
    Command::EntryModeSet as u8 | text_direction as u8 | shift_mode as u8
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum DisplayState {
    On = 0b0000_0100,
    Off = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum CursorState {
    On = 0b0000_0010,
    Off = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BlinkState {
    On = 0b0000_0001,
    Off = 0,
}

pub fn display_on_off(
    display_state: DisplayState,
    cursor_state: CursorState,
    blink_state: BlinkState,
) -> u8 {
    Command::DisplayOnOff as u8 | display_state as u8 | cursor_state as u8 | blink_state as u8
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum ShiftDirection {
    ShiftLeft = 0,
    ShiftRight = 0b0000_0001,
}

/// Moves the cursor and shifts the display without changing DD RAM contents
#[allow(unused)]
pub fn cursor_display_shift(shift_mode: ShiftMode, shift_direction: ShiftDirection) -> u8 {
    Command::CursorDisplayShift as u8 | ((shift_mode as u8) << 1) | shift_direction as u8
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BusWidth {
    EightBits = 0b0001_0000,
    FourBits = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum DisplayHeight {
    OneLine = 0,
    TwoLines = 0b0000_1000,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum FontHeight {
    Normal = 0,
    High = 0b0000_0100,
}

pub fn function_set(
    data_width: BusWidth,
    display_height: DisplayHeight,
    font_height: FontHeight,
) -> u8 {
    Command::FunctionSet as u8 | (data_width as u8) | (display_height as u8) | (font_height as u8)
}

#[allow(unused)]
pub fn set_gcram_address(address: u8) -> u8 {
    Command::SetCgramAddress as u8 | (address & 0b0011_1111)
}

#[allow(unused)]
pub fn set_ddram_address(address: u8) -> u8 {
    Command::SetDdramAddress as u8 | (address & 0b0111_1111)
}
