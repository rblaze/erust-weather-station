#![deny(unsafe_code)]

use crate::hd44780;

/// Extended command set for ST7036 LCD controller (instruction set 1)
#[derive(Debug, Clone, Copy)]
enum CommandExt1 {
    BiasSet = 0b0001_0000,
    SetIconRamAddress = 0b0100_0000,
    PowerIconContrastSet = 0b0101_0000,
    FollowerControl = 0b0110_0000,
    ContrastSet = 0b0111_0000,
}

/// Extended command set for ST7036 LCD controller (instruction set 2)
#[derive(Debug, Clone, Copy)]
enum CommandExt2 {
    DoubleHeightPositionSet = 0b0001_0000,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum Bias {
    /// Bias 1/4
    Set1_4 = 0b0000_1000,
    /// Bias 1/5
    Set1_5 = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BiasFixedBit {
    ThreeLineDisplay = 0b0000_0001,
    OtherDisplay = 0,
}

/// Bias selection
pub const fn bias_set(bias: Bias, fixed_bit: BiasFixedBit) -> u8 {
    CommandExt1::BiasSet as u8 | bias as u8 | 0b0000_0100 | fixed_bit as u8
}

/// Set ICON RAM address to AC
#[allow(unused)]
pub const fn set_icon_ram_address(address: u8) -> u8 {
    CommandExt1::SetIconRamAddress as u8 | (address & 0b0000_1111)
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum IconState {
    On = 0b0000_1000,
    Off = 0,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BoosterState {
    On = 0b0000_0100,
    Off = 0,
}

/// Power/ICON control/Contrast set
pub const fn power_icon_contrast_set(
    icon_state: IconState,
    booster_state: BoosterState,
    contrast: u8,
) -> u8 {
    let contrast_bits = (contrast & 0b0011_0000) >> 4;
    CommandExt1::PowerIconContrastSet as u8 | icon_state as u8 | booster_state as u8 | contrast_bits
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum FollowerState {
    On = 0b0000_1000,
    Off = 0,
}

/// Follower control
pub const fn follower_control(
    follower_state: FollowerState,
    v0_generator_amplified_ratio: u8,
) -> u8 {
    CommandExt1::FollowerControl as u8
        | follower_state as u8
        | (v0_generator_amplified_ratio & 0b0000_0111)
}

/// Contrast set
pub const fn contrast_set(contrast: u8) -> u8 {
    CommandExt1::ContrastSet as u8 | (contrast & 0b0000_1111)
}

/// On 3-line display selects which two lines are used for double-height font
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum DoubleHeightPosition {
    TopTwoLines = 0b0000_1000,
    LowTwoLines = 0,
}

#[allow(unused)]
pub const fn double_height_position_set(position: DoubleHeightPosition) -> u8 {
    CommandExt2::DoubleHeightPositionSet as u8 | position as u8
}

/// This display supports two extra instruction sets.
pub const fn function_set(
    data_width: hd44780::BusWidth,
    display_height: hd44780::DisplayHeight,
    font_height: hd44780::FontHeight,
    instruction_set: u8,
) -> u8 {
    hd44780::function_set(data_width, display_height, font_height) | (instruction_set & 0b0011)
}

const DEFAULT_CONTRAST: u8 = 40;
const DEFAULT_V0_AMPLIFIED_RATIO: u8 = 5;

/// Initialization sequence from the driver datasheet.
/// Insert 2ms delay after each block.
pub const INIT_SEQUENCE: [&[u8]; 3] = [
    &[function_set(
        hd44780::BusWidth::EightBits,
        hd44780::DisplayHeight::TwoLines,
        hd44780::FontHeight::Normal,
        0,
    )],
    &[function_set(
        hd44780::BusWidth::EightBits,
        hd44780::DisplayHeight::TwoLines,
        hd44780::FontHeight::Normal,
        1,
    )],
    &[
        bias_set(Bias::Set1_5, BiasFixedBit::OtherDisplay),
        contrast_set(DEFAULT_CONTRAST),
        power_icon_contrast_set(IconState::On, BoosterState::On, DEFAULT_CONTRAST),
        follower_control(FollowerState::On, DEFAULT_V0_AMPLIFIED_RATIO),
        hd44780::display_on_off(
            hd44780::DisplayState::On,
            hd44780::CursorState::Off,
            hd44780::BlinkState::Off,
        ),
        hd44780::cls(),
        hd44780::entry_mode_set(
            hd44780::TextDirection::LeftToRight,
            hd44780::ShiftMode::CursorShift,
        ),
    ],
];

// TODO: add tests
// commands [0x38], [0x39], [0x14, 0x78, 0x5e, 0x6d, 0x0c, 0x01, 0x06]
