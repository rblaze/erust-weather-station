#![deny(unsafe_code)]

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
pub fn bias_set(bias: Bias, fixed_bit: BiasFixedBit) -> u8 {
    CommandExt1::BiasSet as u8 | bias as u8 | 0b0000_0100 | fixed_bit as u8
}

/// Set ICON RAM address to AC
#[allow(unused)]
pub fn set_icon_ram_address(address: u8) -> u8 {
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
pub fn power_icon_contrast_set(
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
pub fn follower_control(follower_state: FollowerState, v0_generator_amplified_ratio: u8) -> u8 {
    CommandExt1::FollowerControl as u8
        | follower_state as u8
        | (v0_generator_amplified_ratio & 0b0000_0111)
}

/// Contrast set
pub fn contrast_set(contrast: u8) -> u8 {
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
pub fn double_height_position_set(position: DoubleHeightPosition) -> u8 {
    CommandExt2::DoubleHeightPositionSet as u8 | position as u8
}
