#![deny(unsafe_code)]

pub mod adc;
pub mod exti;
pub mod gpio;
pub mod i2c;
pub mod rcc;
pub mod timer;

pub use stm32g0::stm32g071 as pac;
