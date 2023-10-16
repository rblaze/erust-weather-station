#![deny(unsafe_code)]

#[derive(Debug)]
pub enum Error {
    AlreadyTaken,
    CoreFmt,
    Environment(async_scheduler::executor::EnvError),
    I2c(stm32l0xx_hal::i2c::Error),
    InvalidLcdLine,
}

impl From<core::fmt::Error> for Error {
    fn from(_error: core::fmt::Error) -> Self {
        Error::CoreFmt
    }
}

impl From<async_scheduler::executor::EnvError> for Error {
    fn from(error: async_scheduler::executor::EnvError) -> Self {
        Error::Environment(error)
    }
}

impl From<stm32l0xx_hal::i2c::Error> for Error {
    fn from(error: stm32l0xx_hal::i2c::Error) -> Self {
        Error::I2c(error)
    }
}

impl From<void::Void> for Error {
    fn from(_error: void::Void) -> Self {
        unreachable!()
    }
}
