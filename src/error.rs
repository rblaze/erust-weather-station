#![deny(unsafe_code)]

use crate::hal_i2c::I2cError;

#[derive(Debug)]
pub enum Error {
    AlreadyTaken,
    CoreFmt,
    Environment(async_scheduler::executor::EnvError),
    I2c(I2cError),
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

impl From<I2cError> for Error {
    fn from(error: I2cError) -> Self {
        Error::I2c(error)
    }
}

impl From<void::Void> for Error {
    fn from(_error: void::Void) -> Self {
        unreachable!()
    }
}
