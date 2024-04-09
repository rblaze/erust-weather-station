#![deny(unsafe_code)]

use core::convert::Infallible;

use crate::hal_i2c::I2cError;

#[derive(Debug)]
pub enum Error {
    AlreadyTaken,
    CoreFmt,
    Environment(async_scheduler::executor::EnvError),
    I2c(I2cError),
    InvalidLcdLine,
    Mailbox(async_scheduler::sync::mailbox::Error),
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

impl From<async_scheduler::mailbox::Error> for Error {
    fn from(error: async_scheduler::mailbox::Error) -> Self {
        Error::Mailbox(error)
    }
}

impl From<I2cError> for Error {
    fn from(error: I2cError) -> Self {
        Error::I2c(error)
    }
}

impl From<Infallible> for Error {
    fn from(_error: Infallible) -> Self {
        unreachable!()
    }
}
