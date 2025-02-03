#![deny(unsafe_code)]

use core::convert::Infallible;

use stm32g0_hal::i2c::Error as I2cError;

#[allow(unused)] // TODO: check why some errors are considered unused
#[derive(Debug)]
pub enum Error {
    AlreadyTaken,
    CoreFmt,
    #[allow(unused)]
    Environment(async_scheduler::executor::EnvError),
    #[allow(unused)]
    I2c(I2cError),
    InvalidLcdLine,
    Mailbox(async_scheduler::sync::mailbox::Error),
    Uninitialized,
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
