use core::convert::Infallible;

use thiserror::Error;

// Workaround for compiler concerns about possible
// `impl embedded_hal::i2c::Error for async_scheduler::mailbox::Error`
#[derive(Debug, PartialEq, Eq, Clone, Copy, Error)]
pub struct I2cError<Err: embedded_hal::i2c::Error>(#[from] pub Err);

#[derive(Debug, PartialEq, Eq, Clone, Copy, Error)]
pub enum Error<I2cErr: embedded_hal::i2c::Error> {
    #[error("peripherals already initialized")]
    AlreadyTaken,
    #[error("LCD coordinates out of range")]
    InvalidLcdLine,
    #[error("board not initialized")]
    Uninitialized,
    #[error("device busy")]
    Busy,
    #[error(transparent)]
    Mailbox(#[from] async_scheduler::mailbox::Error),
    #[error(transparent)]
    CoreFmt(#[from] core::fmt::Error),
    #[error(transparent)]
    I2c(#[from] I2cError<I2cErr>),
    #[error(transparent)]
    Sensirion(#[from] sensirion::Error<I2cErr>),
}

impl<I2cError: embedded_hal::i2c::Error> From<Infallible> for Error<I2cError> {
    fn from(_error: Infallible) -> Self {
        unreachable!()
    }
}
