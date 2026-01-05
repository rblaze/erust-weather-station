use core::convert::Infallible;

use thiserror::Error;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Error)]
pub enum Error {
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
    I2c(#[from] stm32g0_hal::i2c::Error),
    #[error(transparent)]
    Sensirion(#[from] sensirion::Error<stm32g0_hal::i2c::Error>),
}

impl From<Infallible> for Error {
    fn from(_error: Infallible) -> Self {
        unreachable!()
    }
}
