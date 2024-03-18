#![deny(unsafe_code)]

#[derive(Debug)]
pub struct I2cError(stm32l0xx_hal::i2c::Error);

impl I2cError {
    pub fn new(error: stm32l0xx_hal::i2c::Error) -> Self {
        Self(error)
    }
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self.0 {
            stm32l0xx_hal::i2c::Error::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            stm32l0xx_hal::i2c::Error::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            stm32l0xx_hal::i2c::Error::PECError => embedded_hal::i2c::ErrorKind::Other,
            stm32l0xx_hal::i2c::Error::BusError => embedded_hal::i2c::ErrorKind::Bus,
            stm32l0xx_hal::i2c::Error::ArbitrationLost => {
                embedded_hal::i2c::ErrorKind::ArbitrationLoss
            }
        }
    }
}

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
