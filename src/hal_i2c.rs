#![deny(unsafe_code)]

use embedded_hal::i2c::{self};
use stm32l0xx_hal::i2c as stmhal_i2c;
use stm32l0xx_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};

type HalError = stm32l0xx_hal::i2c::Error;

#[derive(Debug)]
pub struct I2cError(HalError);

impl i2c::Error for I2cError {
    fn kind(&self) -> i2c::ErrorKind {
        match self.0 {
            HalError::Overrun => i2c::ErrorKind::Overrun,
            HalError::Nack => i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Unknown),
            HalError::PECError => i2c::ErrorKind::Other,
            HalError::BusError => i2c::ErrorKind::Bus,
            HalError::ArbitrationLost => i2c::ErrorKind::ArbitrationLoss,
        }
    }
}

pub struct I2cBus<BUS>(BUS);

impl<BUS> I2cBus<BUS> {
    pub fn new(bus: BUS) -> Self {
        Self(bus)
    }
}

impl<BUS> i2c::ErrorType for I2cBus<BUS> {
    type Error = I2cError;
}

impl<BUS> i2c::I2c<i2c::SevenBitAddress> for I2cBus<BUS>
where
    BUS: _embedded_hal_blocking_i2c_Read<Error = stmhal_i2c::Error>
        + _embedded_hal_blocking_i2c_Write<Error = stmhal_i2c::Error>
        + _embedded_hal_blocking_i2c_WriteRead<Error = stmhal_i2c::Error>,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let result = match operations {
            [i2c::Operation::Write(buffer)] => self.0.write(address, buffer),
            [i2c::Operation::Read(buffer)] => self.0.read(address, buffer),
            [i2c::Operation::Write(write_buffer), i2c::Operation::Read(read_buffer)] => {
                self.0.write_read(address, write_buffer, read_buffer)
            }
            _ => panic!("Invalid transaction"),
        };

        result.map_err(I2cError)
    }
}
