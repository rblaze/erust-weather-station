#![deny(unsafe_code)]

use crate::error;

use stm32l0xx_hal::i2c::{self};
use stm32l0xx_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};

pub struct I2cBus<BUS>(BUS);

impl<BUS> I2cBus<BUS> {
    pub fn new(bus: BUS) -> Self {
        Self(bus)
    }
}

impl<BUS> embedded_hal::i2c::ErrorType for I2cBus<BUS> {
    type Error = error::I2cError;
}

impl<BUS> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cBus<BUS>
where
    BUS: _embedded_hal_blocking_i2c_Read<Error = i2c::Error>
        + _embedded_hal_blocking_i2c_Write<Error = i2c::Error>
        + _embedded_hal_blocking_i2c_WriteRead<Error = i2c::Error>,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let result = match operations {
            [embedded_hal::i2c::Operation::Write(buffer)] => self.0.write(address, buffer),
            [embedded_hal::i2c::Operation::Read(buffer)] => self.0.read(address, buffer),
            [embedded_hal::i2c::Operation::Write(write_buffer), embedded_hal::i2c::Operation::Read(read_buffer)] => {
                self.0.write_read(address, write_buffer, read_buffer)
            }
            _ => panic!("Invalid transaction"),
        };

        result.map_err(error::I2cError::new)
    }
}
