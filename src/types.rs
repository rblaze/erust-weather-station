use core::cell::RefCell;

use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;

use crate::error::Error;

pub trait EventWaiter {
    async fn wait(&self);
}

pub trait OnOff {
    fn on(&mut self);
    fn off(&mut self);
}

pub trait Watchdog {
    fn feed(&self);
}

pub trait Joystick {
    #[allow(unused)]
    fn up(&mut self) -> bool;
    #[allow(unused)]
    fn down(&mut self) -> bool;
    fn left(&mut self) -> bool;
    fn right(&mut self) -> bool;
    fn select(&mut self) -> bool;
    fn button(&mut self) -> bool;
}

pub trait VoltageReader {
    fn millivolts(&mut self) -> u16;
}

pub trait UsbSerial {
    /// Synchronously writes to port as much as possible and does not block.
    fn write(&self, buf: &[u8]) -> Result<usize, Error>;
}

pub struct Backlight<R: SetDutyCycle, G: SetDutyCycle, B: SetDutyCycle> {
    pub red: R,
    pub green: G,
    pub blue: B,
}

pub struct Board<
    J: Joystick + EventWaiter,
    V: VoltageReader,
    D: OnOff,
    U: UsbSerial,
    P: OnOff,
    C: EventWaiter,
    W: Watchdog,
    I2cBus: I2c,
    R: SetDutyCycle,
    G: SetDutyCycle,
    B: SetDutyCycle,
> {
    pub joystick: J,
    pub vbat: V,
    pub backlight: Backlight<R, G, B>,
    pub display_power: D,
    pub usb_serial: U,
    pub usb_power: P,
    pub charger_event: C,
    pub i2c: RefCell<I2cBus>,
    pub watchdog: W,
}
