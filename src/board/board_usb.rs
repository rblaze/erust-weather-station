use core::cell::{Cell, RefCell, UnsafeCell};
use core::mem::MaybeUninit;

use critical_section::Mutex;
use embedded_hal::digital::OutputPin;
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::gpioa::PA10;
use stm32g0_hal::gpio::{Output, PushPull};
use stm32g0_hal::pac::interrupt;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{
    StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid,
};
use usb_device::LangID;

use crate::error::Error;

pub type UsbBus = stm32g0_hal::usb::Bus<stm32g0_hal::pac::USB>;
pub type CdcAcm = usbd_serial::CdcAcmClass<'static, UsbBus>;
pub type Usb = UsbDevice<'static, UsbBus>;

const CDC_MAX_PACKET_SIZE: usize = 64;

struct BufferedCdcAcm {
    cdcacm: CdcAcm,
    read_buffer: [u8; CDC_MAX_PACKET_SIZE],
    read_offset: usize,
    read_bytes_in_buffer: usize,
    write_buffer: [u8; CDC_MAX_PACKET_SIZE],
    write_bytes_in_buffer: usize,
}

impl BufferedCdcAcm {
    fn new(cdcacm: CdcAcm) -> Self {
        Self {
            cdcacm,
            read_buffer: [0; CDC_MAX_PACKET_SIZE],
            read_offset: 0,
            read_bytes_in_buffer: 0,
            write_buffer: [0; CDC_MAX_PACKET_SIZE],
            write_bytes_in_buffer: 0,
        }
    }

    /// Reads data to internal buffer and pauses the endpoint until buffer is consumed.
    fn poll_read(&mut self, bus: &UsbBus) -> Result<(), usbd_serial::UsbError> {
        // Pause endpoint so host won't send the next packet until we have buffer space.
        self.pause(bus);

        let result = self
            .cdcacm
            .read_packet(&mut self.read_buffer)
            .and_then(|bytes_read| {
                if self.read_bytes_in_buffer == 0 || bytes_read == 0 {
                    // No buffer overrun.
                    if bytes_read > 0 {
                        self.read_offset = 0;
                        self.read_bytes_in_buffer = bytes_read;
                    }

                    if self.read_bytes_in_buffer == 0 {
                        Err(usbd_serial::UsbError::WouldBlock)
                    } else {
                        Ok(())
                    }
                } else {
                    // Shouldn't happen, but handle it anyway.
                    self.read_offset = 0;
                    self.read_bytes_in_buffer = bytes_read;

                    Err(usbd_serial::UsbError::BufferOverflow)
                }
            });

        self.unpause_if_empty(bus);
        debug_rprintln!("ser rd poll {} {:?}", self.read_bytes_in_buffer, result);

        result
    }

    /// Writes data from internal buffer to endpoint.
    fn poll_write(&mut self) -> Result<(), usbd_serial::UsbError> {
        if self.write_bytes_in_buffer == 0 {
            // Do not return Ok: it means internal buffer was freed.
            return Err(usbd_serial::UsbError::WouldBlock);
        }

        let bytes_to_write = self.write_bytes_in_buffer;
        let bytes_written = self
            .cdcacm
            .write_packet(&self.write_buffer[..bytes_to_write])?;

        debug_rprintln!(
            "ser wr poll {} {}",
            self.write_bytes_in_buffer,
            bytes_written
        );

        // Clear the buffer regardless of overruns.
        self.write_bytes_in_buffer = 0;
        if bytes_written != bytes_to_write {
            return Err(usbd_serial::UsbError::BufferOverflow);
        }

        Ok(())
    }

    /// Reads data from internal buffer.
    fn read(&mut self, bus: &UsbBus, buf: &mut [u8]) -> usize {
        let bytes_to_copy = buf.len().min(self.read_bytes_in_buffer);

        if bytes_to_copy != 0 {
            buf[..bytes_to_copy].copy_from_slice(
                &self.read_buffer[self.read_offset..self.read_offset + bytes_to_copy],
            );
            self.read_offset += bytes_to_copy;
            self.read_bytes_in_buffer -= bytes_to_copy;
        }

        self.unpause_if_empty(bus);

        debug_rprintln!("ser read {} {}", self.read_bytes_in_buffer, bytes_to_copy);

        bytes_to_copy
    }

    /// Writes data to the internal buffer.
    fn write(&mut self, buf: &[u8]) -> usize {
        if self.write_bytes_in_buffer > 0 {
            // Write operation in progress, just append to buffer as much as we can.
            let bytes_to_copy = buf
                .len()
                .min(self.write_buffer.len() - self.write_bytes_in_buffer);

            if bytes_to_copy != 0 {
                self.write_buffer
                    [self.write_bytes_in_buffer..self.write_bytes_in_buffer + bytes_to_copy]
                    .copy_from_slice(&buf[..bytes_to_copy]);
                self.write_bytes_in_buffer += bytes_to_copy;
            }

            debug_rprintln!(
                "ser write {} + {} of {}",
                self.write_bytes_in_buffer,
                bytes_to_copy,
                buf.len()
            );

            bytes_to_copy
        } else {
            // If the buffer is empty, our first write and poll_write() can clear it again,
            // so we need to repeat the send.
            let mut bytes = buf;
            while !bytes.is_empty() && self.write_bytes_in_buffer == 0 {
                let bytes_to_copy = bytes.len().min(self.write_buffer.len());

                self.write_buffer
                    [self.write_bytes_in_buffer..self.write_bytes_in_buffer + bytes_to_copy]
                    .copy_from_slice(&buf[..bytes_to_copy]);
                self.write_bytes_in_buffer += bytes_to_copy;
                bytes = &bytes[bytes_to_copy..];

                // Poke the USB device to send the data.
                let _ = self.poll_write();
            }

            let bytes_sent = buf.len() - bytes.len();
            debug_rprintln!(
                "ser write {} ({}) of {}",
                bytes_sent,
                self.write_bytes_in_buffer,
                buf.len()
            );

            bytes_sent
        }
    }

    /// Sets NACK on the rx endpoint
    fn pause(&self, bus: &UsbBus) {
        bus.set_paused(self.cdcacm.read_ep(), true);
    }

    /// Removes NACK from the rx endpoint if buffer is empty.
    fn unpause_if_empty(&self, bus: &UsbBus) {
        if self.read_bytes_in_buffer == 0 {
            // Buffer is empty, allow new data to come in.
            bus.set_paused(self.cdcacm.read_ep(), false);
        }
    }
}

type MutexWithSerialPort = Mutex<RefCell<Option<BufferedCdcAcm>>>;
type MutexWithUsbDevice = Mutex<RefCell<Option<Usb>>>;
type MutexWithPowerState = Mutex<Cell<PowerState>>;

pub struct UsbSerialPort {
    port: &'static MutexWithSerialPort,
    bus: &'static MutexWithUsbDevice,
    usb_event: &'static async_scheduler::sync::mailbox::Mailbox<()>,
}

impl UsbSerialPort {
    fn new(
        port: &'static MutexWithSerialPort,
        bus: &'static MutexWithUsbDevice,
        usb_event: &'static async_scheduler::sync::mailbox::Mailbox<()>,
    ) -> Self {
        Self {
            port,
            bus,
            usb_event,
        }
    }

    pub async fn read(&self, buf: &mut [u8]) -> Result<usize, Error> {
        loop {
            let read_result = critical_section::with(|cs| {
                let mut usb_device_ref = self.bus.borrow_ref_mut(cs);
                let mut serial_ref = self.port.borrow_ref_mut(cs);

                usb_device_ref
                    .as_mut()
                    .zip(serial_ref.as_mut())
                    .map(|(device, serial)| serial.read(device.bus(), buf))
            });

            if let Some(bytes_read) = read_result {
                if bytes_read > 0 {
                    return Ok(bytes_read);
                }
            }

            self.usb_event.read().await?;
        }
    }

    pub async fn write(&self, buf: &[u8]) -> Result<usize, Error> {
        let mut bytes = buf;
        while !bytes.is_empty() {
            let bytes_written = critical_section::with(|cs| {
                self.port
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .map(|serial| serial.write(bytes))
            })
            .ok_or(Error::Uninitialized)?;

            bytes = &bytes[bytes_written..];

            if !bytes.is_empty() {
                self.usb_event.read().await?;
            }
        }

        Ok(buf.len())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum PowerState {
    ExternalPower,
    Battery,
}

/// Wrapper for statics initialized in Board::new and used later.
struct SafelyInitializedStatic<T>(UnsafeCell<MaybeUninit<T>>);
unsafe impl<T> Sync for SafelyInitializedStatic<T> {}

impl<T> SafelyInitializedStatic<T> {
    /// Constructs a new instance of the wrapper.
    const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    /// Initialize the value.
    #[allow(clippy::mut_from_ref)]
    fn write(&self, value: T) -> &mut T {
        unsafe { &mut *self.0.get() }.write(value)
    }
}

static USB_ALLOCATOR: SafelyInitializedStatic<UsbBusAllocator<UsbBus>> =
    SafelyInitializedStatic::new();
static USB_SERIAL: MutexWithSerialPort = Mutex::new(RefCell::new(None));
static USB_DEVICE: MutexWithUsbDevice = Mutex::new(RefCell::new(None));
static USB_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();
static USB_POWER: MutexWithPowerState = Mutex::new(Cell::new(PowerState::Battery));

#[interrupt]
fn UCPD1_UCPD2_USB() {
    let mut led =
        unsafe { core::mem::MaybeUninit::<PA10<Output<PushPull>>>::uninit().assume_init() };
    led.set_high().unwrap();

    debug_rprintln!("USB interrupt");
    critical_section::with(|cs| {
        let mut usb_device_ref = USB_DEVICE.borrow_ref_mut(cs);
        let mut serial_ref = USB_SERIAL.borrow_ref_mut(cs);

        let (usb_device, serial) = usb_device_ref
            .as_mut()
            .zip(serial_ref.as_mut())
            .expect("USB interrupt with uninitialized USB device");

        if usb_device.poll(&mut [&mut serial.cdcacm]) {
            match serial.poll_read(usb_device.bus()) {
                Ok(_) => {
                    // Read some data, wakeup coroutine.
                    USB_EVENT.post(());
                }
                Err(usbd_serial::UsbError::WouldBlock) => { /* false trigger, ignore */ }
                Err(err) => {
                    // TODO: pass error to reader?
                    debug_rprintln!("USB serial read error: {:?}", err);
                }
            }
            match serial.poll_write() {
                Ok(_) => {
                    // Wrote some data, buffer is free, wakeup coroutine.
                    USB_EVENT.post(());
                }
                Err(usbd_serial::UsbError::WouldBlock) => { /* false trigger, ignore */ }
                Err(err) => {
                    // TODO: pass error to writer?
                    debug_rprintln!("USB serial write error: {:?}", err);
                }
            }
        }

        if usb_device.state() == UsbDeviceState::Suspend {
            // If USB is suspended and we are on battery power, shutdown USB
            if USB_POWER.borrow(cs).get() == PowerState::Battery {
                drop(usb_device_ref);
                drop(serial_ref);

                shutdown_usb(cs);
            }
        }
    });

    led.set_low().unwrap();
}

fn start_usb(cs: critical_section::CriticalSection<'_>) {
    debug_rprintln!("starting USB");
    // TODO: turn on clock and peripheral

    let borrow = USB_DEVICE.borrow_ref(cs);
    let bus = borrow.as_ref().unwrap().bus();
    bus.power_up();
    bus.enable_interrupts();
}

fn shutdown_usb(cs: critical_section::CriticalSection<'_>) {
    debug_rprintln!("shutting down USB");

    let borrow = USB_DEVICE.borrow_ref(cs);
    let bus = borrow.as_ref().unwrap().bus();
    bus.disable_interrupts();
    bus.power_down();

    // TODO: turn off USB clock and peripheral
}

pub(super) fn on_external_power() {
    critical_section::with(|cs| {
        USB_POWER.borrow(cs).set(PowerState::ExternalPower);
        start_usb(cs);
    });
}

pub(super) fn on_battery() {
    critical_section::with(|cs| {
        USB_POWER.borrow(cs).set(PowerState::Battery);
        // If bus is already in suspend, turn off USB
        let borrow = USB_DEVICE.borrow_ref(cs);
        if borrow.as_ref().unwrap().state() == UsbDeviceState::Suspend {
            drop(borrow);
            shutdown_usb(cs);
        }
    });
}

pub(super) fn serial_port(usb: UsbBus) -> UsbSerialPort {
    let usb_bus = USB_ALLOCATOR.write(UsbBusAllocator::new(usb));
    critical_section::with(|cs| {
        USB_SERIAL.replace(
            cs,
            Some(BufferedCdcAcm::new(usbd_serial::CdcAcmClass::new(
                usb_bus,
                CDC_MAX_PACKET_SIZE as u16,
            ))),
        );
        USB_DEVICE.replace(
            cs,
            Some(
                UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x5824, 0x5432))
                    .strings(&[StringDescriptors::new(LangID::EN)
                        .product("Weather Station")
                        .manufacturer("Blaze")
                        .serial_number("unique")])
                    .expect("Failed to set strings")
                    .composite_with_iads()
                    .device_release(0x0001)
                    .self_powered(true)
                    .build(),
            ),
        );
    });

    UsbSerialPort::new(&USB_SERIAL, &USB_DEVICE, &USB_EVENT)
}
