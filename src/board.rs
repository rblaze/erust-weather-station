use core::cell::{RefCell, UnsafeCell};
use core::mem::MaybeUninit;

use critical_section::Mutex;
use rtt_target::debug_rprintln;
use stm32g0_hal::adc::Adc;
use stm32g0_hal::exti::{Event, ExtiExt};
use stm32g0_hal::gpio::gpiob::{PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB6, PB7, PB8, PB9};
use stm32g0_hal::gpio::{Alternate, Analog, GpioExt, Input, PullUp, PushPull, SignalEdge};
use stm32g0_hal::i2c::{self, I2c, I2cExt};
use stm32g0_hal::pac::{interrupt, Interrupt};
use stm32g0_hal::pac::{CorePeripherals, Peripherals};
use stm32g0_hal::pac::{EXTI, I2C3, LPTIM2, NVIC, TIM4};
use stm32g0_hal::rcc::config::{Config, Prescaler};
use stm32g0_hal::rcc::RccExt;
use stm32g0_hal::timer::{LowPowerTimer, Pwm, Timer};
use stm32g0_hal::usb::UsbExt;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::LangID;
use usbd_serial::embedded_io::ReadReady;

use crate::error::Error;
use crate::system_time::Ticker;

pub type BoardI2c = I2c<I2C3>;
pub type Usb = UsbDevice<'static, stm32g0_hal::usb::Bus<stm32g0_hal::pac::USB>>;
pub type Serial = usbd_serial::SerialPort<'static, stm32g0_hal::usb::Bus<stm32g0_hal::pac::USB>>;
pub type MutexWithSerialPort = Mutex<RefCell<Serial>>;

#[allow(unused)]
pub struct Joystick {
    pub up: PB13<Input<PullUp>>,
    pub down: PB12<Input<PullUp>>,
    pub left: PB11<Input<PullUp>>,
    pub right: PB10<Input<PullUp>>,
    pub select: PB15<Input<PullUp>>,
    pub button: PB14<Input<PullUp>>,
}

pub struct VBat {
    adc: Adc,
    vbat: PB2<Analog>,
}

impl VBat {
    const VBAT_MULTIPLIER: f32 = 1.622;

    pub fn read_battery_volts(&mut self) -> f32 {
        let battery_mv = self.adc.read_voltage(&mut self.vbat);
        battery_mv as f32 / 1000.0 * Self::VBAT_MULTIPLIER
    }
}

pub struct Backlight {
    pub pwm: Pwm<TIM4>,
    pub red: PB6<Alternate<9>>,
    pub green: PB7<Alternate<9>>,
    pub blue: PB8<Alternate<9>>,
}

pub type DisplayPowerPin = PB9<stm32g0_hal::gpio::Output<PushPull>>;

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

    /// Get shared reference to the value.
    fn get(&self) -> &T {
        unsafe { (*self.0.get()).assume_init_ref() }
    }

    /// Get mutable reference to the value.
    #[allow(clippy::mut_from_ref)]
    fn get_mut(&self) -> &mut T {
        unsafe { (*self.0.get()).assume_init_mut() }
    }
}

// USB_BUS is initialized once and only used via reference later.
// Later bus is used from interrupt and async context, but it is Sync.
static USB_BUS: SafelyInitializedStatic<
    UsbBusAllocator<stm32g0_hal::usb::Bus<stm32g0_hal::pac::USB>>,
> = SafelyInitializedStatic::new();
// USB_SERIAL is used from both interrupt and async context.
static USB_SERIAL: SafelyInitializedStatic<MutexWithSerialPort> = SafelyInitializedStatic::new();
// USB_DEVICE is only used from interrupt context but needs mutable reference there.
static USB_DEVICE: SafelyInitializedStatic<Usb> = SafelyInitializedStatic::new();

pub struct Board {
    pub ticker: Ticker,
    pub i2c: RefCell<BoardI2c>,
    pub backlight: Backlight,
    pub vbat: VBat,
    pub joystick: Joystick,
    pub display_power: DisplayPowerPin,
    pub usb_serial: &'static MutexWithSerialPort,
}

impl Board {
    pub fn new() -> Result<Self, Error> {
        let _cp = CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = Peripherals::take().ok_or(Error::AlreadyTaken)?;

        // Enable debug while stopped to keep probe-rs happy while WFI
        // Enabling DMA resolves another instability issue:
        // https://github.com/probe-rs/probe-rs/issues/350
        #[cfg(debug_assertions)]
        {
            dp.DBG.cr().modify(|_, w| w.dbg_stop().set_bit());
            dp.RCC.ahbenr().modify(|_, w| w.dma1en().set_bit());
        }

        // Set clock to 4MHz (HSI speed is 16MHz).
        // Check I2C clock requirements (RM0444 32.4.4) before lowering.
        let clocks = Config::sysclk_hsi(Prescaler::Div1).enable_lsi();
        let rcc = dp.RCC.constrain(clocks);

        let gpiob = dp.GPIOB.split(&rcc);

        let backlight_pwm = Timer::<TIM4>::new(dp.TIM4).pwm(0, u16::MAX, &rcc);
        let mut adc = Adc::new(dp.ADC, &rcc);

        adc.calibrate();

        let mut joystick = Joystick {
            up: gpiob.pb13.into_pullup_input(),
            down: gpiob.pb12.into_pullup_input(),
            left: gpiob.pb11.into_pullup_input(),
            right: gpiob.pb10.into_pullup_input(),
            select: gpiob.pb15.into_pullup_input(),
            button: gpiob.pb14.into_pullup_input(),
        };

        let i2c_sda = gpiob.pb4.into_open_drain_output();
        let i2c_scl = gpiob.pb3.into_open_drain_output();
        let config = i2c::Config::from_cubemx(true, 0, 0x00503D58);
        let i2c = dp.I2C3.i2c(i2c_sda, i2c_scl, &config, &rcc);

        let system_timer = LowPowerTimer::<LPTIM2>::new(dp.LPTIM2);
        let ticker = Ticker::new(system_timer, &rcc);

        // Upon reset, a pull-down resistor might be present on PB15, PA8, PD0, or PD2,
        // depending on the voltage level on PB0, PA9, PC6, PA10, PD1, and PD3. In order
        // to disable this resistor, strobe the UCPDx_STROBE bit of the SYSCFG_CFGR1
        // register during start-up sequence.
        let syscfg = dp.SYSCFG;
        syscfg
            .cfgr1()
            .modify(|_, w| w.ucpd1_strobe().set_bit().ucpd2_strobe().set_bit());

        let mut exti = dp.EXTI;
        joystick.up.make_interrupt_source(&mut exti);
        joystick.up.trigger_on_edge(SignalEdge::Falling, &mut exti);
        joystick.down.make_interrupt_source(&mut exti);
        joystick
            .down
            .trigger_on_edge(SignalEdge::Falling, &mut exti);
        joystick.left.make_interrupt_source(&mut exti);
        joystick
            .left
            .trigger_on_edge(SignalEdge::Falling, &mut exti);
        joystick.right.make_interrupt_source(&mut exti);
        joystick
            .right
            .trigger_on_edge(SignalEdge::Falling, &mut exti);
        joystick.button.make_interrupt_source(&mut exti);
        joystick
            .button
            .trigger_on_edge(SignalEdge::Falling, &mut exti);
        joystick.select.make_interrupt_source(&mut exti);
        joystick
            .select
            .trigger_on_edge(SignalEdge::Falling, &mut exti);

        exti.listen(Event::Gpio10);
        exti.listen(Event::Gpio11);
        exti.listen(Event::Gpio12);
        exti.listen(Event::Gpio13);
        exti.listen(Event::Gpio14);
        exti.listen(Event::Gpio15);
        exti.listen(Event::LpTim2);

        let usb_bus = USB_BUS.write(dp.USB.constrain(&rcc));
        let usb_serial = USB_SERIAL.write(Mutex::new(RefCell::new(usbd_serial::SerialPort::new(
            usb_bus,
        ))));
        USB_DEVICE.write(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x5824, 0x5432))
                .strings(&[StringDescriptors::new(LangID::EN)
                    .product("Weather Station")
                    .manufacturer("Blaze")
                    .serial_number("initial")])
                .expect("Failed to set strings")
                .device_class(usbd_serial::USB_CLASS_CDC)
                .device_release(0x0001)
                .self_powered(true)
                .build(),
        );

        unsafe {
            NVIC::unmask(Interrupt::TIM7);
            NVIC::unmask(Interrupt::EXTI4_15);
            NVIC::unmask(Interrupt::UCPD1_UCPD2_USB);
        }

        USB_DEVICE.get().bus().enable_interrupts();

        Ok(Self {
            ticker,
            i2c: RefCell::new(i2c),
            joystick,
            vbat: VBat {
                adc,
                vbat: gpiob.pb2.into_analog(),
            },
            backlight: Backlight {
                pwm: backlight_pwm,
                red: gpiob.pb6.into_alternate_function(),
                green: gpiob.pb7.into_alternate_function(),
                blue: gpiob.pb8.into_alternate_function(),
            },
            display_power: gpiob.pb9.into_push_pull_output(),
            usb_serial,
        })
    }
}

pub static JOYSTICK_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
unsafe fn EXTI4_15() {
    let exti = &(*EXTI::ptr());

    debug_rprintln!("button interrupt {:016b}", exti.fpr1().read().bits());
    debug_rprintln!("button mask      5432109876543210");
    JOYSTICK_EVENT.post(());

    // Clear interrupt for joystick GPIO lines
    exti.fpr1().write(|w| {
        w.fpif10()
            .clear()
            .fpif11()
            .clear()
            .fpif12()
            .clear()
            .fpif13()
            .clear()
            .fpif14()
            .clear()
            .fpif15()
            .clear()
    });
}

pub static USB_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
fn UCPD1_UCPD2_USB() {
    debug_rprintln!("USB interrupt");
    critical_section::with(|cs| {
        let usb_device = USB_DEVICE.get_mut();
        let mut serial = USB_SERIAL.get().borrow_ref_mut(cs);

        if usb_device.poll(&mut [&mut *serial]) {
            // Force serial port to read data from USB buffer to internal buffer.
            // Otherwise "transfer completed" interrupt will not be cleared.
            // As a bonus, wake up serial reader only when the data is actually available.
            if let Ok(true) = serial.read_ready() {
                USB_EVENT.post(());
            }
        }
    });
}
