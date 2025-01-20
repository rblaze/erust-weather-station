use core::cell::RefCell;

use rtt_target::debug_rprintln;
use stm32g0_hal::adc::{Adc, AdcExt};
use stm32g0_hal::exti::{Event, ExtiExt};
use stm32g0_hal::gpio::gpioa::PA15;
use stm32g0_hal::gpio::gpiob::{PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB6, PB7, PB8, PB9};
use stm32g0_hal::gpio::{Alternate, Analog, GpioExt, Input, PullUp, PushPull, SignalEdge};
use stm32g0_hal::i2c::{self, I2c, I2cExt};
use stm32g0_hal::pac::{interrupt, Interrupt};
use stm32g0_hal::pac::{CorePeripherals, Peripherals};
use stm32g0_hal::pac::{EXTI, I2C3, NVIC, TIM4};
use stm32g0_hal::rcc::config::{Config, Prescaler};
use stm32g0_hal::rcc::RccExt;
use stm32g0_hal::timer::{LptimExt, Pwm, TimerExt};
use stm32g0_hal::usb::UsbExt;

use crate::error::Error;
use crate::system_time::Ticker;

mod board_usb;
pub use board_usb::*;

pub type BoardI2c = I2c<I2C3>;

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

pub struct Board {
    pub ticker: Ticker,
    pub i2c: RefCell<BoardI2c>,
    pub backlight: Backlight,
    pub vbat: VBat,
    pub joystick: Joystick,
    pub display_power: DisplayPowerPin,
    pub usb_serial: UsbSerialPort,
    pub on_external_power: fn(),
    pub on_battery: fn(),
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

        let gpioa = dp.GPIOA.split(&rcc);
        let pa15 = gpioa.pa15.into_push_pull_output();
        let _noe: PA15<Alternate<6>> = pa15.into_alternate_function();
        let gpiob = dp.GPIOB.split(&rcc);

        let backlight_pwm = dp.TIM4.constrain().pwm(0, u16::MAX, &rcc);
        let mut adc = dp.ADC.constrain(&rcc);

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
        let i2c = dp.I2C3.constrain(i2c_sda, i2c_scl, &config, &rcc);

        let system_timer = dp.LPTIM2.constrain();
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

        let usb_serial = board_usb::serial_port(dp.USB.constrain(&rcc));

        unsafe {
            NVIC::unmask(Interrupt::TIM7);
            NVIC::unmask(Interrupt::EXTI4_15);
            NVIC::unmask(Interrupt::UCPD1_UCPD2_USB);
        }

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
            on_external_power,
            on_battery,
        })
    }
}

fn on_external_power() {
    board_usb::on_external_power();
}

fn on_battery() {
    board_usb::on_battery();
}

pub static JOYSTICK_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
unsafe fn EXTI4_15() {
    let exti = &(*EXTI::ptr());

    debug_rprintln!("EXTI interrupt {:016b}", exti.fpr1().read().bits());
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
