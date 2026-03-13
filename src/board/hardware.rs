#![allow(unsafe_code)]

use core::cell::RefCell;

use embedded_hal::digital::{InputPin, OutputPin};
use stm32g0_hal::adc::{Adc, AdcExt};
use stm32g0_hal::exti::{Event, ExtiExt};
use stm32g0_hal::gpio::gpiob::{PB2, PB9, PB10, PB11, PB12, PB13, PB14, PB15};
use stm32g0_hal::gpio::{Analog, GpioExt, Input, PullUp, PushPull, SignalEdge};
use stm32g0_hal::i2c::{self, I2c, I2cExt};
use stm32g0_hal::iwdg::{Iwdg, IwdgPrescaler};
use stm32g0_hal::pac::{CorePeripherals, Peripherals};
use stm32g0_hal::pac::{EXTI, I2C3, NVIC, TIM4};
use stm32g0_hal::pac::{Interrupt, interrupt};
use stm32g0_hal::rcc::RccExt;
use stm32g0_hal::rcc::config::{Config, Prescaler};
use stm32g0_hal::timer::{Channel1, Channel2, Channel3, LptimExt, Pwm, PwmPin, TimerExt};
use stm32g0_hal::usb::UsbExt;

use crate::error::Error;
use crate::system_time::Ticker;

use super::statics::SafelyInitializedStatic;
use super::usb::{UsbPowerControl, UsbSerialPort};

pub type I2cBus = I2c<I2C3>;

pub struct Joystick {
    up: PB13<Input<PullUp>>,
    down: PB14<Input<PullUp>>,
    left: PB11<Input<PullUp>>,
    right: PB12<Input<PullUp>>,
    select: PB15<Input<PullUp>>,
    button: PB10<Input<PullUp>>,
}

impl crate::types::Joystick for Joystick {
    fn up(&mut self) -> bool {
        self.up.is_low().expect("use into_ok()")
    }

    fn down(&mut self) -> bool {
        self.down.is_low().expect("use into_ok()")
    }

    fn left(&mut self) -> bool {
        self.left.is_low().expect("use into_ok()")
    }

    fn right(&mut self) -> bool {
        self.right.is_low().expect("use into_ok()")
    }

    fn select(&mut self) -> bool {
        self.select.is_low().expect("use into_ok()")
    }

    fn button(&mut self) -> bool {
        self.button.is_low().expect("use into_ok()")
    }
}

impl crate::types::EventWaiter for Joystick {
    async fn wait(&self) {
        JOYSTICK_EVENT.read().await.expect("use into_ok()");
    }
}

pub struct VBat {
    adc: Adc,
    vbat: PB2<Analog>,
}

impl VBat {
    // Ratio of the voltage divider on the board.
    const VBAT_NUMERATOR: u32 = 1622;
    const VBAT_DENOMINATOR: u32 = 1000;
}

impl crate::types::VoltageReader for VBat {
    fn millivolts(&mut self) -> u16 {
        let adc_mv = self.adc.read_voltage(&mut self.vbat);
        let battery_mv = adc_mv as u32 * Self::VBAT_NUMERATOR / Self::VBAT_DENOMINATOR;
        battery_mv as u16
    }
}

pub type Backlight = crate::types::Backlight<
    PwmPin<'static, TIM4, Channel1>,
    PwmPin<'static, TIM4, Channel2>,
    PwmPin<'static, TIM4, Channel3>,
>;

pub struct DisplayPowerPin(PB9<stm32g0_hal::gpio::Output<PushPull>>);

impl DisplayPowerPin {
    fn new(pin: PB9<stm32g0_hal::gpio::Output<PushPull>>) -> Self {
        Self(pin)
    }
}

impl crate::types::OnOff for DisplayPowerPin {
    fn on(&mut self) {
        self.0.set_low().expect("use into_ok()");
    }

    fn off(&mut self) {
        self.0.set_high().expect("use into_ok()");
    }
}

pub struct EventWaiter(&'static async_scheduler::sync::mailbox::Mailbox<()>);

impl EventWaiter {
    pub fn new(event: &'static async_scheduler::sync::mailbox::Mailbox<()>) -> Self {
        Self(event)
    }
}

impl crate::types::EventWaiter for EventWaiter {
    async fn wait(&self) {
        self.0.read().await.expect("Error waiting for event")
    }
}

pub struct Watchdog(Iwdg);

impl crate::types::Watchdog for Watchdog {
    fn feed(&self) {
        self.0.feed();
    }
}

pub type Board = crate::types::Board<
    Joystick,
    VBat,
    DisplayPowerPin,
    UsbSerialPort,
    UsbPowerControl,
    EventWaiter,
    Watchdog,
    I2cBus,
    PwmPin<'static, TIM4, Channel1>,
    PwmPin<'static, TIM4, Channel2>,
    PwmPin<'static, TIM4, Channel3>,
>;

pub fn init() -> Result<(super::env::Env, Board), Error> {
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

    // Set clock to 16MHz.
    // Check I2C clock requirements (RM0444 32.4.4) before lowering.
    let clocks = Config::sysclk_hsi(Prescaler::Div1).enable_lsi();
    let rcc = dp.RCC.constrain(clocks);

    let gpioa = dp.GPIOA.split(&rcc);
    let _pa10 = gpioa.pa10.into_push_pull_output();
    let _pa15 = gpioa.pa15.into_push_pull_output();
    let gpiob = dp.GPIOB.split(&rcc);

    let mut pwrctl = gpiob.pb0.into_push_pull_output();
    pwrctl.set_high()?;

    let backlight_pwm = BACKLIGHT_PWM.write(dp.TIM4.constrain().pwm(0, u16::MAX, &rcc));
    let mut adc = dp.ADC.constrain(&rcc);

    adc.calibrate();

    let mut joystick = Joystick {
        up: gpiob.pb13.into_pullup_input(),
        down: gpiob.pb14.into_pullup_input(),
        left: gpiob.pb11.into_pullup_input(),
        right: gpiob.pb12.into_pullup_input(),
        select: gpiob.pb15.into_pullup_input(),
        button: gpiob.pb10.into_pullup_input(),
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

    let mut charger_interrupt = gpiob.pb5.into_floating_input();
    charger_interrupt.make_interrupt_source(&mut exti);
    charger_interrupt.trigger_on_edge(SignalEdge::Falling, &mut exti);

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

    exti.listen(Event::Gpio5);
    exti.listen(Event::Gpio10);
    exti.listen(Event::Gpio11);
    exti.listen(Event::Gpio12);
    exti.listen(Event::Gpio13);
    exti.listen(Event::Gpio14);
    exti.listen(Event::Gpio15);
    exti.listen(Event::LpTim2);

    let usb_serial = super::usb::serial_port(dp.USB.constrain(&rcc));

    unsafe {
        NVIC::unmask(Interrupt::TIM7);
        NVIC::unmask(Interrupt::EXTI4_15);
        NVIC::unmask(Interrupt::UCPD1_UCPD2_USB);
    }

    let iwdg = Iwdg::new(dp.IWDG);
    iwdg.start(IwdgPrescaler::Div256, 0xFFF);

    Ok((
        super::env::Env::new(ticker),
        Board {
            joystick,
            vbat: VBat {
                adc,
                vbat: gpiob.pb2.into_analog(),
            },
            backlight: Backlight {
                red: backlight_pwm.bind_pin(gpiob.pb6.into_alternate_function()),
                green: backlight_pwm.bind_pin(gpiob.pb7.into_alternate_function()),
                blue: backlight_pwm.bind_pin(gpiob.pb8.into_alternate_function()),
            },
            display_power: DisplayPowerPin::new(gpiob.pb9.into_push_pull_output()),
            usb_serial,
            usb_power: UsbPowerControl,
            charger_event: EventWaiter::new(&CHARGER_EVENT),
            i2c: RefCell::new(i2c),
            watchdog: Watchdog(iwdg),
        },
    ))
}

static CHARGER_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();
static JOYSTICK_EVENT: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();
static BACKLIGHT_PWM: SafelyInitializedStatic<Pwm<TIM4>> = SafelyInitializedStatic::new();

#[interrupt]
fn EXTI4_15() {
    let exti = unsafe { EXTI::steal() };

    if exti.is_pending(Event::Gpio5, SignalEdge::Falling) {
        // Charger interrupt
        CHARGER_EVENT.post(());
        exti.unpend(Event::Gpio5);
    }

    if exti.fpr1().read().bits() != 0 {
        // Remaining lines are assigned to joystick.
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
}
