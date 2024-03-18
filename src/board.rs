use core::cmp;
use core::iter::from_fn;

use embedded_time::rate::Hertz;
use rtt_target::debug_rprintln;
use stm32l0xx_hal::exti::{DirectLine, Exti, ExtiLine, GpioLine};
use stm32l0xx_hal::gpio::gpioc::{PC0, PC1};
use stm32l0xx_hal::gpio::{GpioExt, OpenDrain, Output, Port};
use stm32l0xx_hal::i2c::I2c;
use stm32l0xx_hal::pac::{self, interrupt, EXTI, I2C3, RCC, TIM21};
use stm32l0xx_hal::pwr::PWR;
use stm32l0xx_hal::rcc::{self, Enable, RccExt};
use stm32l0xx_hal::syscfg::SYSCFG;

use crate::error::Error;
use crate::hal_i2c::I2cBus;
use crate::system_time::Ticker;

pub type I2cSda = PC1<Output<OpenDrain>>;
pub type I2cScl = PC0<Output<OpenDrain>>;
type I2cBus3 = I2c<I2C3, I2cSda, I2cScl>;

pub struct Board {
    pub ticker: Ticker,
    pub i2c: I2cBus<I2cBus3>,
}

impl Board {
    pub fn new(cp: pac::CorePeripherals, mut dp: pac::Peripherals) -> Result<Self, Error> {
        // Enable debug while sleeping to keep probe-rs happy while WFI
        #[cfg(debug_assertions)]
        #[rustfmt::skip]
        dp.DBG.cr.modify(|_, w| { w
            .dbg_sleep().enabled()
            .dbg_standby().enabled()
            .dbg_stop().enabled()
        });

        let lsi_freq = unsafe { Self::measure_lsi(&mut dp.RCC) };
        debug_rprintln!("LSI freq: {}", lsi_freq);

        let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range5));
        let pwr = PWR::new(dp.PWR, &mut rcc);
        let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let _button = gpioc.pc13.into_floating_input();

        let i2c_sda = gpioc.pc1.into_open_drain_output();
        let i2c_scl = gpioc.pc0.into_open_drain_output();
        let i2c = I2c::new(dp.I2C3, i2c_sda, i2c_scl, Hertz(50_000), &mut rcc);

        let ticker = Ticker::new(dp.LPTIM, rcc, pwr, cp.SCB, Hertz(lsi_freq));

        let mut exti = Exti::new(dp.EXTI);
        let exti_line = DirectLine::Lptim1;
        exti.listen_direct(exti_line);

        // Enable button interrupt to force-wakeup the board.
        exti.listen_gpio(
            &mut syscfg,
            Port::PC,
            GpioLine::from_raw_line(13).unwrap(),
            stm32l0xx_hal::exti::TriggerEdge::Falling,
        );

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::LPTIM1);
            pac::NVIC::unmask(pac::Interrupt::EXTI4_15);
        }

        Ok(Self {
            ticker,
            i2c: I2cBus::new(i2c),
        })
    }

    /// Measure LSI clock frequency against HSI16 clock.
    unsafe fn measure_lsi(rcc: &mut RCC) -> u32 {
        // Taking `&mut RCC` because `Rcc` can't be released and
        // clock switched to MSI.
        //
        // Measurement per RM0367 7.2.15 and AN4631 3.1

        // Wait for HSI16 to be ready.
        rcc.cr.modify(|_, w| w.hsi16on().enabled());
        while rcc.cr.read().hsi16rdyf().is_not_ready() {}

        // Use HSI16 as system clock.
        rcc.cfgr
            .modify(|_, w| w.sw().hsi16().hpre().div1().ppre1().div1().ppre2().div1());
        // Wait for switch.
        while !rcc.cfgr.read().sws().is_hsi16() {}

        // Turn on LSI.
        rcc.csr.modify(|_, w| w.lsion().on());
        // Wait for LSI to be ready
        while rcc.csr.read().lsirdy().is_not_ready() {}

        // Enable TIM21.
        TIM21::enable_unchecked();
        cortex_m::asm::dsb();

        // Enable clock output (MCO) from LSI with a divider of 16.
        rcc.cfgr.modify(|_, w| w.mcosel().lsi().mcopre().div16());

        let tim21 = &(*TIM21::ptr());
        // Count up, edge aligned, count every pulse.
        #[rustfmt::skip]
        tim21.cr1.modify(|_, w| w
            .dir().up()
            .cms().edge_aligned()
            .ckd().div1()
        );
        tim21.arr.write(|w| w.bits(0xFFFF));
        tim21.psc.write(|w| w.bits(0));
        // Set Input Capture parameters.
        #[rustfmt::skip]
        tim21.ccmr1_input().modify(|_, w| w
            .cc1s().ti1()
            .ic1f().no_filter()
        );
        // Capture on rising edge.
        tim21
            .ccer
            .modify(|_, w| w.cc1p().rising_edge().cc1np().clear_bit());

        // Connect MCO to TIM21 input TI1
        // RM0367 22.4.14 says `111: TIM21 TI1 input connected to MCO clock`
        tim21.or.modify(|_, w| w.ti1_rmp().bits(0b111));

        let num_iterations = 10_u32;
        let frequency = from_fn(|| Some(Self::capture_frequency(tim21)))
            .take(num_iterations as usize)
            .sum::<u32>()
            / num_iterations;

        // Disable TIM21.
        TIM21::disable_unchecked();
        cortex_m::asm::dsb();

        // Switch system clock back to MSI (default mode)
        rcc.cfgr
            .modify(|_, w| w.sw().msi().hpre().div1().ppre1().div1().ppre2().div1());
        // Wait for switch.
        while !rcc.cfgr.read().sws().is_msi() {}

        // Disable HSI16
        rcc.cr.modify(|_, w| w.hsi16on().disabled());

        // Leave LSI running. We are going to use it, right?
        frequency
    }

    fn capture_frequency(tim21: &pac::tim21::RegisterBlock) -> u32 {
        // Generate Update event to load ARR and PSC registers.
        tim21.egr.write(|w| w.ug().update());

        // Start the capture.
        tim21.ccer.modify(|_, w| w.cc1e().enabled());
        tim21.cr1.modify(|_, w| w.cen().enabled());

        // Wait for value ready.
        let counter1: u32 = Self::read_capture(tim21).into();

        // Wait for second value ready.
        let counter2 = Self::read_capture(tim21).into();

        // Stop the capture.
        tim21.cr1.modify(|_, w| w.cen().disabled());
        tim21.ccer.modify(|_, w| w.cc1e().disabled());

        // Compute frequency.
        let difference = match counter1.cmp(&counter2) {
            cmp::Ordering::Less => counter2 - counter1,
            cmp::Ordering::Greater => 0xffff - counter1 + counter2,
            cmp::Ordering::Equal => panic!("Can't measure LSI frequency, counters are equal"),
        };

        // Multiply by 16 because the MCO divider is 16.
        // System clock is 16MHz (HSI16).
        16 * 16_000_000 / difference
    }

    fn read_capture(tim21: &pac::tim21::RegisterBlock) -> u16 {
        loop {
            let flags = tim21.sr.read();
            if flags.cc1of().is_overcapture() {
                panic!("Overcapture");
            }
            if flags.cc1if().bit_is_set() {
                break;
            }
        }

        tim21.ccr1.read().ccr().bits()
    }
}

#[interrupt]
unsafe fn EXTI4_15() {
    debug_rprintln!("button interrupt");
    // Clear interrupts.
    (*EXTI::ptr()).pr.write(|w| w.pif13().clear());
}
