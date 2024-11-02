use crate::microhal::pac::adc::chselr1;
use crate::microhal::pac::ADC;

use super::gpio::gpiob::{PB0, PB1, PB10, PB2};
use super::gpio::Analog;
use super::rcc::{Rcc, ResetEnable};

pub struct Adc {
    adc: ADC,
    vref_cache: Option<u16>,
}

impl Adc {
    pub fn new(adc: ADC, rcc: &Rcc) -> Self {
        ADC::enable(rcc);
        ADC::reset(rcc);

        // Enable ADC voltage regulator.
        adc.cr().modify(|_, w| w.advregen().enabled());

        // RM0444 15.3.3 Calibration can only be initiated when the ADC voltage regulator is
        // enabled (ADVREGEN = 1 and tADCVREG_SETUP has elapsed) and the ADC is disabled
        // (when ADEN = 0).
        // tADCVREG_SETUP = 20us, at 80MHz it's 80*20 = 1600 cycles.
        // Round up for a safety margin.
        cortex_m::asm::delay(2000);

        adc.cfgr1().modify(|_, w| w.chselrmod().sequence());

        // Set clock to PCLK/2.
        // TODO: make this configurable.
        adc.cfgr2().modify(|_, w| w.ckmode().pclk_div2());

        Self {
            adc,
            vref_cache: None,
        }
    }

    pub fn calibrate(&mut self) {
        self.adc.isr().write(|w| w.eocal().clear());
        self.adc.cr().modify(|_, w| w.adcal().start_calibration());
        while self.adc.isr().read().eocal().is_not_complete() {}
        self.adc.isr().write(|w| w.eocal().clear());

        // Doesn't work for some reason. VRef reads ok, but pin returns 0.
        // Enable auto-on-off mode.
        // self.adc.cfgr1.modify(|_, w| w.autoff().enabled());
    }

    fn power_on(&mut self) {
        self.adc.cr().modify(|_, w| w.aden().enabled());
        while self.adc.isr().read().adrdy().is_not_ready() {}
    }

    fn power_off(&mut self) {
        self.adc.cr().modify(|_, w| w.addis().disable());
        while self.adc.isr().read().adrdy().is_ready() {}
    }

    pub fn read<PIN>(&mut self, pin: &mut PIN) -> u16
    where
        PIN: AdcPin,
    {
        self.power_on();

        self.adc.isr().modify(|_, w| w.ccrdy().clear());
        // Set pin as the only channel.
        self.adc
            .chselr1()
            .write(|w| w.sq1().variant(pin.channel()).sq2().eos());
        while self.adc.isr().read().ccrdy().is_not_complete() {}

        // Do conversion.
        self.adc.isr().modify(|_, w| w.eos().clear());
        self.adc.cr().modify(|_, w| w.adstart().start_conversion());
        // Wait for conversion to complete.
        while self.adc.isr().read().eos().is_not_complete() {}

        self.power_off();
        self.adc.dr().read().data().bits()
    }

    pub fn read_voltage<PIN>(&mut self, pin: &mut PIN) -> u16
    where
        PIN: AdcPin,
    {
        let vref: u32 = self.read_vref_cached().into();
        let raw: u32 = self.read(pin).into();

        let adc_mv = (vref * raw) >> 12;
        adc_mv as u16
    }

    pub fn read_vref(&mut self) -> u16 {
        self.adc.ccr().modify(|_, w| w.vrefen().enabled());
        let val: u32 = self.read(&mut VRef).into();
        self.adc.ccr().modify(|_, w| w.vrefen().disabled());

        #[allow(unsafe_code)]
        let vref_cal: u32 = unsafe {
            // DS12766 3.13.2
            core::ptr::read_volatile(0x1FFF_75AA as *const u16) as u32
        };

        // RM0454 14.9 Calculating the actual VDDA voltage using the internal reference voltage
        // V_DDA = 3 V x VREFINT_CAL / VREFINT_DATA
        let vref = (vref_cal * 3_000_u32 / val) as u16;
        self.vref_cache = Some(vref);
        vref
    }

    pub fn read_vref_cached(&mut self) -> u16 {
        if let Some(vref) = self.vref_cache {
            return vref;
        }

        self.read_vref()
    }
}

pub struct VRef;

pub trait AdcPin {
    fn channel(&self) -> chselr1::SQ1;
}

macro_rules! adc_pin {
    ($name:ident, $channel:ident) => {
        impl AdcPin for $name<Analog> {
            fn channel(&self) -> chselr1::SQ1 {
                chselr1::SQ1::$channel
            }
        }
    };
}

adc_pin!(PB0, Ch8);
adc_pin!(PB1, Ch9);
adc_pin!(PB2, Ch10);
adc_pin!(PB10, Ch11);
// adc_pin!(PB11, Ch15);
// adc_pin!(PB12, Ch16);

impl AdcPin for VRef {
    fn channel(&self) -> chselr1::SQ1 {
        chselr1::SQ1::Ch13
    }
}
