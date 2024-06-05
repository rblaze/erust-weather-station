#![allow(unused)]
use core::marker::PhantomData;

use stm32g0::stm32g071::{TIM2, TIM3};

use crate::microhal::gpio::gpiob::{PB0, PB1, PB4, PB5};
use crate::microhal::gpio::Alternate;
use crate::microhal::rcc::{RccControl, ResetEnable};

/// Wrapper for timer peripheral.
#[derive(Debug)]
pub struct Timer<TIM> {
    timer: TIM,
}

/// Counting timer
#[derive(Debug)]
pub struct Counter<TIM> {
    timer: TIM,
}

/// PWM timer
#[derive(Debug)]
pub struct Pwm<TIM> {
    timer: TIM,
}

macro_rules! general_purpose_timer {
    ($TIM:ident, $REG:tt) => {
        impl Timer<$TIM> {
            pub fn new(timer: $TIM) -> Self {
                Self { timer }
            }

            pub fn upcounter(self, prescaler: u16, limit: $REG, rcc: &RccControl) -> Counter<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc.write(|w| w.psc().bits(prescaler));

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr.write(|w| unsafe { w.bits(limit.into()) });

                Counter { timer: self.timer }
            }

            pub fn pwm(self, prescaler: u16, limit: $REG, rcc: &RccControl) -> Pwm<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc.write(|w| w.psc().bits(prescaler));
                self.timer.cr1.modify(|_, w| w.arpe().enabled());

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr.write(|w| unsafe { w.bits(limit.into()) });

                // Generate update event to copy ARR to shadow register
                self.timer.egr.write(|w| w.ug().update());
                while self.timer.sr.read().uif().bit_is_clear() {}
                self.timer.sr.modify(|_, w| w.uif().clear_bit());

                // Enable timer
                self.timer.cr1.modify(|_, w| w.cen().set_bit());

                Pwm { timer: self.timer }
            }
        }

        impl Counter<$TIM> {
            pub fn start(&self) {
                self.timer.cr1.modify(|_, w| w.cen().set_bit());
            }

            pub fn stop(&self) {
                self.timer.cr1.modify(|_, w| w.cen().clear_bit());
            }

            basic_timer_impl!($TIM, $REG);
        }
    };
}

macro_rules! basic_timer_impl {
    ($TIM:ident, u16) => {
        pub fn counter(&self) -> u16 {
            self.timer.cnt.read().cnt_l().bits()
        }
    };
    ($TIM:ident, u32) => {
        pub fn counter(&self) -> u32 {
            self.timer.cnt.read().bits()
        }
    };
}

general_purpose_timer!(TIM2, u32);
general_purpose_timer!(TIM3, u16);

pub struct Channel1;

pub struct Channel2;

pub struct Channel3;

pub struct Channel4;

pub trait TimerPin<TIM> {
    type Channel;
    fn set_pwm_mode(&self, tim: &TIM);
}

pub struct PwmPin<'a, TIM, CH> {
    timer: &'a TIM,
    channel: PhantomData<CH>,
}

impl<TIM, CH> embedded_hal::pwm::ErrorType for PwmPin<'_, TIM, CH> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for PwmPin<'_, TIM3, Channel1> {
    fn max_duty_cycle(&self) -> u16 {
        self.timer.arr.read().arr_l().bits()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.timer.ccr1.write(|w| w.ccr1_l().variant(duty));
        Ok(())
    }
}

impl embedded_hal::pwm::SetDutyCycle for PwmPin<'_, TIM3, Channel2> {
    fn max_duty_cycle(&self) -> u16 {
        self.timer.arr.read().arr_l().bits()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.timer.ccr2.write(|w| w.ccr2_l().variant(duty));
        Ok(())
    }
}

impl embedded_hal::pwm::SetDutyCycle for PwmPin<'_, TIM3, Channel3> {
    fn max_duty_cycle(&self) -> u16 {
        self.timer.arr.read().arr_l().bits()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.timer.ccr3.write(|w| w.ccr3_l().variant(duty));
        Ok(())
    }
}

impl embedded_hal::pwm::SetDutyCycle for PwmPin<'_, TIM3, Channel4> {
    fn max_duty_cycle(&self) -> u16 {
        self.timer.arr.read().arr_l().bits()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.timer.ccr4.write(|w| w.ccr4_l().variant(duty));
        Ok(())
    }
}

impl TimerPin<TIM3> for PB0<Alternate<1>> {
    type Channel = Channel3;

    fn set_pwm_mode(&self, tim: &TIM3) {
        // Disable channel
        tim.ccer.modify(|_, w| w.cc3e().clear_bit());
        // Set PWM mode and preload
        tim.ccmr2_output()
            .modify(|_, w| w.oc3m().pwm_mode1().oc3pe().set_bit());
        // Enable channel
        tim.ccer.modify(|_, w| w.cc3e().set_bit());
    }
}

impl TimerPin<TIM3> for PB1<Alternate<1>> {
    type Channel = Channel4;

    fn set_pwm_mode(&self, tim: &TIM3) {
        tim.ccer.modify(|_, w| w.cc4e().clear_bit());
        tim.ccmr2_output()
            .modify(|_, w| w.oc4m().pwm_mode1().oc4pe().set_bit());
        tim.ccer.modify(|_, w| w.cc4e().set_bit());
    }
}

impl TimerPin<TIM3> for PB4<Alternate<1>> {
    type Channel = Channel1;

    fn set_pwm_mode(&self, tim: &TIM3) {
        tim.ccer.modify(|_, w| w.cc1e().clear_bit());
        tim.ccmr1_output()
            .modify(|_, w| w.oc1m().pwm_mode1().oc1pe().set_bit());
        tim.ccer.modify(|_, w| w.cc1e().set_bit());
    }
}

impl TimerPin<TIM3> for PB5<Alternate<1>> {
    type Channel = Channel2;

    fn set_pwm_mode(&self, tim: &TIM3) {
        tim.ccer.modify(|_, w| w.cc2e().clear_bit());
        tim.ccmr1_output()
            .modify(|_, w| w.oc2m().pwm_mode1().oc2pe().set_bit());
        tim.ccer.modify(|_, w| w.cc2e().set_bit());
    }
}

impl Pwm<TIM3> {
    pub fn bind_pin<PIN>(&self, pin: PIN) -> PwmPin<TIM3, PIN::Channel>
    where
        PIN: TimerPin<TIM3>,
    {
        // Set channel mode to PWM1
        pin.set_pwm_mode(&self.timer);

        PwmPin {
            timer: &self.timer,
            channel: PhantomData,
        }
    }
}
