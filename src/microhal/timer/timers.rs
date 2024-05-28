#![allow(unused)]
use core::marker::PhantomData;

use stm32g0::stm32g071::{TIM2, TIM3};

use crate::microhal::gpio::gpiob::{PB0, PB1};
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

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr.write(|w| unsafe { w.bits(limit.into()) });

                // TODO: set PWM mode

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
}

pub struct PwmPin<TIM, CH> {
    timer: PhantomData<TIM>,
    channel: PhantomData<CH>,
}

impl<TIM, CH> embedded_hal::pwm::ErrorType for PwmPin<TIM, CH> {
    type Error = core::convert::Infallible;
}

impl<TIM, CH> embedded_hal::pwm::SetDutyCycle for PwmPin<TIM, CH> {
    fn max_duty_cycle(&self) -> u16 {
        todo!()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        todo!()
    }
}

impl TimerPin<TIM3> for PB0<Alternate> {
    type Channel = Channel3;
}

impl TimerPin<TIM3> for PB1<Alternate> {
    type Channel = Channel4;
}

impl Pwm<TIM3> {
    pub fn bind_pin<PIN>(&self, pin: PIN) -> PwmPin<TIM3, PIN::Channel>
    where
        PIN: TimerPin<TIM3>,
    {
        // TODO: set alternate function
        PwmPin {
            timer: PhantomData,
            channel: PhantomData,
        }
    }
}
