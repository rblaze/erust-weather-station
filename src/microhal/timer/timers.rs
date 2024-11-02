#![allow(unused)]
use core::marker::PhantomData;

use crate::microhal::pac::{TIM2, TIM3};

use crate::microhal::gpio::gpiob::{PB0, PB1, PB4, PB5};
use crate::microhal::gpio::Alternate;
use crate::microhal::rcc::{Rcc, ResetEnable};

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

            pub fn upcounter(self, prescaler: u16, limit: $REG, rcc: &Rcc) -> Counter<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc().write(|w| w.psc().set(prescaler));

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr().write(|w| unsafe { w.arr().bits(limit) });

                Counter { timer: self.timer }
            }

            pub fn pwm(self, prescaler: u16, limit: $REG, rcc: &Rcc) -> Pwm<$TIM> {
                $TIM::enable(rcc);
                $TIM::reset(rcc);

                self.timer.psc().write(|w| w.psc().set(prescaler));
                self.timer.cr1().modify(|_, w| w.arpe().enabled());

                // TODO: update after switching to g0b1
                #[allow(unsafe_code)]
                self.timer.arr().write(|w| unsafe { w.arr().bits(limit) });

                // Generate update event to copy ARR to shadow register
                self.timer.egr().write(|w| w.ug().update());
                while self.timer.sr().read().uif().bit_is_clear() {}
                self.timer.sr().modify(|_, w| w.uif().clear_bit());

                // Enable timer
                self.timer.cr1().modify(|_, w| w.cen().set_bit());

                Pwm { timer: self.timer }
            }
        }

        impl Counter<$TIM> {
            pub fn start(&self) {
                self.timer.cr1().modify(|_, w| w.cen().set_bit());
            }

            pub fn stop(&self) {
                self.timer.cr1().modify(|_, w| w.cen().clear_bit());
            }

            pub fn counter(&self) -> $REG {
                self.timer.cnt().read().cnt().bits()
            }
        }

        impl Pwm<$TIM> {
            pub fn bind_pin<PIN>(&self, pin: PIN) -> PwmPin<$TIM, PIN::Channel>
            where
                PIN: TimerPin<$TIM>,
            {
                // Set channel mode to PWM1
                pin.set_pwm_mode(&self.timer);

                PwmPin {
                    timer: &self.timer,
                    channel: PhantomData,
                }
            }
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

macro_rules! pwm {
    ($TIM:ident, [$($CH:ident => $CCR:ident, $ENBIT:ident, $CCMR:ident, $MODE:ident, $POLBIT:ident,
        [$(($PIN:ident,$AF:literal),)+],)+]) => {
        $(
            // Implement SetDutyCycle for each channel of this timer.
            impl embedded_hal::pwm::SetDutyCycle for PwmPin<'_, $TIM, $CH> {
                fn max_duty_cycle(&self) -> u16 {
                    self.timer.arr().read().arr().bits()
                }

                fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
                    #[allow(unsafe_code)]
                    self.timer.$CCR().write(|w| unsafe { w.ccr().bits(duty) });
                    Ok(())
                }
            }

            // Implement TimerPin for each pin of each channel.
            $(
                impl TimerPin<$TIM> for $PIN<Alternate<$AF>> {
                    type Channel = $CH;

                    fn set_pwm_mode(&self, tim: &$TIM) {
                        // Disable channel
                        tim.ccer().modify(|_, w| w.$ENBIT().clear_bit());
                        // Set PWM mode and enable preload
                        tim.$CCMR().modify(|_, w| w.$MODE().pwm_mode1().$POLBIT().set_bit());
                        // Enable channel
                        tim.ccer().modify(|_, w| w.$ENBIT().set_bit());
                    }
                }
            )+
        )+
    };
}

pwm!(TIM3, [
        Channel1 => ccr1, cc1e, ccmr1_output, oc1m, oc1pe, [(PB4, 1),],
        Channel2 => ccr2, cc2e, ccmr1_output, oc2m, oc2pe, [(PB5, 1),],
        Channel3 => ccr3, cc3e, ccmr2_output, oc3m, oc3pe, [(PB0, 1),],
        Channel4 => ccr4, cc4e, ccmr2_output, oc4m, oc4pe, [(PB1, 1),],
    ]
);
