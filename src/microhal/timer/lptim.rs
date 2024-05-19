#![allow(unused)]
use core::marker::PhantomData;

use crate::microhal::rcc::lptim::{LptimClock, LptimClockExt};
use crate::microhal::rcc::{RccControl, ResetEnable};

use stm32g0::stm32g071::{LPTIM1, LPTIM2};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LptimPrescaler {
    Div1 = 0b000,
    Div2 = 0b001,
    Div4 = 0b010,
    Div8 = 0b011,
    Div16 = 0b100,
    Div32 = 0b101,
    Div64 = 0b110,
    Div128 = 0b111,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LptimEvent {
    DirectionDown,
    DirectionUp,
    ArrOk,
    CmpOk,
    ExtTrig,
    ArrMatch,
    CmpMatch,
}

#[derive(Debug)]
pub struct LowPowerTimer<TIM> {
    timer: TIM,
}

/// Type tag for enabled timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Enabled;

/// Type tag for disabled timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disabled;

/// Low power timer in counting mode
#[derive(Debug)]
pub struct LptimCounter<TIM, STATE> {
    timer: TIM,
    state: PhantomData<STATE>,
}

macro_rules! low_power_timer {
    ($TIM:ident) => {
        impl LowPowerTimer<$TIM> {
            pub fn new(timer: $TIM) -> Self {
                Self { timer }
            }

            pub fn upcounter(
                self,
                clock: LptimClock,
                prescaler: LptimPrescaler,
                limit: u16,
                rcc: &RccControl,
            ) -> LptimCounter<$TIM, Enabled> {
                // Configure timer clock.
                $TIM::set_clock(clock, rcc);
                $TIM::enable(rcc);
                $TIM::reset(rcc);
                self.timer
                    .cfgr
                    .modify(|_, w| w.presc().variant(prescaler as u8));

                self.timer.cr.modify(|_, w| w.enable().set_bit());

                // "After setting the ENABLE bit, a delay of two counter clock is needed before the LPTIM is
                // actually enabled."
                // The slowest LPTIM clock source is LSI at 32000 Hz, the fastest CPU clock is ~64 MHz. At
                // these conditions, one cycle of the LPTIM clock takes about 2000 CPU cycles.
                cortex_m::asm::delay(5000);

                // ARR can only be changed while the timer is *en*abled.
                self.timer.arr.write(|w| w.arr().variant(limit));
                while self.timer.isr.read().arrok().bit_is_clear() {}
                self.timer.icr.write(|w| w.arrokcf().set_bit());

                LptimCounter {
                    timer: self.timer,
                    state: PhantomData,
                }
            }
        }

        impl LptimCounter<$TIM, Enabled> {
            /// Disables timer.
            pub fn disable(self) -> LptimCounter<$TIM, Disabled> {
                self.timer.cr.modify(|_, w| w.enable().clear_bit());

                LptimCounter {
                    timer: self.timer,
                    state: PhantomData,
                }
            }

            /// Starts counting.
            pub fn start(&self) {
                self.timer.cr.modify(|_, w| w.cntstrt().set_bit());
            }

            /// Stops counting.
            pub fn stop(&self) {
                self.timer.cr.modify(|_, w| w.cntstrt().clear_bit());
            }

            /// Returns current counter value.
            pub fn counter(&self) -> u16 {
                self.timer.cnt.read().cnt().bits()
            }

            /// Returns compare value.
            pub fn cmp(&self) -> u16 {
                self.timer.cmp.read().cmp().bits()
            }

            /// Writes to CMP and waits for CMPOK to be set again to make sure there is
            /// no races later.
            pub fn set_cmp(&self, value: u16) {
                debug_assert!(self.timer.isr.read().cmpok().bit_is_clear());

                self.timer.cmp.write(|w| w.cmp().variant(value));
                while self.timer.isr.read().cmpok().bit_is_clear() {}
                self.timer.icr.write(|w| w.cmpokcf().set_bit());
                while self.timer.isr.read().cmpok().bit_is_set() {}
            }

            /// Checks if event is pending.
            pub fn is_pending(&self, event: LptimEvent) -> bool {
                let v = self.timer.isr.read();
                match event {
                    LptimEvent::DirectionDown => v.down().bit_is_set(),
                    LptimEvent::DirectionUp => v.up().bit_is_set(),
                    LptimEvent::ArrOk => v.arrok().bit_is_set(),
                    LptimEvent::CmpOk => v.cmpok().bit_is_set(),
                    LptimEvent::ExtTrig => v.exttrig().bit_is_set(),
                    LptimEvent::ArrMatch => v.arrm().bit_is_set(),
                    LptimEvent::CmpMatch => v.cmpm().bit_is_set(),
                }
            }

            /// Clears pending event.
            pub fn unpend(&self, event: LptimEvent) {
                self.timer.icr.write(|w| match event {
                    LptimEvent::DirectionDown => w.downcf().set_bit(),
                    LptimEvent::DirectionUp => w.upcf().set_bit(),
                    LptimEvent::ArrOk => w.arrokcf().set_bit(),
                    LptimEvent::CmpOk => w.cmpokcf().set_bit(),
                    LptimEvent::ExtTrig => w.exttrigcf().set_bit(),
                    LptimEvent::ArrMatch => w.arrmcf().set_bit(),
                    LptimEvent::CmpMatch => w.cmpmcf().set_bit(),
                })
            }
        }

        impl LptimCounter<$TIM, Disabled> {
            pub fn enable(self) -> LptimCounter<$TIM, Enabled> {
                self.timer.cr.modify(|_, w| w.enable().set_bit());

                // "After setting the ENABLE bit, a delay of two counter clock is needed before the LPTIM is
                // actually enabled."
                // The slowest LPTIM clock source is LSI at 32000 Hz, the fastest CPU clock is ~64 MHz. At
                // these conditions, one cycle of the LPTIM clock takes about 2000 CPU cycles.
                cortex_m::asm::delay(5000);

                LptimCounter {
                    timer: self.timer,
                    state: PhantomData,
                }
            }

            pub fn listen(&self, event: LptimEvent) {
                self.timer.ier.modify(|_, w| match event {
                    LptimEvent::DirectionDown => w.downie().set_bit(),
                    LptimEvent::DirectionUp => w.upie().set_bit(),
                    LptimEvent::ArrOk => w.arrokie().set_bit(),
                    LptimEvent::CmpOk => w.cmpokie().set_bit(),
                    LptimEvent::ExtTrig => w.exttrigie().set_bit(),
                    LptimEvent::ArrMatch => w.arrmie().set_bit(),
                    LptimEvent::CmpMatch => w.cmpmie().set_bit(),
                })
            }

            pub fn unlisten(&self, event: LptimEvent) {
                self.timer.ier.modify(|_, w| match event {
                    LptimEvent::DirectionDown => w.downie().clear_bit(),
                    LptimEvent::DirectionUp => w.upie().clear_bit(),
                    LptimEvent::ArrOk => w.arrokie().clear_bit(),
                    LptimEvent::CmpOk => w.cmpokie().clear_bit(),
                    LptimEvent::ExtTrig => w.exttrigie().clear_bit(),
                    LptimEvent::ArrMatch => w.arrmie().clear_bit(),
                    LptimEvent::CmpMatch => w.cmpmie().clear_bit(),
                })
            }
        }
    };
}

low_power_timer!(LPTIM1);
low_power_timer!(LPTIM2);
