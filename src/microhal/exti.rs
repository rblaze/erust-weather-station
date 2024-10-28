#![allow(unsafe_code, unused)]

use stm32g0::stm32g071::EXTI;

use super::gpio::SignalEdge;

// TODO: conditionally enable events by MCU model

/// EXTI trigger event
#[derive(Debug, Eq, PartialEq, PartialOrd, Clone, Copy)]
pub enum Event {
    GPIO0 = 0,
    GPIO1 = 1,
    GPIO2 = 2,
    GPIO3 = 3,
    GPIO4 = 4,
    GPIO5 = 5,
    GPIO6 = 6,
    GPIO7 = 7,
    GPIO8 = 8,
    GPIO9 = 9,
    GPIO10 = 10,
    GPIO11 = 11,
    GPIO12 = 12,
    GPIO13 = 13,
    GPIO14 = 14,
    GPIO15 = 15,
    PVD = 16,
    COMP1 = 17,
    COMP2 = 18,
    RTC = 19,
    TAMP = 21,
    I2C1 = 23,
    USART1 = 25,
    USART2 = 26,
    CEC = 27,
    LPUART1 = 28,
    LPTIM1 = 29,
    LPTIM2 = 30,
    LSECSS = 31,
    UCPD1 = 32,
    UCPD2 = 33,
    VDDIO2 = 34,
    LPUART2 = 35,
}

/// Extension trait for EXTI
pub trait ExtiExt {
    /// Interrupt enable
    fn listen(&self, ev: Event);
    /// Interrupt disable
    fn unlisten(&self, ev: Event);
    /// Indicate if the interrupt is currently pending
    fn is_pending(&self, ev: Event, edge: SignalEdge) -> bool;
    /// Clear interrupt pending flag
    fn unpend(&self, ev: Event);
}

impl ExtiExt for EXTI {
    fn listen(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=31 => self.imr1.modify(|r, w| w.bits(r.bits() | (1 << line))),
                32..=35 => self
                    .imr2
                    .modify(|r, w| w.bits(r.bits() | (1 << (line - 32)))),
                _ => unreachable!(),
            }
        }
    }

    fn unlisten(&self, ev: Event) {
        self.unpend(ev);

        let line = ev as u8;
        unsafe {
            match line {
                0..=31 => self.imr1.modify(|r, w| w.bits(r.bits() & !(1 << line))),
                32..=35 => self
                    .imr2
                    .modify(|r, w| w.bits(r.bits() & !(1 << (line - 32)))),
                _ => unreachable!(),
            }
        }
    }

    fn is_pending(&self, ev: Event, edge: SignalEdge) -> bool {
        let line = ev as u8;

        match line {
            0..=31 => match edge {
                SignalEdge::Rising => self.rpr1.read().bits() & (1 << line) != 0,
                SignalEdge::Falling => self.fpr1.read().bits() & (1 << line) != 0,
                SignalEdge::Both => {
                    self.rpr1.read().bits() & (1 << line) != 0
                        || self.fpr1.read().bits() & (1 << line) != 0
                }
            },
            // 32..=35 => match edge {
            //     SignalEdge::Rising => self.rpr2.read().bits() & (1 << (line - 32)) != 0,
            //     SignalEdge::Falling => self.fpr2.read().bits() & (1 << (line - 32)) != 0,
            //     SignalEdge::Both => {
            //         self.rpr2.read().bits() & (1 << (line - 32)) != 0
            //             || self.fpr2.read().bits() & (1 << (line - 32)) != 0
            //     }
            // },
            _ => false,
        }
    }

    fn unpend(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=18 | 20 => {
                    self.rpr1.modify(|_, w| w.bits(1 << line));
                    self.fpr1.modify(|_, w| w.bits(1 << line));
                }
                34 => {
                    // self.rpr2.modify(|_, w| w.bits(1 << (line - 32)));
                    // self.fpr2.modify(|_, w| w.bits(1 << (line - 32)));
                }
                _ => unreachable!(),
            }
        }
    }
}
