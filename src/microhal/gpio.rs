#![allow(unused)]
use core::marker::PhantomData;

use super::pac::{EXTI, GPIOA, GPIOB};
use super::rcc::{Rcc, ResetEnable};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, rcc: &Rcc) -> Self::Parts;
}

/// Trigger edge
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SignalEdge {
    Rising,
    Falling,
    Both,
}

/// Input mode
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input
pub struct Floating;

/// Pulled down input
pub struct PullDown;

/// Pulled up input
pub struct PullUp;

/// Output mode
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Open drain input or output
pub struct OpenDrain;

/// Push pull output
pub struct PushPull;

/// Analog mode
pub struct Analog;

/// Alternate function mode
pub struct Alternate<const N: u8>;

pub(crate) enum AltFunction {
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
    // AF8 = 8,
    // AF9 = 9,
    // AF10 = 10,
}

macro_rules! gpio_af {
    ($ENUM:ty) => {
        impl From<AltFunction> for $ENUM {
            fn from(af: AltFunction) -> Self {
                match af {
                    AltFunction::AF0 => <$ENUM>::Af0,
                    AltFunction::AF1 => <$ENUM>::Af1,
                    AltFunction::AF2 => <$ENUM>::Af2,
                    AltFunction::AF3 => <$ENUM>::Af3,
                    AltFunction::AF4 => <$ENUM>::Af4,
                    AltFunction::AF5 => <$ENUM>::Af5,
                    AltFunction::AF6 => <$ENUM>::Af6,
                    AltFunction::AF7 => <$ENUM>::Af7,
                    // AltFunction::AF8 => <$ENUM>::Af8,
                    // AltFunction::AF9 => <$ENUM>::Af9,
                    // AltFunction::AF10 => <$ENUM>::Af10,
                }
            }
        }
    };
}

gpio_af!(crate::microhal::pac::gpioa::afrl::AFSEL0);
gpio_af!(crate::microhal::pac::gpioa::afrh::AFSEL8);
gpio_af!(crate::microhal::pac::gpiob::afrl::AFSEL0);
gpio_af!(crate::microhal::pac::gpiob::afrh::AFSEL8);

mod marker {
    // Marker trait for allowed alternate function values
    pub trait AF {
        const AF_A_L: crate::microhal::pac::gpioa::afrl::AFSEL0;
        const AF_A_H: crate::microhal::pac::gpioa::afrh::AFSEL8;
        const AF_B_L: crate::microhal::pac::gpiob::afrl::AFSEL0;
        const AF_B_H: crate::microhal::pac::gpiob::afrh::AFSEL8;
    }

    macro_rules! impl_af {
        ($N:literal, $AF:ident) => {
            impl AF for super::Alternate<$N> {
                const AF_A_L: crate::microhal::pac::gpioa::afrl::AFSEL0 =
                    crate::microhal::pac::gpioa::afrl::AFSEL0::$AF;
                const AF_A_H: crate::microhal::pac::gpioa::afrh::AFSEL8 =
                    crate::microhal::pac::gpioa::afrh::AFSEL8::$AF;
                const AF_B_L: crate::microhal::pac::gpiob::afrl::AFSEL0 =
                    crate::microhal::pac::gpiob::afrl::AFSEL0::$AF;
                const AF_B_H: crate::microhal::pac::gpiob::afrh::AFSEL8 =
                    crate::microhal::pac::gpiob::afrh::AFSEL8::$AF;
            }
        };
    }

    impl_af!(0, Af0);
    impl_af!(1, Af1);

    // Marker trait for modes able to generate interrupts
    pub trait Interruptable {}

    impl<MODE> Interruptable for super::Input<MODE> {}
    impl<MODE> Interruptable for super::Output<MODE> {}
}

macro_rules! gpio {
    ($GPIO:ident, $gpio:ident, $muxport:ident, [$($PXi:ident: ($pxi:ident, $default_mode:ty,
            $idrbit:ident, $odrbit:ident, $bsbit:ident, $brbit:ident, $moderbit:ident,
            $pupdrbit:ident, $otbit:ident, $afreg:ident, $afbit:ident, $afval:ident,
            $muxreg:ident, $muxbits:ident, $rtbit:ident, $ftbit:ident),)+]) => {
        pub mod $gpio {
            use super::{GpioExt, ResetEnable, Rcc, EXTI, $GPIO};
            use super::{Output, PushPull, OpenDrain};
            use super::{Input, Floating, PullUp, PullDown};
            use super::{Analog, Alternate, AltFunction};
            use super::SignalEdge;
            use super::marker::{AF, Interruptable};

            use core::marker::PhantomData;

            impl GpioExt for $GPIO {
                type Parts = Parts;

                fn split(self, rcc: &Rcc) -> Self::Parts {
                    $GPIO::enable(rcc);
                    $GPIO::reset(rcc);

                    Self::Parts{
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            pub struct Parts {
                $(
                    pub $pxi: $PXi<$default_mode>,
                )+
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                #[allow(unsafe_code)]
                impl<MODE> $PXi<MODE> {
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().floating());
                        rb.otyper().modify(|_, w| w.$otbit().push_pull());
                        rb.moder().modify(|_, w| w.$moderbit().output());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().floating());
                        rb.otyper().modify(|_, w| w.$otbit().open_drain());
                        rb.moder().modify(|_, w| w.$moderbit().output());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().floating());
                        rb.moder().modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pulldown_input(self) -> $PXi<Input<PullDown>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().pull_down());
                        rb.moder().modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pullup_input(self) -> $PXi<Input<PullUp>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().pull_up());
                        rb.moder().modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_analog(self) -> $PXi<Analog> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr().modify(|_, w| w.$pupdrbit().floating());
                        rb.moder().modify(|_, w| w.$moderbit().analog());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_alternate_function<const N: u8>(self) -> $PXi<Alternate<N>>
                    where Alternate<N> : AF {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.$afreg().modify(|_, w| w.$afbit().variant(<Alternate<N>>::$afval));
                        rb.moder().modify(|_, w| w.$moderbit().alternate());
                        $PXi { _mode: PhantomData }
                    }

                    pub(crate) fn set_alternate_function_mode(&self, mode: AltFunction) {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.$afreg().modify(|_, w| w.$afbit().variant(mode.into()));
                        rb.moder().modify(|_, w| w.$moderbit().alternate());
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Interruptable {
                    pub fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                        exti.$muxreg().modify(|_, w| w.$muxbits().$muxport());
                    }

                    pub fn trigger_on_edge(&mut self, edge: SignalEdge, exti: &mut EXTI) {
                        match edge {
                            SignalEdge::Rising => exti.rtsr1().modify(|_, w| w.$rtbit().enabled()),
                            SignalEdge::Falling => exti.ftsr1().modify(|_, w| w.$ftbit().enabled()),
                            SignalEdge::Both => {
                                exti.rtsr1().modify(|_, w| w.$rtbit().enabled());
                                exti.ftsr1().modify(|_, w| w.$ftbit().enabled());
                            }
                        }
                    }
                }

                impl<MODE> embedded_hal::digital::ErrorType for $PXi<MODE> {
                    type Error = core::convert::Infallible;
                }

                impl<MODE> embedded_hal::digital::InputPin for $PXi<Input<MODE>> {
                    #[allow(unsafe_code)]
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        Ok(rb.idr().read().$idrbit().is_high())
                    }

                    fn is_low(&mut self) -> Result<bool, Self::Error> {
                        self.is_high().map(|v| !v)
                    }
                }

                #[allow(unsafe_code)]
                impl<MODE> embedded_hal::digital::OutputPin for $PXi<Output<MODE>> {
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        unsafe {
                            let rb =  &(*$GPIO::ptr());
                            Ok(rb.bsrr().write(|w| w.$bsbit().set_bit()))
                        }
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        unsafe {
                            let rb =  &(*$GPIO::ptr());
                            Ok(rb.bsrr().write(|w| w.$brbit().set_bit()))
                        }
                    }
                }

                impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXi<Output<MODE>> {
                    #[allow(unsafe_code)]
                    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        Ok(rb.odr().read().$odrbit().is_high())
                    }

                    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                        self.is_set_high().map(|v| !v)
                    }
                }
            )+
        }
    };
}

gpio!(GPIOA, gpioa, pa, [
    // Pin: (pin, default_mode, bits...)
    PA0:  (pa0,  Analog, idr0,  odr0,  bs0,  br0,  moder0,  pupdr0,  ot0,  afrl,  afrel0, AF_A_L, exticr1, exti0_7,   tr0,  tr0),
    PA1:  (pa1,  Analog, idr1,  odr1,  bs1,  br1,  moder1,  pupdr1,  ot1,  afrl,  afrel1, AF_A_L, exticr1, exti8_15,  tr1,  tr1),
    PA2:  (pa2,  Analog, idr2,  odr2,  bs2,  br2,  moder2,  pupdr2,  ot2,  afrl,  afrel2, AF_A_L, exticr1, exti16_23, tr2,  tr2),
    PA3:  (pa3,  Analog, idr3,  odr3,  bs3,  br3,  moder3,  pupdr3,  ot3,  afrl,  afrel3, AF_A_L, exticr1, exti24_31, tr3,  tr3),
    PA4:  (pa4,  Analog, idr4,  odr4,  bs4,  br4,  moder4,  pupdr4,  ot4,  afrl,  afrel4, AF_A_L, exticr2, exti0_7,   tr4,  tr4),
    PA5:  (pa5,  Analog, idr5,  odr5,  bs5,  br5,  moder5,  pupdr5,  ot5,  afrl,  afrel5, AF_A_L, exticr2, exti8_15,  tr5,  tr5),
    PA6:  (pa6,  Analog, idr6,  odr6,  bs6,  br6,  moder6,  pupdr6,  ot6,  afrl,  afrel6, AF_A_L, exticr2, exti16_23, tr6,  tr6),
    PA7:  (pa7,  Analog, idr7,  odr7,  bs7,  br7,  moder7,  pupdr7,  ot7,  afrl,  afrel7, AF_A_L, exticr2, exti24_31, tr7,  tr7),
    PA8:  (pa8,  Analog, idr8,  odr8,  bs8,  br8,  moder8,  pupdr8,  ot8,  afrh,  afrel8, AF_A_H, exticr3, exti0_7,   tr8,  tr8),
    PA9:  (pa9,  Analog, idr9,  odr9,  bs9,  br9,  moder9,  pupdr9,  ot9,  afrh,  afrel9, AF_A_H, exticr3, exti8_15,  tr9,  tr9),
    PA10: (pa10, Analog, idr10, odr10, bs10, br10, moder10, pupdr10, ot10, afrh, afrel10, AF_A_H, exticr3, exti16_23, tr10, tr10),
    PA11: (pa11, Analog, idr11, odr11, bs11, br11, moder11, pupdr11, ot11, afrh, afrel11, AF_A_H, exticr3, exti24_31, tr11, tr11),
    PA12: (pa12, Analog, idr12, odr12, bs12, br12, moder12, pupdr12, ot12, afrh, afrel12, AF_A_H, exticr4, exti0_7,   tr12, tr12),
    PA13: (pa13, Alternate<0>, idr13, odr13, bs13, br13, moder13, pupdr13, ot13, afrh, afrel13, AF_A_H, exticr4, exti8_15,  tr13, tr13),
    PA14: (pa14, Alternate<0>, idr14, odr14, bs14, br14, moder14, pupdr14, ot14, afrh, afrel14, AF_A_H, exticr4, exti16_23, tr14, tr14),
    PA15: (pa15, Analog, idr15, odr15, bs15, br15, moder15, pupdr15, ot15, afrh, afrel15, AF_A_H, exticr4, exti24_31, tr15, tr15),
]);

gpio!(GPIOB, gpiob, pb, [
    // Pin: (pin, default_mode, bits...)
    PB0:  (pb0,  Analog, idr0,  odr0,  bs0,  br0,  moder0,  pupdr0,  ot0,  afrl,  afrel0, AF_B_L, exticr1, exti0_7,   tr0,  tr0),
    PB1:  (pb1,  Analog, idr1,  odr1,  bs1,  br1,  moder1,  pupdr1,  ot1,  afrl,  afrel1, AF_B_L, exticr1, exti8_15,  tr1,  tr1),
    PB2:  (pb2,  Analog, idr2,  odr2,  bs2,  br2,  moder2,  pupdr2,  ot2,  afrl,  afrel2, AF_B_L, exticr1, exti16_23, tr2,  tr2),
    PB3:  (pb3,  Analog, idr3,  odr3,  bs3,  br3,  moder3,  pupdr3,  ot3,  afrl,  afrel3, AF_B_L, exticr1, exti24_31, tr3,  tr3),
    PB4:  (pb4,  Analog, idr4,  odr4,  bs4,  br4,  moder4,  pupdr4,  ot4,  afrl,  afrel4, AF_B_L, exticr2, exti0_7,   tr4,  tr4),
    PB5:  (pb5,  Analog, idr5,  odr5,  bs5,  br5,  moder5,  pupdr5,  ot5,  afrl,  afrel5, AF_B_L, exticr2, exti8_15,  tr5,  tr5),
    PB6:  (pb6,  Analog, idr6,  odr6,  bs6,  br6,  moder6,  pupdr6,  ot6,  afrl,  afrel6, AF_B_L, exticr2, exti16_23, tr6,  tr6),
    PB7:  (pb7,  Analog, idr7,  odr7,  bs7,  br7,  moder7,  pupdr7,  ot7,  afrl,  afrel7, AF_B_L, exticr2, exti24_31, tr7,  tr7),
    PB8:  (pb8,  Analog, idr8,  odr8,  bs8,  br8,  moder8,  pupdr8,  ot8,  afrh,  afrel8, AF_B_H, exticr3, exti0_7,   tr8,  tr8),
    PB9:  (pb9,  Analog, idr9,  odr9,  bs9,  br9,  moder9,  pupdr9,  ot9,  afrh,  afrel9, AF_B_H, exticr3, exti8_15,  tr9,  tr9),
    PB10: (pb10, Analog, idr10, odr10, bs10, br10, moder10, pupdr10, ot10, afrh, afrel10, AF_B_H, exticr3, exti16_23, tr10, tr10),
    PB11: (pb11, Analog, idr11, odr11, bs11, br11, moder11, pupdr11, ot11, afrh, afrel11, AF_B_H, exticr3, exti24_31, tr11, tr11),
    PB12: (pb12, Analog, idr12, odr12, bs12, br12, moder12, pupdr12, ot12, afrh, afrel12, AF_B_H, exticr4, exti0_7,   tr12, tr12),
    PB13: (pb13, Analog, idr13, odr13, bs13, br13, moder13, pupdr13, ot13, afrh, afrel13, AF_B_H, exticr4, exti8_15,  tr13, tr13),
    PB14: (pb14, Analog, idr14, odr14, bs14, br14, moder14, pupdr14, ot14, afrh, afrel14, AF_B_H, exticr4, exti16_23, tr14, tr14),
    PB15: (pb15, Analog, idr15, odr15, bs15, br15, moder15, pupdr15, ot15, afrh, afrel15, AF_B_H, exticr4, exti24_31, tr15, tr15),
]);
