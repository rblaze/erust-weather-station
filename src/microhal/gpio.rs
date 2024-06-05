#![allow(unused)]
use core::marker::PhantomData;

use stm32g0::stm32g071::GPIOB;

use super::rcc::{RccControl, ResetEnable};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, rcc: &RccControl) -> Self::Parts;
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

macro_rules! gpio {
    ($GPIO:ident, $gpio:ident, [$($PXi:ident: ($pxi:ident, $default_mode:ident,
            $idrbit:ident, $odrbit:ident, $bsbit:ident, $brbit:ident, $moderbit:ident,
            $pupdrbit:ident, $otbit:ident, $afreg:ident, $afbit:ident),)+]) => {
        pub mod $gpio {
            use super::{GpioExt, ResetEnable, RccControl, $GPIO};
            use super::{Output, PushPull, OpenDrain};
            use super::{Input, Floating, PullUp, PullDown};
            use super::{Analog, Alternate};

            use core::marker::PhantomData;

            mod marker {
                // Marker trait for allowed alternate function values
                pub trait AF {}

                impl AF for super::Alternate<0> {}
                impl AF for super::Alternate<1> {}
                impl AF for super::Alternate<2> {}
                impl AF for super::Alternate<3> {}
                impl AF for super::Alternate<4> {}
                impl AF for super::Alternate<5> {}
                impl AF for super::Alternate<6> {}
                impl AF for super::Alternate<7> {}
            }

            impl GpioExt for $GPIO {
                type Parts = Parts;

                fn split(self, rcc: &RccControl) -> Self::Parts {
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
                        rb.pupdr.modify(|_, w| w.$pupdrbit().floating());
                        rb.otyper.modify(|_, w| w.$otbit().push_pull());
                        rb.moder.modify(|_, w| w.$moderbit().output());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr.modify(|_, w| w.$pupdrbit().floating());
                        rb.otyper.modify(|_, w| w.$otbit().open_drain());
                        rb.moder.modify(|_, w| w.$moderbit().output());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr.modify(|_, w| w.$pupdrbit().floating());
                        rb.moder.modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pulldown_input(self) -> $PXi<Input<PullDown>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr.modify(|_, w| w.$pupdrbit().pull_down());
                        rb.moder.modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pullup_input(self) -> $PXi<Input<PullUp>> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr.modify(|_, w| w.$pupdrbit().pull_up());
                        rb.moder.modify(|_, w| w.$moderbit().input());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_analog(self) -> $PXi<Analog> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.pupdr.modify(|_, w| w.$pupdrbit().floating());
                        rb.moder.modify(|_, w| w.$moderbit().analog());
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_alternate_function<const N: u8>(self) -> $PXi<Alternate<N>>
                    where Alternate<N> : marker::AF {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        rb.$afreg.modify(|_, w| unsafe { w.$afbit().bits(N) });
                        rb.moder.modify(|_, w| w.$moderbit().alternate());
                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> embedded_hal::digital::ErrorType for $PXi<MODE> {
                    type Error = core::convert::Infallible;
                }

                impl<MODE> embedded_hal::digital::InputPin for $PXi<Input<MODE>> {
                    #[allow(unsafe_code)]
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        Ok(rb.idr.read().$idrbit().is_high())
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
                            Ok(rb.bsrr.write(|w| w.$bsbit().set_bit()))
                        }
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        unsafe {
                            let rb =  &(*$GPIO::ptr());
                            Ok(rb.bsrr.write(|w| w.$brbit().set_bit()))
                        }
                    }
                }

                impl<MODE> embedded_hal::digital::StatefulOutputPin for $PXi<Output<MODE>> {
                    #[allow(unsafe_code)]
                    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                        let rb = unsafe { &(*$GPIO::ptr()) };
                        Ok(rb.odr.read().$odrbit().is_high())
                    }

                    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                        self.is_set_high().map(|v| !v)
                    }
                }
            )+
        }
    };
}

gpio!(GPIOB, gpiob, [
    // Pin: (pin, default_mode, bits...)
    PB0:  (pb0,  Analog, idr0,  odr0,  bs0,  br0,  moder0,  pupdr0,  ot0,  afrl,  afsel0 ),
    PB1:  (pb1,  Analog, idr1,  odr1,  bs1,  br1,  moder1,  pupdr1,  ot1,  afrl,  afsel1 ),
    PB2:  (pb2,  Analog, idr2,  odr2,  bs2,  br2,  moder2,  pupdr2,  ot2,  afrl,  afsel2 ),
    PB3:  (pb3,  Analog, idr3,  odr3,  bs3,  br3,  moder3,  pupdr3,  ot3,  afrl,  afsel3 ),
    PB4:  (pb4,  Analog, idr4,  odr4,  bs4,  br4,  moder4,  pupdr4,  ot4,  afrl,  afsel4 ),
    PB5:  (pb5,  Analog, idr5,  odr5,  bs5,  br5,  moder5,  pupdr5,  ot5,  afrl,  afsel5 ),
    PB6:  (pb6,  Analog, idr6,  odr6,  bs6,  br6,  moder6,  pupdr6,  ot6,  afrl,  afsel6 ),
    PB7:  (pb7,  Analog, idr7,  odr7,  bs7,  br7,  moder7,  pupdr7,  ot7,  afrl,  afsel7 ),
    PB8:  (pb8,  Analog, idr8,  odr8,  bs8,  br8,  moder8,  pupdr8,  ot8,  afrh,  afsel8 ),
    PB9:  (pb9,  Analog, idr9,  odr9,  bs9,  br9,  moder9,  pupdr9,  ot9,  afrh,  afsel9 ),
    PB10: (pb10, Analog, idr10, odr10, bs10, br10, moder10, pupdr10, ot10, afrh, afsel10),
    PB11: (pb11, Analog, idr11, odr11, bs11, br11, moder11, pupdr11, ot11, afrh, afsel11),
    PB12: (pb12, Analog, idr12, odr12, bs12, br12, moder12, pupdr12, ot12, afrh, afsel12),
    PB13: (pb13, Analog, idr13, odr13, bs13, br13, moder13, pupdr13, ot13, afrh, afsel13),
    PB14: (pb14, Analog, idr14, odr14, bs14, br14, moder14, pupdr14, ot14, afrh, afsel14),
    PB15: (pb15, Analog, idr15, odr15, bs15, br15, moder15, pupdr15, ot15, afrh, afsel15),
]);
