use stm32g0::stm32g071::rcc::{AHBENR, AHBRSTR, APBENR1, APBENR2, APBRSTR1, APBRSTR2};
use stm32g0::stm32g071::{ADC, DMA, LPTIM1, LPTIM2, TIM1, TIM2, TIM3};

use super::{RccControl, ResetEnable};

macro_rules! reset_enable_bus {
    ($bus:ident, $enable_register:ident, $enable_type:ty, $reset_register:ident, $reset_type:ty) => {
        #[derive(Debug)]
        struct $bus;

        impl $bus {
            #[inline(always)]
            pub fn enable_register(rcc: &RccControl) -> &$enable_type {
                &rcc.rcc.$enable_register
            }

            #[inline(always)]
            pub fn reset_register(rcc: &RccControl) -> &$reset_type {
                &rcc.rcc.$reset_register
            }
        }
    };
}

reset_enable_bus!(Ahb, ahbenr, AHBENR, ahbrstr, AHBRSTR);
reset_enable_bus!(Apb1, apbenr1, APBENR1, apbrstr1, APBRSTR1);
reset_enable_bus!(Apb2, apbenr2, APBENR2, apbrstr2, APBRSTR2);

macro_rules! reset_enable {
    ($dev:ident, $bus:ident, $enable:ident, $reset:ident) => {
        impl ResetEnable for $dev {
            #[inline(always)]
            fn enable(rcc: &RccControl) {
                $bus::enable_register(rcc).modify(|_, w| w.$enable().set_bit());
            }

            #[inline(always)]
            fn disable(rcc: &RccControl) {
                $bus::enable_register(rcc).modify(|_, w| w.$enable().clear_bit());
            }

            #[inline(always)]
            fn reset(rcc: &RccControl) {
                $bus::reset_register(rcc).modify(|_, w| w.$reset().set_bit());
                $bus::reset_register(rcc).modify(|_, w| w.$reset().clear_bit());
            }
        }
    };
}

// AHB devices
reset_enable!(DMA, Ahb, dmaen, dmarst); // 0

// APB1 devices
reset_enable!(TIM2, Apb1, tim2en, tim2rst); // 0
reset_enable!(TIM3, Apb1, tim3en, tim3rst); // 1
reset_enable!(LPTIM2, Apb1, lptim2en, lptim2rst); // 30
reset_enable!(LPTIM1, Apb1, lptim1en, lptim1rst); // 31

// APB2 devices
reset_enable!(TIM1, Apb2, tim1en, tim1rst); // 11
reset_enable!(ADC, Apb2, adcen, adcrst); // 20
