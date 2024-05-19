use stm32g0::stm32g071::rcc::{AHBENR, AHBRSTR, APBENR1, APBENR2, APBRSTR1, APBRSTR2};
use stm32g0::stm32g071::{ADC, DMA, LPTIM1, LPTIM2, TIM1, TIM2, TIM3};

use super::{RccControl, ResetEnable};

macro_rules! reset_enable_bus {
    ($bus:ident, $enable_register:ident, $enable_type:ty, $reset_register:ident, $reset_type:ty) => {
        #[allow(clippy::upper_case_acronyms)]
        #[derive(Debug)]
        struct $bus;

        impl $bus {
            pub fn enr(rcc: &RccControl) -> &$enable_type {
                &rcc.rcc.$enable_register
            }

            pub fn rstr(rcc: &RccControl) -> &$reset_type {
                &rcc.rcc.$reset_register
            }
        }
    };
}

reset_enable_bus!(AHB, ahbenr, AHBENR, ahbrstr, AHBRSTR);
reset_enable_bus!(APB1, apbenr1, APBENR1, apbrstr1, APBRSTR1);
reset_enable_bus!(APB2, apbenr2, APBENR2, apbrstr2, APBRSTR2);

macro_rules! reset_enable {
    ($dev:ident, $bus:ident, $enable:ident, $reset:ident) => {
        impl ResetEnable for $dev {
            fn enable(rcc: &RccControl) {
                $bus::enr(rcc).modify(|_, w| w.$enable().set_bit());
            }

            fn disable(rcc: &RccControl) {
                $bus::enr(rcc).modify(|_, w| w.$enable().clear_bit());
            }

            fn reset(rcc: &RccControl) {
                $bus::rstr(rcc).modify(|_, w| w.$reset().set_bit());
                $bus::rstr(rcc).modify(|_, w| w.$reset().clear_bit());
            }
        }
    };
}

// AHB devices
reset_enable!(DMA, AHB, dmaen, dmarst); // 0

// APB1 devices
reset_enable!(TIM2, APB1, tim2en, tim2rst); // 0
reset_enable!(TIM3, APB1, tim3en, tim3rst); // 1
reset_enable!(LPTIM2, APB1, lptim2en, lptim2rst); // 30
reset_enable!(LPTIM1, APB1, lptim1en, lptim1rst); // 31

// APB2 devices
reset_enable!(TIM1, APB2, tim1en, tim1rst); // 11
reset_enable!(ADC, APB2, adcen, adcrst); // 20
