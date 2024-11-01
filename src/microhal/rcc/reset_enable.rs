use stm32g0::stm32g071::rcc::{
    AHBENR, AHBRSTR, APBENR1, APBENR2, APBRSTR1, APBRSTR2, IOPENR, IOPRSTR,
};
use stm32g0::stm32g071::{
    ADC, DMA, GPIOA, GPIOB, GPIOC, GPIOD, GPIOF, I2C1, I2C2, LPTIM1, LPTIM2, TIM1, TIM2, TIM3,
};

use super::{Rcc, ResetEnable};

macro_rules! reset_enable_bus {
    ($bus:ident, $enable_register:ident, $enable_type:ty, $reset_register:ident, $reset_type:ty) => {
        #[allow(clippy::upper_case_acronyms)]
        #[derive(Debug)]
        struct $bus;

        impl $bus {
            pub fn enr(rcc: &Rcc) -> &$enable_type {
                &rcc.rcc.$enable_register
            }

            pub fn rstr(rcc: &Rcc) -> &$reset_type {
                &rcc.rcc.$reset_register
            }
        }
    };
}

reset_enable_bus!(AHB, ahbenr, AHBENR, ahbrstr, AHBRSTR);
reset_enable_bus!(APB1, apbenr1, APBENR1, apbrstr1, APBRSTR1);
reset_enable_bus!(APB2, apbenr2, APBENR2, apbrstr2, APBRSTR2);
reset_enable_bus!(GPIO, iopenr, IOPENR, ioprstr, IOPRSTR);

macro_rules! reset_enable {
    ($dev:ident, $bus:ident, $enable:ident, $reset:ident) => {
        impl ResetEnable for $dev {
            fn enable(rcc: &Rcc) {
                $bus::enr(rcc).modify(|_, w| w.$enable().set_bit());
            }

            fn disable(rcc: &Rcc) {
                $bus::enr(rcc).modify(|_, w| w.$enable().clear_bit());
            }

            fn reset(rcc: &Rcc) {
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
reset_enable!(I2C1, APB1, i2c1en, i2c1rst); // 21
reset_enable!(I2C2, APB1, i2c2en, i2c2rst); // 22
reset_enable!(LPTIM2, APB1, lptim2en, lptim2rst); // 30
reset_enable!(LPTIM1, APB1, lptim1en, lptim1rst); // 31

// APB2 devices
reset_enable!(TIM1, APB2, tim1en, tim1rst); // 11
reset_enable!(ADC, APB2, adcen, adcrst); // 20

// GPIO devices
reset_enable!(GPIOA, GPIO, iopaen, ioparst); // 0
reset_enable!(GPIOB, GPIO, iopben, iopbrst); // 1
reset_enable!(GPIOC, GPIO, iopcen, iopcrst); // 2
reset_enable!(GPIOD, GPIO, iopden, iopdrst); // 3
reset_enable!(GPIOF, GPIO, iopfen, iopfrst); // 5
