#![no_std]
#![no_main]

use cortex_m_rt::entry;
use rtt_target::rtt_init_print;
use rtt_target::debug_rprintln;
use stm32l0xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    loop {
      debug_rprintln!("loop");
    }
}
