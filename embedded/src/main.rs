#![no_main]
#![no_std]

use cortex_m::peripheral::syst::SystClkSource;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7::stm32f7x6;
use systick_monotonic::*; // PAC for STM32F7
                          // Define the frequency of the monotonic timer (e.g., 100 Hz)
const MONO_HZ: u32 = 100;

#[rtic::app(device = stm32f7::stm32f7x6)]
mod app {
    use super::*;

    // Monotonic timer using SysTick
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONO_HZ>;

    #[shared]
    struct Shared {
        // Add any shared resources here
    }

    #[local]
    struct Local {
        // Add any local resources here
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("\n---init---\n");

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, MONO_HZ);

        // Initialize peripherals (example: GPIO, LED, Timers, etc.)
        // let dp = cx.device;
        // let rcc = dp.RCC.constrain();
        // let gpioa = dp.GPIOA.split();
        // let led = gpioa.pa5.into_push_pull_output();

        // Return Shared, Local, and Monotonic timer
        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi(); // Enter low-power mode, waiting for interrupts
        }
    }
}
