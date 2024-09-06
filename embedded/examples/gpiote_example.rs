#![no_std]
#![no_main]

use {
    core::panic::PanicInfo, 
    nrf52833_hal as hal,
    hal::gpio::{Input, Pin, PullDown},
    rtt_target::rprintln
};

const TIMER_HZ: u32 = 4; // 4 Hz (250 ms granularity)

type Button = Pin<hal::gpio::Input<hal::gpio::PullDown>>;

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0])]
mod app {
    use super::*;
    use hal::gpio;
    use systick_monotonic::*;
    use {
        embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin},
        hal::{
            gpio::{Input, Level, Pin, PullUp},
            gpiote::*,
            ppi::{self, ConfigurablePpi, Ppi},
        },
        crate::Button,
        nrf52833_hal as hal,
        rtt_target::{rprintln, rtt_init_print},
    };

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<TIMER_HZ>;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        btn1: Button,
        btn2: Button,
        btn3: Button,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let p1 = hal::gpio::p1::Parts::new(ctx.device.P1);
        let buzzer = p1.p1_09.into_push_pull_output(Level::Low).degrade();
        let btn1 = p0.p0_02.into_pulldown_input().degrade();
        let btn2 = p0.p0_03.into_pulldown_input().degrade();
        let btn3 = p0.p0_28.into_pulldown_input().degrade();
        let led1 = p0.p0_11.into_push_pull_output(Level::High).degrade();

        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        // Set btn1 to generate port event
        gpiote.port().input_pin(&btn1).high();
        // Enable interrupt for port event
        gpiote.port().enable_interrupt();

        // Set btn2 to generate port event
        gpiote.port().input_pin(&btn2).high();
        // Enable interrupt for port event
        gpiote.port().enable_interrupt();

        // Set btn3 to generate port event
        gpiote.port().input_pin(&btn3).high();
        // Enable interrupt for port event
        gpiote.port().enable_interrupt();

        // Bind the led1 pin to channel 1
        gpiote.channel1().output_pin(led1).task_out_polarity(TaskOutPolarity::Toggle).init_high();
        let ppi_channels = ppi::Parts::new(ctx.device.PPI);
        let mut ppi0 = ppi_channels.ppi0;
        ppi0.set_task_endpoint(gpiote.channel1().task_out());
        ppi0.set_event_endpoint(gpiote.port().event());
        ppi0.enable();

        let mono = Systick::new(ctx.core.SYST, 64_000_000);

        (
            Shared { gpiote },
            Local { btn1, btn2, btn3 },
            init::Monotonics(mono),
        )
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn on_gpiote(mut ctx: on_gpiote::Context) {
        ctx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                rprintln!("Interrupt from channel 0 event");
            }
            if gpiote.port().is_event_triggered() {
                rprintln!("Interrupt from port event");
            }
            // Reset all events
            gpiote.reset_events();
            // Debounce
            debounce::spawn_after(50.millis()).ok();
        });
    }

    #[task(shared = [gpiote], local = [btn1, btn2, btn3])]
    fn debounce(mut ctx: debounce::Context) {
        let btn1_pressed = ctx.local.btn1.is_high().unwrap();
        let btn2_pressed = ctx.local.btn2.is_high().unwrap();
        let btn3_pressed = ctx.local.btn3.is_high().unwrap();

        ctx.shared.gpiote.lock(|gpiote| {
            if btn1_pressed {
                rprintln!("Button 1 was pressed!");
                // Manually run "task out" operation (toggle) on channel1 (toggles led1)
                gpiote.channel1().task_out();
            }
            if btn2_pressed {
                rprintln!("Button 2 was pressed!");
                // Manually run "task clear" on channel 1 (led1 on)
                gpiote.channel1().clear();
            }
            if btn3_pressed {
                rprintln!("Button 3 was pressed!");
                // Manually run "task set" on channel 1 (led1 off)
                gpiote.channel1().set();
            }
        });
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {}
}
