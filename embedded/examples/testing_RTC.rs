#![no_main]
#![no_std]
// #![deny(unsafe_code)]
// #![deny(warnings)]

use {
    cortex_m::asm,
    embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin},
    fugit::{MillisDurationU64, TimerInstantU64},
    hal::{
        gpio::{Level, Output, Pin, PullDown, PushPull, Input},
        gpiote::*,
        // ppi::{self, ConfigurablePpi, Ppi},
        prelude::*,
        saadc::*,
        clocks::*,
        pac::interrupt,
        rtc::{Rtc, RtcCompareReg, RtcInterrupt}, 
    },
    nrf52833_hal as hal, panic_rtt_target as _,
    rtt_target::{rprintln, rtt_init_print},
    systick_monotonic::*,
};

const TIMER_HZ: u32 = 4; // 4 Hz (250 ms granularity)
const TIME_0: Instant = TimerInstantU64::from_ticks(0); // Constant for time zero

type Led = Pin<Output<PushPull>>;
type Instant = TimerInstantU64<TIMER_HZ>;
type Duration = MillisDurationU64;

#[rtic::app(device = nrf52833_hal::pac, dispatchers= [TIMER0])]
mod app {
    use core::borrow::Borrow;

    use hal::{comp::CompInputPin, gpio::{self, p0::P0_31, Disconnected, Floating}, pac::RTC0};

    use super::*;
    //#[monotonic(binds = RTC0, default = true)]
    //type Mono = hal::monotonic::MonotonicTimer<hal::pac::RTC0, 32_768u32>;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<TIMER_HZ>;

    #[shared]
    struct Shared {
        led1: Led,

    }

    #[local]
    struct Local {
        //led1: Led,
        rtc: Rtc<nrf52833_hal::pac::RTC0>,

    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("\n---init---\n");
        

        /************************** SECTION: Ports and time *************************/
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);
        let mut mono = Systick::new(cx.core.SYST, 64_000_000);
        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        /****************************************************************************/
        
        /******************** SECTION: Buttons, buzzer and led setup ***************/
        let buzzer = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let led1 = port0.p0_11.into_push_pull_output(Level::Low).degrade();
      
    
        /************************** SECTION: RTC setup *****************************/


        //let mut cp = hal::pac::CorePeripherals::take().unwrap();
       
        // Enable interrupt
        //cx.device.RTC0.intenset.write(|w| w.compare0().set());
        // Clock
        rprintln!("Before clock...");
        let clocks = hal::clocks::Clocks::new(cx.device.CLOCK);
        rprintln!("After first...");
        //let clocks = clocks.set_lfclk_src_external(hal::clocks::LfOscConfiguration::ExternalAndBypass);
        rprintln!("After second...");
        //let clocks = clocks.enable_ext_hfosc();
        rprintln!("After third...");
        let _ = clocks.start_lfclk();
        rprintln!("After start...");
        // Run RTC for 1 second (1hz == LFCLK_FREQ)
        
        let mut rtc = Rtc::new(cx.device.RTC0, 0).unwrap();

        rtc.set_compare(RtcCompareReg::Compare0, hal::clocks::LFCLK_FREQ).unwrap();
        rtc.enable_event(RtcInterrupt::Compare0);
        rtc.enable_interrupt(RtcInterrupt::Compare0, Some(&mut cx.core.NVIC));
        
        rprintln!("Starting RTC");
        rtc.enable_counter();

        //let mono_rtc = Mono::new(cx.device.RTC0, &clocks).unwrap();
        

        /***************************************************************************/


        (Shared {led1}, Local {rtc }, init::Monotonics(mono))
    }
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop{
            //rprintln!("LOOPIN");
            asm::wfi();
        }
    }
       
    #[task(binds = RTC0, local =[rtc], shared = [led1])]
    fn rtc(mut cx: rtc::Context) {
        let rtc = cx.local.rtc;
        rprintln!("YEEET!");
        //unsafe { (*hal::pac::RTC0::ptr()).events_compare[0].write(|w| w.bits(0)); }
        // Toggle the LED
        cx.shared.led1.lock(|led1|{
        if led1.is_set_high().unwrap() {
            led1.set_low().ok();
        } else {
            led1.set_high().ok();
        }
        });
        
        rtc.reset_event(RtcInterrupt::Compare0);
        rtc.clear_counter();
    }

}
