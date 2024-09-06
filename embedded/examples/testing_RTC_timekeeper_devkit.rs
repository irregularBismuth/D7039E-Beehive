#![no_main]
#![no_std]
// #![deny(unsafe_code)]
// #![deny(warnings)]
mod timekeeper;
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
    nrf52840_hal as hal, panic_rtt_target as _,
    rtt_target::{rprintln, rtt_init_print},
    systick_monotonic::*,
};
use crate::timekeeper::{DateTime, DAYS_IN_MONTHS};

const TIMER_HZ: u32 = 4; // 4 Hz (250 ms granularity)
const TIME_0: Instant = TimerInstantU64::from_ticks(0); // Constant for time zero

type Led = Pin<Output<PushPull>>;
type Instant = TimerInstantU64<TIMER_HZ>;
type Duration = MillisDurationU64;


#[rtic::app(device = nrf52840_hal::pac, dispatchers= [TIMER0])]
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
        date: DateTime,
        rtc: Rtc<nrf52840_hal::pac::RTC0>,
    }

    #[local]
    struct Local {
        //led1: Led,
        
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("\n---init---\n");
        

        /************************** SECTION: Ports and time *************************/
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);
        let mut mono = Systick::new(cx.core.SYST, 64_000_000);
        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        
        /******************** SECTION: Buttons, buzzer and led setup ***************/
        let buzzer = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let led1 = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        /****************************************************************************/
    
        /************************** SECTION: RTC setup *****************************/

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
        /***************************************************************************/

        /************************** SECTION: Set time *****************************/
        
        // year, month, day, hour, minute, second
        let date = DateTime::new(2024, 3, 20, 11, 02, 0);

        /***************************************************************************/


        (Shared {led1, date, rtc}, Local { }, init::Monotonics(mono))
    }
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop{
            //rprintln!("LOOPIN");
            asm::wfi();
        }
    }
       
   
    #[task(binds = RTC0, local =[], shared = [led1, date, rtc])]
    fn keeping_time(mut cx: keeping_time::Context) {
        rprintln!("I AM THE TIMEKEEPER");
        cx.shared.rtc.lock(|rtc|{
            cx.shared.date.lock(|date| {
                date.update();
                let daytum = date.get_date();
                let time = date.get_time();
                rprintln!("Date: {}-{}-{}", daytum.0, daytum.1, daytum.2);
                rprintln!("Time: {}:{}:{}", time.0, time.1, time.2);
            rtc.reset_event(RtcInterrupt::Compare0);
            rtc.clear_counter();
            
    })
    });

}
}
