#![no_main]
#![no_std]
// #![deny(unsafe_code)]
// #![deny(warnings)]

use stm32l0::stm32l0x3;
use {
    cortex_m::asm,
    panic_rtt_target as _,
    // systick_monotonic::*,
    // usb_device::{
    //     class_prelude::UsbBusAllocator,
    //     device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    // },
    // usbd_serial::{SerialPort, USB_CLASS_CDC},
    // embedded_hal::digital::v2::InputPin,
    // hal::{
    //     ppi::{self, ConfigurablePpi, Ppi},
    //     twim,
    //     Twim,
    //     prelude::*,
    //     clocks::*,
    //     gpio::{Level, Output, Input, Pin, PushPull,self, p0::P0_31, Disconnected, Floating},
    //     gpiote::*,
    //     rtc::{Rtc, RtcCompareReg, RtcInterrupt},
    //     usbd::{UsbPeripheral, Usbd},
    // },
    // fugit::{MillisDurationU64, TimerInstantU64},
    rtt_target::{rprintln, rtt_init_print},
};

const TIMER_HZ: u32 = 32_768; // 4 Hz (250 ms granularity)
const PD_CONVERSION: f32 = 0.2021; // Photodiode conversion rate. Unit mV/(value read)
                                   // type Instant = TimerInstantU64<TIMER_HZ>;
                                   // type Duration = MillisDurationU64;

enum DateTimeTuple {
    Date(u32, u8, u8),
    Time(u8, u8, u8),
}

#[rtic::app(device = stm32l0x3)]
mod app {
    use super::*;

    // #[monotonic(binds = RTC0, default = true)]
    // type MyMono = hal::monotonic::MonotonicTimer<hal::pac::RTC0, 32_768u32>;
    // #[monotonic(binds = SysTick, default = true)]
    // type MyMono = Systick<TIMER_HZ>;

    #[shared]
    struct Shared {
        //led1: Led,
        // date: DateTime,
        // rtc: Rtc<nrf52833_hal::pac::RTC0>,
        // rtc1: Rtc<nrf52833_hal::pac::RTC1>,
        // gpiote: Gpiote,
    }

    #[local]
    struct Local {
        // usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        // serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    }

    // Initialization function
    // #[init(local = [
    //     clocks: Option<Clocks<ExternalOscillator, Internal, LfOscStarted>> = None,
    //     usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
    // ])]

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("\n---init---\n");
        //
        //
        //  /************************** SECTION: Ports and time *************************/
        //  // let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        //  // let port1 = hal::gpio::p1::Parts::new(cx.device.P1);
        //  let mut mono = Systick::new(cx.core.SYST, 64_000_000);
        //
        //
        //  /******************** SECTION: Buttons, buzzer and led setup ***************/
        //  // let btn1 = port0.p0_02.into_pulldown_input().degrade();
        //  // let btn2 = port0.p0_03.into_pulldown_input().degrade();
        //  // let btn3 = port0.p0_28.into_pulldown_input().degrade();
        //  // let btn4 = port0.p0_29.into_pulldown_input().degrade();
        //  // let btn5 = port0.p0_30.into_pulldown_input().degrade();
        //
        //
        //  /************************** SECTION: RTC setup *****************************/
        //  //let clocks = hal::clocks::Clocks::new(cx.device.CLOCK);
        //  let clocks = Clocks::new(cx.device.CLOCK);
        //
        //  cx.local.clocks.replace(clocks.start_lfclk().enable_ext_hfosc());
        //
        //   /************************** SECTION: Config a compare reg to update time ****/
        //  let mut rtc = Rtc::new(cx.device.RTC0, 0).unwrap();
        //  rtc.set_compare(RtcCompareReg::Compare0, hal::clocks::LFCLK_FREQ).unwrap();
        //  rtc.enable_event(RtcInterrupt::Compare0);
        //  rtc.enable_interrupt(RtcInterrupt::Compare0, Some(&mut cx.core.NVIC));
        //  rprintln!("Starting RTC");
        //  rtc.enable_counter();
        //
        //  /************************** SECTION: Set time *****************************/
        //  // year, month, day, hour, minute, second
        //  let date = DateTime::new(2024, 10, 31, 22, 0, 0);
        //
        //  /************************** SECTION: Setup gpiote *****************************/
        //  let gpiote = Gpiote::new(cx.device.GPIOTE);
        //  // Set buttons to generate port event
        //  gpiote.port().input_pin(&btn1).high();
        //  gpiote.port().input_pin(&btn2).high();
        //  gpiote.port().input_pin(&btn3).high();
        //  gpiote.port().input_pin(&btn4).high();
        //  gpiote.port().input_pin(&btn5).high();
        //  // Enable interrupt for port event
        //  gpiote.port().enable_interrupt();
        //
        //
        //
        //  /************************** SECTION: USB shananigans ***************************/
        //  //Enable USBD interrupt
        //  let _ = cx.device.USBD.intenset.write(|w| w.sof().set());
        //
        //  let usb_bus = UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
        //      cx.device.USBD,
        //      cx.local.clocks.as_ref().unwrap(),
        //  )));
        //  cx.local.usb_bus.replace(usb_bus);
        //
        //  let serial = SerialPort::new(&cx.local.usb_bus.as_ref().unwrap());
        //
        //  let usb_dev = UsbDeviceBuilder::new(
        //      &cx.local.usb_bus.as_ref().unwrap(),
        //      UsbVidPid(0x16c0, 0x27dd),
        //  )
        //  .manufacturer("Fake taxi")
        //  .product("Serial port")
        //  .serial_number("TEST")
        //  .device_class(USB_CLASS_CDC)
        //  .max_packet_size_0(64)
        //  .build();
        //
        //toggle_led::spawn_after(50.millis()).ok();
        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {
            asm::wfi();
        }
    }

    // #[task(binds=USBD, shared = [date, rtc1, duty, freq], local = [usb_dev, serial, count: u32 = 0, len: usize = 0, data_arr: [u8; 30] = [0; 30]])]
    // fn on_usbd(mut cx: on_usbd::Context) {
    //
    //     let on_usbd::LocalResources {
    //         usb_dev,
    //         serial,
    //         count,
    //         len,
    //         data_arr,
    //     } = cx.local;
    //
    //     // Create a buffer to read USB data
    //     let mut buf = [0u8; 64];
    //     usb_dev.poll(&mut [serial]);
    //     *count = (*count + 1) % 1000;
    //
    //     // Read USB data and process it
    //     match serial.read(&mut buf) {
    //         Ok(count) if count > 0 => {
    //             for c in buf[0..count].iter_mut() {
    //                 match *c {
    //                     13 => { // Carriage return signifies end of command
    //                         let slice = &data_arr[0..*len];
    //                         // Debug: Print the command received
    //                         rprintln!("Command received: {:?}", core::str::from_utf8(slice));
    //
    //                         match parse_result(slice) {
    //                             Ok(Command::TimeSet(hour, minute, second)) => {
    //                                 cx.shared.date.lock(|date| {
    //                                     rprintln!("Setting time: {:?}:{:?}:{:?}", hour, minute, second);
    //                                     date.set_time(hour, minute, second);
    //                                 });
    //                             }
    //                             Ok(Command::DateSet(year, month, day)) => {
    //                                 cx.shared.date.lock(|date| {
    //                                     rprintln!("Setting date: {:?}-{:?}-{:?}", year, month, day);
    //                                     date.set_date(year, month, day);
    //                                 });
    //                             }
    //                             Ok(Command::AlarmSet(year, month, day, hour, minute, second)) => {
    //                                 cx.shared.rtc1.lock(|rtc1| {
    //                                     cx.shared.date.lock(|date| {
    //                                         let alarm_date = DateTime::new(year, month, day, hour, minute, second);
    //                                         let alarm_time = date.get_datetime_difference_as_seconds(&alarm_date);
    //                                         rtc1.clear_counter();
    //                                         rtc1.set_compare(RtcCompareReg::Compare1, (alarm_time*8) as u32).unwrap();
    //                                         rtc1.enable_counter();
    //                                     });
    //                                 });
    //                             }
    //                             Err(e) => {
    //                                 rprintln!("error {:?}", e);
    //                                 serial.write("Error!\r".as_bytes());
    //                             }
    //                         }
    //                         *len = 0;
    //                     },
    //                     _ => {
    //                         // Accumulate command data
    //                         if *len < data_arr.len() {
    //                             data_arr[*len] = *c;
    //                             *len += 1;
    //                         } else {
    //                             rprintln!("Data array full, cannot store more characters");
    //                         }
    //                     }
    //                 }
    //             }
    //         },
    //         _ => {} // No data read, or an error occurred
    //     }
    // }
}
