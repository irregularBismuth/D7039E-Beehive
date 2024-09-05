#![no_main]
#![no_std]
// #![deny(unsafe_code)]
// #![deny(warnings)]
mod timekeeper;
mod alarm;

use crate::timekeeper::DateTime;
use crate::alarm::*;
use heapless::String;
use itoa;
use nrf52833_hal::{pac::TWIM0,
saadc::*,pwm::*,};
use {
    command_parser::*,
    embedded_hal::digital::v2::InputPin,
    nrf52833_hal as hal,
    hal::{
        ppi::{self, ConfigurablePpi, Ppi},
        twim,
        Twim,
        prelude::*,
        clocks::*,
        gpio::{Level, Output, Input, Pin, PushPull,self, p0::P0_31, Disconnected, Floating},
        gpiote::*,
        rtc::{Rtc, RtcCompareReg, RtcInterrupt},
        usbd::{UsbPeripheral, Usbd}, 
    },
    fugit::{MillisDurationU64, TimerInstantU64},
    rtt_target::{rprintln, rtt_init_print},
    cortex_m::asm,
    panic_rtt_target as _, 
    systick_monotonic::*,
    usb_device::{
        class_prelude::UsbBusAllocator,
        device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    },
    usbd_serial::{SerialPort, USB_CLASS_CDC},
    ssd1306::{
        mode::BufferedGraphicsMode,
        prelude::*,
        I2CDisplayInterface,
        Ssd1306,
        command::*,
    },
    embedded_graphics::{
        mono_font::{
            ascii::{FONT_9X18, FONT_9X18_BOLD},
            MonoTextStyle, MonoTextStyleBuilder,
        },
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    },
    
};

const TIMER_HZ: u32 = 32_768; // 4 Hz (250 ms granularity)
const PD_CONVERSION: f32 = 0.2021; // Photodiode conversion rate. Unit mV/(value read)
type Led = Pin<Output<PushPull>>;
type Button = Pin<hal::gpio::Input<hal::gpio::PullDown>>;
type OLED = Ssd1306<I2CInterface<Twim<TWIM0>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;
type Buzzer = Pin<Output<PushPull>>;
type Instant = TimerInstantU64<TIMER_HZ>;
type Duration = MillisDurationU64;

enum DateTimeTuple {
    Date(u32, u8, u8),
    Time(u8, u8, u8),
}


#[rtic::app(device = nrf52833_hal::pac, dispatchers= [TIMER0])]
mod app {
    

    use hal::{comp, pac::{rtc0::evtenclr::COMPARE0_A, PWM0}};

    use super::*;
    

    // #[monotonic(binds = RTC0, default = true)]
    // type MyMono = hal::monotonic::MonotonicTimer<hal::pac::RTC0, 32_768u32>;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<TIMER_HZ>;

    #[shared]
    struct Shared {
        //led1: Led,
        date: DateTime,
        rtc: Rtc<nrf52833_hal::pac::RTC0>,
        rtc1: Rtc<nrf52833_hal::pac::RTC1>,
        gpiote: Gpiote,
        freq: u32,
        duty: u8,
        buzzer: Buzzer,
        display: OLED,
        pwm0: Pwm<PWM0>,
        pwm0_duty: u16,
    }

    #[local]
    struct Local {
        btn1: Button,
        btn2: Button,
        btn3: Button,
        btn4: Button,
        btn5: Button,
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
        saadc: hal::saadc::Saadc,
        saadc_pin: P0_31<Input<Floating>>,
        saadc_latest_read: f32,
        oled_on: bool,
    }

     // Initialization function
    #[init(local = [
        clocks: Option<Clocks<ExternalOscillator, Internal, LfOscStarted>> = None,
        usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
    ])]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("\n---init---\n");

        
        /************************** SECTION: Ports and time *************************/
        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);
        let mut mono = Systick::new(cx.core.SYST, 64_000_000);
        
        
        /******************** SECTION: Buttons, buzzer and led setup ***************/
        let buzzer = port1.p1_09.into_push_pull_output(Level::Low).degrade();

        let led = port0.p0_11.into_push_pull_output(Level::Low).degrade();

        let btn1 = port0.p0_02.into_pulldown_input().degrade();
        let btn2 = port0.p0_03.into_pulldown_input().degrade();
        let btn3 = port0.p0_28.into_pulldown_input().degrade();
        let btn4 = port0.p0_29.into_pulldown_input().degrade();
        let btn5 = port0.p0_30.into_pulldown_input().degrade();
       

        /************************** SECTION: PWM Led ********************************/
        let mut pwm0 = Pwm::new(cx.device.PWM0);
        // Write 160000u32.hz() for 62 Hz and 1u32.hz() for 1 Hz. There is conversion 
        // done in the background with prescaling and countermode. 1.
        pwm0.set_period(5000u32.hz())
            .set_output_pin(Channel::C0, led)
            .set_counter_mode(CounterMode::UpAndDown)
            .set_prescaler(Prescaler::Div8)
            .enable_interrupt(PwmEvent::PwmPeriodEnd)
            .enable();
        pwm0.set_max_duty(3200);
        rprintln!("period {}", pwm0.get_period().0);
        let pwm0_duty = pwm0.get_max_duty();
        pwm0.set_duty(Channel::C0, pwm0_duty);
        pwm0.task_start_seq0();

        /****************************************************************************/

        
        /************************** SECTION: RTC setup *****************************/
        //let clocks = hal::clocks::Clocks::new(cx.device.CLOCK);
        let clocks = Clocks::new(cx.device.CLOCK);
        
        cx.local.clocks.replace(clocks.start_lfclk().enable_ext_hfosc());
        
         /************************** SECTION: Config a compare reg to update time ****/
        let mut rtc = Rtc::new(cx.device.RTC0, 0).unwrap();
        rtc.set_compare(RtcCompareReg::Compare0, hal::clocks::LFCLK_FREQ).unwrap();
        rtc.enable_event(RtcInterrupt::Compare0);
        rtc.enable_interrupt(RtcInterrupt::Compare0, Some(&mut cx.core.NVIC));
        rprintln!("Starting RTC");
        rtc.enable_counter();

        /************************** SECTION: Set time *****************************/
        // year, month, day, hour, minute, second
        let date = DateTime::new(2024, 10, 31, 22, 0, 0);

        /************************** SECTION: Config compare reg for alarm ***********/
        let mut rtc1 = Rtc::new(cx.device.RTC1, 4095).unwrap();
        rtc1.set_compare(RtcCompareReg::Compare1, 100000).unwrap();
        rtc1.enable_event(RtcInterrupt::Compare1);
        rtc1.enable_interrupt(RtcInterrupt::Compare1, Some(&mut cx.core.NVIC));
        rprintln!("Alarm setup");
        rtc1.enable_counter();

        /************************** SECTION: Setup gpiote *****************************/
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        // Set buttons to generate port event
        gpiote.port().input_pin(&btn1).high();
        gpiote.port().input_pin(&btn2).high();
        gpiote.port().input_pin(&btn3).high();
        gpiote.port().input_pin(&btn4).high();
        gpiote.port().input_pin(&btn5).high();
        // Enable interrupt for port event
        gpiote.port().enable_interrupt();

        

        /************************** SECTION: USB shananigans ***************************/
        //Enable USBD interrupt
        let _ = cx.device.USBD.intenset.write(|w| w.sof().set());

        let usb_bus = UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
            cx.device.USBD,
            cx.local.clocks.as_ref().unwrap(),
        )));
        cx.local.usb_bus.replace(usb_bus);

        let serial = SerialPort::new(&cx.local.usb_bus.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(
            &cx.local.usb_bus.as_ref().unwrap(),
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake taxi")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64)
        .build();

        /************************** SECTION: OLED initialization *************************/
        let i2c = Twim::new(
            cx.device.TWIM0,
            twim::Pins {
                scl: port0.p0_04.into_floating_input().degrade(),
                sda: port0.p0_05.into_floating_input().degrade(),
            },
            twim::Frequency::K400,
        );

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        let oled_on = true;
        

        /************************ SECTION: Photodiode setup ************************/
        // Setting up the SAADC
        let saadc_config = SaadcConfig::default();
        let mut saadc = hal::Saadc::new(cx.device.SAADC, saadc_config);
        // Photodiode pin
        let mut saadc_pin = port0.p0_31.into_floating_input();
        // Blocking read from saadc for `saadc_config.time` microseconds
        let saadc_latest_read = saadc.read(&mut saadc_pin).unwrap() as f32 * PD_CONVERSION;
        /***************************************************************************/

        let duty = 50;
        let freq = 1;
        let on = true;


        let _ = update_oled::spawn();
        //toggle_led::spawn_after(50.millis()).ok();
        (Shared {date, rtc, rtc1, gpiote, freq, duty, buzzer, display, pwm0, pwm0_duty}, Local {btn1, btn2, btn3, btn4, btn5, usb_dev, serial, saadc, saadc_pin, saadc_latest_read, oled_on}, init::Monotonics(mono))
    }
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop{
            asm::wfi();
        }
    }

    // #[task(local = [])]
    // fn convert_tuple_str(cx: convert_tuple::Context){//, tuple: (u8, u8, u8)) -> &string{
    //     //let mut buffer = itoa::Buffer::new();
    //     //let mut string = String::<32>::new();
    //     //string.push_str(buffer.format(time.0)).unwrap();
       
    // }

    
   
    #[task(local = [], shared = [date, display])] 
    fn update_oled(mut cx: update_oled::Context){
      
        cx.shared.date.lock(|date| {
            cx.shared.display.lock(|display| {
                let time = date.get_time();
                let daytum = date.get_date();
                let date_tuple = DateTimeTuple::Date(daytum.0, daytum.1, daytum.2);
                let time_tuple = DateTimeTuple::Time(time.0, time.1, time.2);
                let time_text = convert_date_time_to_str(time_tuple, false);
                let date_text = convert_date_time_to_str(date_tuple, true);
                
                let _ = display.clear(embedded_graphics::pixelcolor::BinaryColor::Off);
                
                let text_style = MonoTextStyleBuilder::new()
                    .font(&FONT_9X18_BOLD)
                    .text_color(BinaryColor::On)
                    .underline()
                    .build();

                Text::with_baseline(&time_text, Point::new(25, 40), text_style, Baseline::Top)
                    .draw(display)
                    .unwrap();

                Text::with_baseline(&date_text, Point::new(25, 0), text_style, Baseline::Top)
                    .draw(display)
                    .unwrap();
                
                display.flush().unwrap();
        })
    });
    }

    #[task(priority = 2, binds = RTC0, local =[], shared = [date, rtc])]
    fn keeping_time(mut cx: keeping_time::Context) {
        cx.shared.rtc.lock(|rtc|{
            cx.shared.date.lock(|date| {
                date.update(0, 0, 0, 0, 0, 1);
                let daytum = date.get_date();
                let time = date.get_time();
                //rprintln!("Date: {}-{}-{}", daytum.0, daytum.1, daytum.2);
                //rprintln!("Time: {}:{}:{}", time.0, time.1, time.2);
                rtc.reset_event(RtcInterrupt::Compare0);
                rtc.clear_counter();
            })
        });
        let _ = update_oled::spawn(); 
        let _ = photodiode_read::spawn();
    }

    #[task(priority = 1, binds = RTC1, local =[], shared = [date, rtc1])]
    fn alarm(mut cx: alarm::Context) {
        rprintln!("½½½½½½½½½½½½½½½½½½½½½½½½½I AM THE ALARMER½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½½");
        cx.shared.rtc1.lock(|rtc|{
            let _ = activate_buzzer::spawn();
            rtc.reset_event(RtcInterrupt::Compare1);
            rtc.disable_counter();
        });
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
    #[task(binds=USBD, shared = [date, rtc1, duty, freq], local = [usb_dev, serial, count: u32 = 0, len: usize = 0, data_arr: [u8; 30] = [0; 30]])]
    fn on_usbd(mut cx: on_usbd::Context) {
        use command_parser::*;

        let on_usbd::LocalResources {
            usb_dev,
            serial,
            count,
            len,
            data_arr,
        } = cx.local;

        // Create a buffer to read USB data
        let mut buf = [0u8; 64];
        usb_dev.poll(&mut [serial]);
        *count = (*count + 1) % 1000;

        // Read USB data and process it
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                for c in buf[0..count].iter_mut() {
                    match *c {
                        13 => { // Carriage return signifies end of command
                            let slice = &data_arr[0..*len];
                            // Debug: Print the command received
                            rprintln!("Command received: {:?}", core::str::from_utf8(slice));

                            match parse_result(slice) {
                                Ok(Command::TimeSet(hour, minute, second)) => {
                                    cx.shared.date.lock(|date| {
                                        rprintln!("Setting time: {:?}:{:?}:{:?}", hour, minute, second);
                                        date.set_time(hour, minute, second);
                                    });
                                }
                                Ok(Command::DateSet(year, month, day)) => {
                                    cx.shared.date.lock(|date| {
                                        rprintln!("Setting date: {:?}-{:?}-{:?}", year, month, day);
                                        date.set_date(year, month, day);
                                    });
                                }
                                Ok(Command::AlarmSet(year, month, day, hour, minute, second)) => {
                                    cx.shared.rtc1.lock(|rtc1| {
                                        cx.shared.date.lock(|date| {
                                            let alarm_date = DateTime::new(year, month, day, hour, minute, second);
                                            let alarm_time = date.get_datetime_difference_as_seconds(&alarm_date);
                                            rtc1.clear_counter();
                                            rtc1.set_compare(RtcCompareReg::Compare1, (alarm_time*8) as u32).unwrap();
                                            rtc1.enable_counter();
                                        });
                                    });
                                }
                                Err(e) => {
                                    rprintln!("error {:?}", e);
                                    serial.write("Error!\r".as_bytes());
                                }
                            }
                            *len = 0;
                        },
                        _ => {
                            // Accumulate command data
                            if *len < data_arr.len() {
                                data_arr[*len] = *c;
                                *len += 1;
                            } else {
                                rprintln!("Data array full, cannot store more characters");
                            }
                        }
                    }
                }
            },
            _ => {} // No data read, or an error occurred
        }
    }
    #[task(shared = [gpiote, display], local = [btn1, btn2, btn3, btn4, btn5, oled_on])]
    fn debounce(mut ctx: debounce::Context) {
        let btn1_pressed = ctx.local.btn1.is_high().unwrap();
        let btn2_pressed = ctx.local.btn2.is_high().unwrap();
        let btn3_pressed = ctx.local.btn3.is_high().unwrap();
        let btn4_pressed = ctx.local.btn4.is_high().unwrap();
        let btn5_pressed = ctx.local.btn5.is_high().unwrap();

        ctx.shared.gpiote.lock(|gpiote| {
            if btn1_pressed {
                ctx.shared.display.lock(|display| {
                rprintln!("Turn off display%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%!");
                // Manually run "task out" operation (toggle) on channel1 (toggles led1)
                if *ctx.local.oled_on {
                    let _ = display.set_display_on(false);
                    *ctx.local.oled_on = false;
                }
                else {
                    let _ = display.set_display_on(true);
                    *ctx.local.oled_on = true;
                }
                
                gpiote.channel1().task_out();
                });
            }
            if btn2_pressed {
                rprintln!("Alarm turned off!");
                // Manually run "task clear" on channel 1 (led1 on)
                let _ = deactivate_buzzer::spawn();
                gpiote.channel1().clear();
            }
            if btn3_pressed {
                rprintln!("SNOOZE 1 minute!");
                // Manually run "task set" on channel 1 (led1 off)
                let time = 60;
                let _ = snooze::spawn(time);

                gpiote.channel1().set();
            }
            if btn4_pressed {
                rprintln!("Button 4 was pressed!");
                // Manually run "task set" on channel 1 (led1 off)
                //gpiote.channel1().set();
            }
            if btn5_pressed {
                rprintln!("Button 5 was pressed!");
                // Manually run "task set" on channel 1 (led1 off)
                //gpiote.channel1().set();
            }
        });
    }

    #[task(shared = [buzzer])]
    fn activate_buzzer(mut cx: activate_buzzer::Context) {
        cx.shared.buzzer.lock(|buzzer| {
            buzzer.set_high().ok()
        });
    }

    #[task(shared = [buzzer])]
    fn deactivate_buzzer(mut cx: deactivate_buzzer::Context) {
        cx.shared.buzzer.lock(|buzzer| {
            buzzer.set_low().ok()
        });
    }
    #[task(shared = [rtc1])]
    fn snooze(mut cx: snooze::Context, time :u32) {
        cx.shared.rtc1.lock(|rtc| {
            rtc.clear_counter();
            rtc.set_compare(RtcCompareReg::Compare1, time*8).unwrap();
            rtc.enable_counter();
            let _ = deactivate_buzzer::spawn();

        });
    }
    #[task(local = [cnt:u32 = 0, saadc, saadc_pin, saadc_latest_read])]
    fn photodiode_read(cx: photodiode_read::Context) {
        let latest_read = cx.local.saadc.read(cx.local.saadc_pin).unwrap() as f32 * PD_CONVERSION;
        if *cx.local.cnt % 50000 == 0 {
            rprintln!("Voltage reading from photodiode: {} mV", latest_read);

        }
        *cx.local.cnt += 1;
        *cx.local.saadc_latest_read = latest_read;
        
    }
    fn convert_date_time_to_str(tuple: DateTimeTuple, date: bool) -> String<32> {
        let mut buffer = itoa::Buffer::new();
        let mut string = String::<32>::new();
        let colon_or_dash = if date { '-' } else { ':' };
        
        match tuple {
            DateTimeTuple::Date(year, month, day) => {
                string.push_str(&buffer.format(year)).unwrap();
                string.push(colon_or_dash).unwrap();
                string.push_str(&buffer.format(month)).unwrap();
                string.push(colon_or_dash).unwrap();
                string.push_str(&buffer.format(day)).unwrap();
            },
            DateTimeTuple::Time(hour, minute, second) => {
                string.push_str(&buffer.format(hour)).unwrap();
                string.push(colon_or_dash).unwrap();
                string.push_str(&buffer.format(minute)).unwrap();
                string.push(colon_or_dash).unwrap();
                string.push_str(&buffer.format(second)).unwrap();
            },
        }
    
        string
    }

    #[task(shared = [pwm0])]
    fn pwm_stop(mut cx: pwm_stop::Context) {
        cx.shared.pwm0.lock(|pwm| {
            pwm.stop();
        });
    }

    #[task(binds = PWM0, local = [direction: i16 = -1, step_size: u16 = 1], shared = [pwm0, pwm0_duty])]
    fn ramping_led(mut cx: ramping_led::Context) {
        cx.shared.pwm0.lock(|pwm| {
            cx.shared.pwm0_duty.lock(|duty| {
                //rprintln!("period {}", pwm.get_period().0);
                let duty_wrap = core::num::Wrapping(*duty);
                let new = match *cx.local.direction {
                    dir if dir < 0 => (duty_wrap - core::num::Wrapping(*cx.local.step_size)).0,
                    _ => (duty_wrap + core::num::Wrapping(*cx.local.step_size)).0,
                };
                
                let res = if new > (core::u16::MAX / 2) {
                    if *cx.local.direction < 0 {
                        //core::u16::MAX / 2
                        3200
                    } else {
                        0
                    }
                } else {
                    new
                };
                *duty = res;
                //rprintln!("duty {}", *duty);
                pwm.set_duty(Channel::C0, *duty);
            });
        });
    }

    #[task(shared = [pwm0, pwm0_duty])]
    fn toggle_led(mut cx: toggle_led::Context) {
        cx.shared.pwm0.lock(|pwm| {
            cx.shared.pwm0_duty.lock(|duty| {
                rprintln!("toggle_led");
                if *duty != 0 { // Set high
                    *duty = 0;
                    rprintln!("toggle_led high");
                    pwm.set_duty(Channel::C0, 0);
                } else { // Set low
                    *duty = pwm.get_max_duty();
                    rprintln!("toggle_led low");
                    pwm.set_duty(Channel::C0, pwm.get_max_duty());
                }
            });
        });
        toggle_led::spawn_after(1.secs()).ok();

    }
}
