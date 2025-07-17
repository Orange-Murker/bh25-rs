#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod accel;
mod pov;

const NUM_LEDS: usize = 16;

const SAMPLING_PERIOD_S: f32 = 1.0 / 200.0;
const SAMPLING_PERIOD_US: u64 = (SAMPLING_PERIOD_S * 1000000.0) as u64;
const VELOCITY_AVG_SAMPLES: usize = 10;

const LED_HORIZONTAL_DISTANCE_M: f32 = 0.01;

const NUM_EFFECTS: u32 = 4;

const G: f32 = 9.80665;

// Peripherals
static ACCEL: Mutex<RefCell<Option<Lis2dh12<I2c<'static, Blocking>>>>> =
    Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<PeriodicTimer<'_, Blocking>>>> = Mutex::new(RefCell::new(None));
static UP_BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

// Shared variables
static LED_PERIOD_US: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(100000));
// true -right
// false - left
static LED_DIR: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(true));
static STATIONARY: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static CURRENT_EFFECT: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

use core::cell::RefCell;
use core::u32;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::rmt::Rmt;
use esp_hal::timer::PeriodicTimer;
use esp_hal::{clock::CpuClock, time::Instant};

use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use log::info;
use micromath::F32Ext;
use tinybmp::Bmp;

use crate::accel::init_accel;
use crate::pov::PovDisplay;

use accelerometer::Accelerometer;
use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::Rgb888,
    prelude::*,
    text::Text,
};
use esp_hal::gpio::{Event, Input, InputConfig, Io, Pin};
use esp_hal::i2c::master::I2c;
use esp_hal::{handler, ram, Blocking};
use lis2dh12::Lis2dh12;

use esp_backtrace as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[handler]
#[ram]
fn button_handler() {
    let current_effect = critical_section::with(|cs| {
        UP_BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
        let mut current_effect = CURRENT_EFFECT.borrow_ref_mut(cs);
        *current_effect = (*current_effect + 1) % NUM_EFFECTS;
        *current_effect
    });
    info!("Current effect: {}", current_effect);
}

#[handler]
#[ram]
// Safe because only used in the interrupt
#[allow(static_mut_refs)]
fn accel_int() {
    static mut AVG_VELOCITY: [f32; VELOCITY_AVG_SAMPLES] = [0.0; VELOCITY_AVG_SAMPLES];
    static mut VELOCITY: f32 = 0.0;

    critical_section::with(|cs| {
        TIMER.borrow_ref_mut(cs).as_mut().unwrap().clear_interrupt();
    });

    let accel = critical_section::with(|cs| {
        ACCEL
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .accel_norm()
            .unwrap()
    });

    // Acceleration in m/s without G
    let total_accel =
        ((accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt() - 1.0) * G;

    let dir = accel.y < 0.0;
    let stationary = total_accel.abs() < 0.5;

    unsafe {
        if stationary {
            VELOCITY = 0.0;
        }
        VELOCITY += total_accel * SAMPLING_PERIOD_S;
    };

    let avg_velocity = unsafe {
        AVG_VELOCITY.rotate_right(1);
        AVG_VELOCITY[0] = VELOCITY;
        AVG_VELOCITY.iter().sum::<f32>() / AVG_VELOCITY.len() as f32
    };

    let mut led_period_us = ((LED_HORIZONTAL_DISTANCE_M / avg_velocity.abs()) * 1000000.0) as u64;

    // Cap LED period
    if led_period_us > 50000 {
        led_period_us = 50000;
    }

    // Make sure that the response is fast from being stationary
    if stationary {
        led_period_us = 1000;
    }

    critical_section::with(|cs| {
        LED_PERIOD_US.replace(cs, led_period_us);
        LED_DIR.replace(cs, dir);
        STATIONARY.replace(cs, stationary);
    });
}

#[embassy_executor::task]
async fn display(led: SmartLedsAdapter<esp_hal::rmt::Channel<esp_hal::Blocking, 0>, 385>) {
    let mut pov = PovDisplay::<_, 70, NUM_LEDS>::new(led);

    let mut stationary_started_time = Instant::now();

    let mut previous_stationary = false;
    let mut previous_dir = false;
    let mut previous_current_effect = u32::MAX;

    loop {
        let (period, dir, stationary, current_effect) = critical_section::with(|cs| {
            (
                *LED_PERIOD_US.borrow(cs).borrow(),
                *LED_DIR.borrow(cs).borrow(),
                *STATIONARY.borrow(cs).borrow(),
                *CURRENT_EFFECT.borrow(cs).borrow(),
            )
        });

        if previous_current_effect != current_effect {
            pov.clear();
            match current_effect {
                0 => {}
                1 => {
                    let style = MonoTextStyle::new(&FONT_9X15, Rgb888::WHITE);

                    Text::new("Testing", Point::new(0, 10), style)
                        .draw(&mut pov)
                        .unwrap();
                }
                2 => {
                    let image_data = include_bytes!("nyan.bmp");
                    let bmp = Bmp::<Rgb888>::from_slice(image_data).unwrap();
                    bmp.draw(&mut pov).unwrap();
                }
                3 => {
                    let image_data = include_bytes!("rust.bmp");
                    let bmp = Bmp::<Rgb888>::from_slice(image_data).unwrap();
                    bmp.draw(&mut pov).unwrap();
                }
                _ => panic!("Invalid effect"),
            }
        }

        if dir {
            if previous_dir != dir {
                pov.reset_col();
            } else {
                pov.next_col();
            }
            // pov.green();
        } else {
            pov.previous_col();
            // pov.red();
        }

        let mut reset = false;
        if stationary {
            if !previous_stationary {
                stationary_started_time = Instant::now();
            }

            if (Instant::now() - stationary_started_time).as_millis() > 50 {
                reset = true;
            }
        }

        if reset {
            pov.reset_col();
            pov.show_colour(smart_leds::colors::RED, current_effect as usize, 200);
        } else {
            pov.flush(100);
        }
        Timer::after(Duration::from_micros(period)).await;

        previous_stationary = stationary;
        previous_dir = dir;
        previous_current_effect = current_effect;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let rng = Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng.clone(), peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    let frequency = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, frequency).expect("Failed to initialize RMT0");
    let led = SmartLedsAdapter::new(
        rmt.channel0,
        peripherals.GPIO10,
        smart_led_buffer!(NUM_LEDS),
    );

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(button_handler);

    let input_config = InputConfig::default();
    let mut up_button = Input::new(peripherals.GPIO2, input_config);
    critical_section::with(|cs| {
        up_button.listen(Event::FallingEdge);
        UP_BUTTON.borrow_ref_mut(cs).replace(up_button);
    });

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let mut timer = PeriodicTimer::new(timg1.timer0);
    timer.set_interrupt_handler(accel_int);
    timer.enable_interrupt(true);
    critical_section::with(|cs| {
        timer
            .start(esp_hal::time::Duration::from_micros(SAMPLING_PERIOD_US))
            .unwrap();
        TIMER.replace(cs, Some(timer));
    });

    let accel = init_accel(
        peripherals.I2C0,
        peripherals.GPIO6.degrade(),
        peripherals.GPIO7.degrade(),
    );

    critical_section::with(|cs| {
        ACCEL.borrow_ref_mut(cs).replace(accel);
    });

    spawner
        .spawn(display(led))
        .expect("Failed to spawn display task");

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
