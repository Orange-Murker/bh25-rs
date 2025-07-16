#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

const NUM_LEDS: usize = 16;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::rmt::Rmt;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use log::info;
use smart_leds::{brightness, SmartLedsWrite as _, RGB8};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

fn random_colours(rng: &mut Rng) -> [RGB8; NUM_LEDS] {
    let mut colours = [RGB8::default(); NUM_LEDS];

    for colour in &mut colours {
        let random = rng.random();
        colour.r = (random & 0xff) as u8;
        colour.g = ((random >> 8) & 0xff) as u8;
        colour.b = ((random >> 16) & 0xff) as u8;
    }

    colours
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

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng.clone(), peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    let frequency = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, frequency).expect("Failed to initialize RMT0");
    let mut led = SmartLedsAdapter::new(
        rmt.channel0,
        peripherals.GPIO10,
        smart_led_buffer!(NUM_LEDS),
    );

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        let colours = random_colours(&mut rng).into_iter();
        led.write(brightness(colours, 100)).unwrap();
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
