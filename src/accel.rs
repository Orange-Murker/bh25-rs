use esp_hal::{
    gpio::AnyPin,
    i2c::{self, master::I2c},
    peripherals::I2C0,
    time::Rate,
    Blocking,
};
use lis2dh12::{Lis2dh12, SlaveAddr};
use log::{error, info};

pub fn init_accel<'a>(
    i2c: I2C0<'a>,
    sda: AnyPin<'a>,
    scl: AnyPin<'a>,
) -> Lis2dh12<I2c<'a, Blocking>> {
    let i2c_config = i2c::master::Config::default()
        .with_frequency(Rate::from_khz(400))
        .with_timeout(i2c::master::BusTimeout::BusCycles(20));
    let i2c = I2c::new(i2c, i2c_config)
        .expect("Failed to create I2C")
        .with_sda(sda)
        .with_scl(scl);
    let mut accel =
        Lis2dh12::new(i2c, SlaveAddr::Alternative(true)).expect("Failed to create accel");

    let mut retries = 100;
    while retries > 0 {
        if let Ok(id) = accel.get_device_id() {
            info!("Accel ID: {id}");
            break;
        } else {
            error!("Failed to init accel. Trying again");
            retries -= 1;
        }
    }

    accel
        .set_mode(lis2dh12::Mode::HighResolution)
        .expect("Failed to set accel mode");
    accel
        .set_odr(lis2dh12::Odr::Hz200)
        .expect("Failed to set accel data rate");
    accel
        .set_fs(lis2dh12::FullScale::G4)
        .expect("Failed to set accel full scale");
    accel
        .enable_axis((true, true, true))
        .expect("Failed to enable axes");

    accel
}
