
use embassy_nrf::{
    bind_interrupts,
    peripherals::{self, SERIAL0},
    twim::{self, Twim},
    gpio::AnyPin,
};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex},
    mutex::Mutex,
};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    SERIAL0 => twim::InterruptHandler<peripherals::SERIAL0>;
});

/// Single global `Mutex` wrapping the TWIM peripheral.
static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<'static, SERIAL0>>> = StaticCell::new();

/// What each consumer gets back.
pub struct I2cHandles {
    pub npm1300: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
    pub bme680:  I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
    pub ads1115: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
}

/// Initialise the shared TWIM bus *once*
///
/// Call this from `main()` **before** you spawn any tasks that use I²C.
pub fn init(
    twi_port: peripherals::SERIAL0,   // whatever board::split() returns
    sda: AnyPin,
    scl: AnyPin,
) -> I2cHandles {
    defmt::info!("Configuring TWIM…");

    let mut cfg = twim::Config::default();
    cfg.sda_pullup = true;      // board without external pull-ups
    cfg.scl_pullup = true;

    // 1) Concrete TWIM peripheral
    let i2c = Twim::new(twi_port, Irqs, sda, scl, cfg);

    // 2) Wrap in a Mutex so shared-bus can schedule async use
    let i2c_bus = Mutex::new(i2c);

    // 3) Put it in a StaticCell so it lives for 'static
    let i2c_bus = I2C_BUS.init(i2c_bus);

    // 4) Hand out three virtual devices
    I2cHandles {
        npm1300: I2cDevice::new(i2c_bus),
        bme680:  I2cDevice::new(i2c_bus),
        ads1115: I2cDevice::new(i2c_bus),
    }
}