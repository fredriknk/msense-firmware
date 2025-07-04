// src/board_types.rs
use embassy_nrf::gpio::AnyPin;
use embassy_nrf::peripherals::WDT;
#[cfg(feature = "i2c")]
use embassy_nrf::peripherals::SERIAL0;

pub struct Pins {
    pub led1:   AnyPin,
    pub led2:   AnyPin,
    pub button: AnyPin,
    #[cfg(feature = "sensors")]
    pub heater: AnyPin,
    #[cfg(feature = "sensors")]
    pub sensor: AnyPin,
    #[cfg(feature = "npm1300")]
    pub host:   AnyPin,
    #[cfg(feature = "i2c")]
    pub sda:    AnyPin,
    #[cfg(feature = "i2c")]
    pub scl:    AnyPin,
}

/// Everything the application needs, harvested in one call
pub struct Board {
    pub pins: Pins,
    #[cfg(feature = "i2c")]
    pub twi_port: SERIAL0,
    pub wdt: WDT,
}