// src/board_types.rs
use embassy_nrf::{
    gpio::{AnyPin},
    peripherals::{SERIAL0, WDT},
};

pub struct Pins {
    pub led1:   AnyPin,
    pub led2:   AnyPin,
    pub heater: AnyPin,
    pub sensor: AnyPin,
    pub host:   AnyPin,
    pub button: AnyPin,
    pub sda:    AnyPin,
    pub scl:    AnyPin,
}

/// Everything the application needs, harvested in one call
pub struct Board {
    pub pins: Pins,
    pub twi_port: SERIAL0,
    pub wdt: WDT,
}