#![allow(non_snake_case)]

pub const BOARD_NAME:&str  = "2_0";

use embassy_nrf::{Peripherals, gpio::Pin};
use crate::board_types::{Board, Pins};

pub fn split(p: Peripherals) -> Board {
    Board {
        twi_port: p.SERIAL0,      // ← ​move it out here
        wdt:      p.WDT,          // ← ​and here
        pins: Pins {
            led1:   p.P0_06.degrade(),
            led2:   p.P0_05.degrade(),
            heater: p.P0_31.degrade(),
            sensor: p.P0_27.degrade(),
            host:   p.P0_04.degrade(),
            button: p.P0_07.degrade(),
            sda:    p.P0_28.degrade(),
            scl:    p.P0_29.degrade(),
        },
    }
}