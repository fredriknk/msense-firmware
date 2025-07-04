#![allow(non_snake_case)]

pub const BOARD_NAME:&str  = "DEV";

use embassy_nrf::{Peripherals, gpio::Pin};
use crate::board_types::{Board, Pins};

pub fn split(p: Peripherals) -> Board {
    Board {
        wdt:      p.WDT,          // ← ​and here
        pins: Pins {
            led1:   p.P0_02.degrade(),
            led2:   p.P0_03.degrade(),
            button: p.P0_06.degrade(),
        },
    }
}