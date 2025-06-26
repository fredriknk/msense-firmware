use embassy_nrf::gpio::{AnyPin, Level,Input, Output, OutputDrive};
use embassy_nrf::wdt::{self, HaltConfig, Watchdog};
use embassy_nrf::peripherals::WDT;
use defmt::{info};

use crate::LteTrigger;
use crate::LTE_SIGNAL;

#[embassy_executor::task]
pub async fn watchdog_task(mut handle: wdt::WatchdogHandle) {
    loop {
        embassy_time::Timer::after_millis(5000).await; 
        handle.pet();                                 
    }
}

pub fn init_watchdog(wdt_periph: WDT) -> wdt::WatchdogHandle {
    info!("Setting up Watchdog…");

    let mut cfg = wdt::Config::default();
    cfg.timeout_ticks = 32_768 * 30;             // 30 s
    cfg.action_during_debug_halt = HaltConfig::PAUSE;

    let (_wdt, [handle]) = match Watchdog::try_new(wdt_periph, cfg) {
        Ok(t)  => t,
        Err(_) => {
            info!("Watchdog already active with wrong config, waiting for it to timeout...");
            loop {}
        }
    };

    handle
}

#[embassy_executor::task]
pub async fn blink_task(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::Standard);
    loop {
        led.set_high();
        embassy_time::Timer::after_millis(10).await;
        led.set_low();
        embassy_time::Timer::after_millis(10000).await;
    }
}

#[embassy_executor::task]
pub async fn button_interrupt(host_pin: AnyPin) {
    let mut host = Input::new(host_pin, embassy_nrf::gpio::Pull::Up);
    loop {
        host.wait_for_falling_edge().await;
        LTE_SIGNAL.signal(LteTrigger::TriggerLteSend);
        embassy_time::Timer::after_millis(10000).await;
    }
}
