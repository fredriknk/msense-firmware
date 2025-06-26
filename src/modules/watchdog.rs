use embassy_nrf::wdt::{self, HaltConfig, Watchdog};
use embassy_nrf::peripherals::WDT;

#[embassy_executor::task]
pub async fn watchdog_task(mut handle: wdt::WatchdogHandle) {
    loop {
        embassy_time::Timer::after_millis(5000).await; 
        handle.pet();                                 
    }
}

pub fn init_watchdog(wdt_periph: WDT) -> wdt::WatchdogHandle {
   defmt::info!("Setting up Watchdog…");

    let mut cfg = wdt::Config::default();
    cfg.timeout_ticks = 32_768 * 30;             // 30 s
    cfg.action_during_debug_halt = HaltConfig::PAUSE;

    let (_wdt, [handle]) = match Watchdog::try_new(wdt_periph, cfg) {
        Ok(t)  => t,
        Err(_) => {
           defmt::info!("Watchdog already active with wrong config, waiting for it to timeout...");
            loop {}
        }
    };

    handle
}
