use embassy_nrf::wdt::{self, HaltConfig, Watchdog};
use embassy_nrf::peripherals::WDT;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use super::config::LTE_WATCHDOG_TIMEOUT_SECONDS;

pub static LTE_HEARTBEAT: AtomicU32 = AtomicU32::new(0);

/// Call this after a *completed* LTE attempt (success or fail).
pub fn lte_heartbeat_bump() {
    LTE_HEARTBEAT.fetch_add(1, Relaxed);
}

#[embassy_executor::task]
pub async fn watchdog_task(mut handle: wdt::WatchdogHandle) {
    let mut last_cnt = LTE_HEARTBEAT.load(Relaxed);
    let mut last_send = embassy_time::Instant::now();
    let stale = embassy_time::Duration::from_secs(LTE_WATCHDOG_TIMEOUT_SECONDS as u64);
    let mut timed_out = false;

    loop {
        embassy_time::Timer::after_millis(1000).await;

        let cur = LTE_HEARTBEAT.load(Relaxed);
        if cur != last_cnt {
            last_cnt = cur;
            last_send = embassy_time::Instant::now();
        }

        if embassy_time::Instant::now() - last_send < stale {
            timed_out = false;
            handle.pet();
        } else if !timed_out {
            defmt::error!("LTE watchdog timeout (> {:?} without progress).", stale);
            timed_out = true;
        }
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
