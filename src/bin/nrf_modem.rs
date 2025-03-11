#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use nrf_modem::*;
use embassy_nrf::{
    interrupt,
    pac,
};
use cortex_m::peripheral::NVIC;
use defmt::unwrap;

use {defmt_rtt as _, panic_probe as _};


// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
#[allow(non_snake_case)]
fn IPC() {
    nrf_modem::ipc_irq_handler();
}




#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_05, Level::Low, OutputDrive::Standard);

    // Set IPC RAM to nonsecure
    const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
    const RAM_START: u32 = 0x2000_0000; // 256kb
    let spu = embassy_nrf::pac::SPU;
    let region_start = 0x2000_000 - RAM_START / SPU_REGION_SIZE;
    let region_end = region_start + (0x2000_8000 - 0x2000_0000) / SPU_REGION_SIZE;
    for i in region_start..region_end {
        spu.ramregion(i as usize).perm().write(|w| {
            w.set_execute(true);
            w.set_write(true);
            w.set_read(true);
            w.set_secattr(false);
            w.set_lock(false);
        })
    }

    // Set regulator access registers to nonsecure
    spu.periphid(4).perm().write(|w| w.set_secattr(false));
    // Set clock and power access registers to nonsecure
    spu.periphid(5).perm().write(|w| w.set_secattr(false));
    // Set IPC access register to nonsecure
    spu.periphid(42).perm().write(|w| w.set_secattr(false));

        
    let mut cp = unwrap!(cortex_m::Peripherals::take());

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init(SystemMode {
        lte_support: true,
        lte_psm_support: true,
        nbiot_support: true,
        gnss_support: true,
        preference: ConnectionPreference::None,
    })
    .await
    .unwrap();

    let response = nrf_modem::send_at::<64>("AT+CGMI").await.unwrap();
    assert_eq!(response.as_str(), "AT+CGMI\n\rNordic Semiconductor ASA\n\rOK\n\r");

    loop {
        led.set_high();
        Timer::after_millis(300).await;
        led.set_low();
        Timer::after_millis(300).await;
    }
}
