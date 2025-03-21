#![no_std]
#![no_main]

extern crate tinyrlibc;
use core::error;
use core::net::SocketAddr;
use cortex_m::peripheral::NVIC;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::interrupt;
use embassy_time::Timer;
use embassy_nrf::pac;
use nrf_modem::ConnectionPreference;
use nrf_modem::SystemMode;
use nrf_modem::{MemoryLayout};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use defmt::debug;

use {defmt_rtt as _, panic_probe as _};
//get hostname from env file
const HOST_ADDRESS : &str = "iot.nekko.no";
unsafe extern "C" {
    unsafe static __start_ipc: u8;
    unsafe static __end_ipc: u8;
}

pub async fn setup_modem() -> Result<(), nrf_modem::Error> {
    fn configure_modem_non_secure() -> u32 {
        // The RAM memory space is divided into 32 regions of 8 KiB.
        // Set IPC RAM to nonsecure
        const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
        const RAM_START: u32 = 0x2000_0000; // 256kb
        let ipc_start: u32 = unsafe { &__start_ipc as *const u8 } as u32;
        let ipc_reg_offset = (ipc_start - RAM_START) / SPU_REGION_SIZE;
        let ipc_reg_count =
            (unsafe { &__end_ipc as *const u8 } as u32 - ipc_start) / SPU_REGION_SIZE;
        let spu = embassy_nrf::pac::SPU;
        let range = ipc_reg_offset..(ipc_reg_offset + ipc_reg_count);
        debug!("marking region as non secure: {}", range);
        for i in range {
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
        ipc_start
    }
    let ipc_start = configure_modem_non_secure();
    // Interrupt Handler for LTE related hardware. Defer straight to the library.
    #[interrupt]
    #[allow(non_snake_case)]
    fn IPC() {
        nrf_modem::ipc_irq_handler();
    }

    let mut cp = unwrap!(cortex_m::Peripherals::take());

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init_with_custom_layout(
        SystemMode {
            lte_support: true,
            lte_psm_support: true,
            nbiot_support: false,
            gnss_support: false,
            preference: ConnectionPreference::None,
        },
        MemoryLayout {
            base_address: ipc_start,
            tx_area_size: 0x2000,
            rx_area_size: 0x2000,
            trace_area_size: 0,
        },
    )
    .await?;
    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);

    let _ = setup_modem().await.unwrap();
    let response = nrf_modem::send_at::<64>("AT+CGMR").await.unwrap();
    defmt::info!("AT+CGMR response: {:?}", response.as_str());

    
    let mut counter = 0;

    loop {
        match nrf_modem::TlsStream::connect(
            HOST_ADDRESS,
            8443,
            nrf_modem::PeerVerification::Enabled,
            &[16842753],
            Some(&[nrf_modem::CipherSuite::TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256]),
            1 as u32,
        ).await{
            Ok(stream) => {
            defmt::info!("Connected to host");
            stream
                .write("test".as_bytes())
                .await
                .unwrap();

            let mut buffer = [0; 1024];
            let received = stream
                .receive(&mut buffer)
                .await
                .unwrap();

            defmt::info!("Received: {:?}", core::str::from_utf8(received).unwrap());

            stream
                .deactivate().await.unwrap();
            },
            Err(_e) => {
                defmt::info!("Error connecting to host ");
            }
        }
        
        /*                        
        if counter == 0 {
            let response = nrf_modem::send_at::<64>("AT%XCONNSTAT=1").await.unwrap();
            defmt::info!("%XCONNSTAT response: {:?}", response.as_str());
        }

        counter += 1;
        let response = nrf_modem::send_at::<64>("AT%XCONNSTAT?").await.unwrap();
        defmt::info!("%XCONNSTAT after send:{} {:?}",counter, response.as_str());*/

        let seconds = 300;
        defmt::info!("Waiting {} seconds before trying again",seconds);
        Timer::after_secs(seconds).await;
    }
}