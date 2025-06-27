//dont warn deadcode
//#![allow(dead_code)]

use cortex_m::peripheral::NVIC;
use nrf_modem::{MemoryLayout};
use embassy_nrf::pac;
use nrf_modem::ConnectionPreference;
use nrf_modem::SystemMode;
use embassy_nrf::interrupt;
use defmt::{Debug2Format};
use nrf_modem::TlsStream;

#[allow(dead_code)]
#[derive(Debug, defmt::Format)]
pub struct XMonitorData<'a> {
    pub reg_status: &'a str,
    pub full_name:        Option<&'a str>,
    pub short_name:       Option<&'a str>,
    pub plmn:             Option<&'a str>,
    pub tac:              Option<&'a str>,
    pub act:              Option<&'a str>,
    pub band:             Option<&'a str>,
    pub cell_id:          Option<&'a str>,
    pub phys_cell_id:     Option<&'a str>,
    pub earfcn:           Option<&'a str>,
    pub rsrp:             Option<&'a str>,
    pub snr:              Option<&'a str>,
    pub edrx_value:       Option<&'a str>,
    pub active_time:      Option<&'a str>,
    pub periodic_tau_ext: Option<&'a str>,
}

impl<'a> Default for XMonitorData<'a> {
    fn default() -> Self {
        Self {
            reg_status: "",
            full_name:        None,
            short_name:       None,
            plmn:             None,
            tac:              None,
            act:              None,
            band:             None,
            cell_id:          None,
            phys_cell_id:     None,
            earfcn:           None,
            rsrp:             None,
            snr:              None,
            edrx_value:       None,
            active_time:      None,
            periodic_tau_ext: None,
        }
    }
}

/// Strips surrounding quotes if the string starts and ends with a double quote.
fn trim_quotes(s: &str) -> &str {
    let s = s.trim();
    if s.len() >= 2 && s.starts_with('"') && s.ends_with('"') {
        &s[1..s.len() - 1]
    } else {
        s
    }
}

pub fn _strip_at<'a>(prefix: &'a str, line: &'a str) -> Option<&'a str> {
    let line = &line.strip_prefix(prefix)?;
    let line = &line.strip_suffix("\r\nOK\r\n")?;
    let line = &line.trim();
    Some(line)
}

pub fn strip_prefix_suffix<'a>(prefix: &'a str, line: &'a str,suffix: &'a str) -> Option<&'a str> {
    let line = &line.strip_prefix(prefix)?;
    let line = &line.strip_suffix(suffix)?;
    let line = &line.trim();
    Some(line)
}

pub fn parse_xmonitor_response(line: &str) -> Option<XMonitorData> {
    let line = line.trim();

    // Split on commas. Each entry might be quoted, so we'll trim them below.
    let mut split_iter = line.split(',').map(|s| s.trim());

    // The first field is the mandatory <reg_status>.
    let reg_status_raw = split_iter.next()?;
    let reg_status = trim_quotes(reg_status_raw);

    // All remaining fields are optional, so we capture them in sequence
    // as `Option<&str>`, trimming quotes where present.
    let full_name = split_iter.next().map(trim_quotes);
    let short_name = split_iter.next().map(trim_quotes);
    let plmn = split_iter.next().map(trim_quotes);
    let tac = split_iter.next().map(trim_quotes);
    let act = split_iter.next().map(trim_quotes);
    let band = split_iter.next().map(trim_quotes);
    let cell_id = split_iter.next().map(trim_quotes);
    let phys_cell_id = split_iter.next().map(trim_quotes);
    let earfcn = split_iter.next().map(trim_quotes);
    let rsrp = split_iter.next().map(trim_quotes);
    let snr = split_iter.next().map(trim_quotes);
    let edrx_value = split_iter.next().map(trim_quotes);
    let active_time = split_iter.next().map(trim_quotes);
    let periodic_tau_ext = split_iter.next().map(trim_quotes);

    Some(XMonitorData {
        reg_status,
        full_name,
        short_name,
        plmn,
        tac,
        act,
        band,
        cell_id,
        phys_cell_id,
        earfcn,
        rsrp,
        snr,
        edrx_value,
        active_time,
        periodic_tau_ext,
    })
}


pub async fn try_tcp_write(
    stream: &mut TlsStream,      // no lifetime parameter
    payload: &[u8],
) -> Result<(), nrf_modem::Error> {
    match stream.write(payload).await {
        Ok(_) => {
           defmt::info!("Data sent to host");
            Ok(())
        }
        Err(e) => {
           defmt::error!("Error sending data: {:?}", Debug2Format(&e));
            Err(e)                       // caller decides what to do next
        }
    }
}

extern crate tinyrlibc;
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
        defmt::debug!("marking region as non secure: {}", range);
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

    let mut cp = defmt::unwrap!(cortex_m::Peripherals::take());

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

