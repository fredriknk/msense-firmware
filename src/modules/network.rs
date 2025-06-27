use embassy_sync::{channel::Receiver, blocking_mutex::raw::{NoopRawMutex,CriticalSectionRawMutex}};
use embassy_sync::signal::Signal;
use embassy_time::{Timer};

use super::{
    modem::{parse_xmonitor_response, try_tcp_write, strip_prefix_suffix},
    sensors::gas::{SingleSampleStorage},
    sensors::battery::{BatteryStatus},
    util::build_cbor_payload,
    config,
};

use config::NUM_MINUTES_PER_SEND;

use defmt::Debug2Format;

static LTE_SIGNAL: Signal<CriticalSectionRawMutex, LteTrigger> = Signal::new();


enum LteTrigger {
    _TriggerLteConnect,
    TriggerLteSend,
}

#[embassy_executor::task]
pub async fn lte_trigger_loop(     
) {
    loop{
        LTE_SIGNAL.signal(LteTrigger::TriggerLteSend);
        Timer::after_secs(60*NUM_MINUTES_PER_SEND).await;
    }
}

async fn send_sampledata<
    const TX: usize,
    const RX: usize,
    const N:  usize,
>(
    stream: &mut nrf_modem::TlsStream,               // ← no lifetime
    imei:   &str,
    gas_rx: &Receiver<'static, NoopRawMutex, SingleSampleStorage, N>,
    batt_rx:&Receiver<'static, NoopRawMutex, BatteryStatus,       N>,
    tx_buf: &mut [u8; TX],
    rx_buf: &mut [u8; RX],
) -> Result<(), ()> {

    /* ---- %XMONITOR ---- */
    let at_xmon = nrf_modem::send_at::<128>("AT%XMONITOR").await
        .map_err(|e| { defmt::error!("AT%XMONITOR fail: {:?}", Debug2Format(&e)); () })?;

    let xmon = parse_xmonitor_response(at_xmon.as_str())
        .ok_or_else(|| { defmt::error!("Parse %XMONITOR fail"); () })?;

    /* ---- build & send ---- */
    if let Some(payload) = build_cbor_payload(imei, &xmon, gas_rx, batt_rx, tx_buf) {
        try_tcp_write(stream, payload).await.map_err(|_| ())?; // already logged
    }

    /* ---- receive ---- */
    match stream.receive(rx_buf).await {
        Ok(buf) => defmt::debug!("RX: {:?}", core::str::from_utf8(buf).unwrap()),
        Err(e)  => defmt::error!("RX err: {:?}", Debug2Format(&e)),
    }

    Ok(())                           // success – caller will deactivate
}
#[embassy_executor::task]
pub async fn lte_task(
    gas_rx:  Receiver<'static, NoopRawMutex, SingleSampleStorage, { config::DATASTORE_SIZE }>,
    batt_rx: Receiver<'static, NoopRawMutex, BatteryStatus,       { config::DATASTORE_SIZE }>,
) {
    let mut tx_buf = [0u8; config::TX_SIZE];
    let mut rx_buf = [0u8; config::RX_SIZE];

    
    let response = nrf_modem::send_at::<128>("AT+CGSN=1").await.unwrap();
    defmt::info!("AT+CGSN imei response: {:?}", response.as_str());
    let imei: &str;

    if let Some(imei_str) = strip_prefix_suffix("+CGSN: \"", response.as_str(),"\"\r\nOK\r\n") {
        defmt::info!("IMEI: {:?}", imei_str);
        imei = imei_str;
        defmt::info!("IMEI: {:?}", imei);
    } else {
        defmt::warn!("IMEI Prefix not found in response.");
        imei = "888888888888888";
    }

    loop {
        LTE_SIGNAL.wait().await;                // wake-up edge
        match nrf_modem::TlsStream::connect(
            config::HOST_ADDRESS,
            config::TCP_PORT,
            nrf_modem::PeerVerification::Enabled,
            &[config::CERT_CHAIN],
            Some(&[nrf_modem::CipherSuite::TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256]),
            true,//Use session ticket if available
        ).await {
            Ok(mut stream) => {
                if send_sampledata::<{config::TX_SIZE}, {config::RX_SIZE}, {config::DATASTORE_SIZE}>(
                &mut stream,
                imei,
                &gas_rx,
                &batt_rx,
                &mut tx_buf,
                &mut rx_buf,
                ).await.is_err() {}

                let _ = stream.deactivate().await;
            }
            Err(e) => defmt::error!("TLS connect Fail: {:?}", Debug2Format(&e)),
        }
    }
}

#[embassy_executor::task]
pub async fn send_button(host_pin: embassy_nrf::gpio::AnyPin) {
    let mut host = embassy_nrf::gpio::Input::new(host_pin, embassy_nrf::gpio::Pull::Up);
    loop {
        host.wait_for_falling_edge().await;
        LTE_SIGNAL.signal(LteTrigger::TriggerLteSend);
        embassy_time::Timer::after_millis(10000).await;
    }
}
