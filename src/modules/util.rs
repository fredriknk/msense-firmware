use minicbor::{Encoder, encode::write::Cursor};
use embassy_time::Instant;
use embassy_sync::channel::Receiver;
use defmt::{info};

use crate::modules::sensors::types::{SingleSampleStorage, BatteryStatus};
use crate::modules::modem::XMonitorData;

/// Fills `tx_buf` with a CBOR-encoded payload and returns the byte slice.
///
/// * `imei` – device IMEI string
/// * `xmon` – parsed %XMONITOR information
/// * `gas_rx` / `batt_rx` – receivers for sensor channels
/// * returns `Ok(encoded_slice)` on success
pub fn build_cbor_payload<'a, const N: usize, const M: usize>(
    imei: &str,
    xmon: &XMonitorData<'_>,
    gas_receiver:   &Receiver<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, SingleSampleStorage,  N>,
    battery_receiver:  &Receiver<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, BatteryStatus,       M>,
    tx_buf:   &'a mut [u8],
) -> Option<&'a [u8]> {
    // ------------ queue lengths -------------
    let gas_measurements     = gas_receiver.len();
    let battery_measurements = battery_receiver.len();
    let time = Instant::now().as_millis();
    info!("Gas queue: {}, battery queue: {}", gas_measurements, battery_measurements);

    // ---------- CBOR outer map size ----------
    let outer_map_fields: u64 = 1 // "d"
        + if gas_measurements     != 0 { 1 } else { 0 } // "m"
        + if battery_measurements != 0 { 1 } else { 0 }; // "b"

    let start = Instant::now();
    let end_pos = {
        // Re-borrow the slice so the outer `tx_buf` stays usable later
        let cursor  = Cursor::new(&mut *tx_buf);
        let mut enc = Encoder::new(cursor);

        let _ = enc.map(outer_map_fields);
        let _ = enc.str("d");
        let _ = enc.map(7);
        let _ = enc.str("IM");
        let _ = enc.str(imei);
        let _ = enc.str("C");
        let _ = enc.u64(time);
        let _ = enc.str("RS");
        let _ = enc.int(xmon.rsrp.unwrap().parse::<i32>().unwrap().into());
        let _ = enc.str("SN");
        let _ = enc.int(xmon.snr.unwrap().parse::<i32>().unwrap().into());
        let _ = enc.str("PL");
        let _ = enc.str(xmon.plmn.unwrap());
        let _ = enc.str("TA");
        let _ = enc.str(xmon.tac.unwrap());
        let _ = enc.str("CI");
        let _ = enc.str(xmon.cell_id.unwrap());

        
        if gas_measurements!=0 {
            let _ = enc.str("m");
            let _ = enc.array(gas_measurements as u64);
            for _ in 0..gas_measurements {
                let gas_data = gas_receiver.try_receive();
                if gas_data.is_ok() {
                    let gas_data_unwrap = gas_data.unwrap();
                    let _ = enc.map(5);
                    let _ = enc.str("T");
                    let _ = enc.f32(gas_data_unwrap.temperature);
                    let _ = enc.str("H");
                    let _ = enc.f32(gas_data_unwrap.humidity);
                    let _ = enc.str("P");
                    let _ = enc.f32(gas_data_unwrap.pressure);
                    let _ = enc.str("G");
                    let _ = enc.f32(gas_data_unwrap.gas_resistance);
                    let _ = enc.str("C");
                    let _ = enc.u64(gas_data_unwrap.timestamp);
                }
            }
        }
        if battery_measurements!=0{
            let _ = enc.str("b");
            let _ = enc.array(battery_measurements as u64);
            for _ in 0..battery_measurements {
                let battery_data_ = battery_receiver.try_receive();
                if battery_data_.is_ok() {
                    let battery_data = battery_data_.unwrap();
                    let _ = enc.map(4);
                    let _ = enc.str("A");
                    let _ = enc.f32(battery_data.battery_current);
                    let _ = enc.str("T");
                    let _ = enc.f32(battery_data.battery_temperature);
                    let _ = enc.str("V");
                    let _ = enc.f32(battery_data.battery_voltage);
                    let _ = enc.str("C");
                    let _ = enc.u64(battery_data.timestamp);
                }
            }
        }
        enc.writer().position()
    };

    info!("Encoded length: {}, time: {} µs", end_pos, start.elapsed().as_micros());
    let payload = &tx_buf[..end_pos];
    info!("Encoded bytes: {:?}", payload);

    Some(payload)
}