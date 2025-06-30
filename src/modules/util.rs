use minicbor::{Encoder, encode::write::Cursor};
use embassy_time::Instant;
use embassy_sync::channel::Receiver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;


use crate::modules::sensors::gas::SingleSampleStorage;
use crate::modules::sensors::battery::BatteryStatus;
use crate::modules::modem::XMonitorData;
use crate::modules::error_log;

/// Fills `tx_buf` with a CBOR-encoded payload and returns the byte slice.
///
/// * `imei` – device IMEI string
/// * `xmon` – parsed %XMONITOR information
/// * `gas_rx` / `batt_rx` – receivers for sensor channels
/// * returns `Ok(encoded_slice)` on success
pub fn build_cbor_payload<'a, const N: usize, const M: usize, const Q:usize>(
    imei: &str,
    xmon: &XMonitorData<'_>,
    gas_receiver:   &Receiver<'static, NoopRawMutex, SingleSampleStorage,  N>,
    battery_receiver:  &Receiver<'static, NoopRawMutex, BatteryStatus,       M>,
    log_rx: &Receiver<'static, NoopRawMutex, error_log::LogLine, Q>,
    tx_buf:   &'a mut [u8],
) -> Option<&'a [u8]> {
    // ------------ queue lengths -------------
    let gas_measurements     = gas_receiver.len();
    let battery_measurements = battery_receiver.len();
    let err_count = log_rx.len();
    let time = Instant::now().as_millis();
    defmt::debug!("Gas queue: {}, battery queue: {}, error queue {}", gas_measurements, battery_measurements, err_count);

    // ---------- CBOR outer map size ----------
    let outer_map_fields: u64 = 1 // "d"
        + if gas_measurements     != 0 { 1 } else { 0 } // "m"
        + if battery_measurements != 0 { 1 } else { 0 } // "b"
        + if err_count != 0 { 1 } else { 0 }; // "e"

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
        let _ = enc.int(xmon.rsrp.unwrap_or("0").parse::<i32>().unwrap_or(0).into());
        let _ = enc.str("SN");
        let _ = enc.int(xmon.snr.unwrap_or("0").parse::<i32>().unwrap_or(0).into());
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
        if err_count != 0 {
            let _ = enc.str("e");
            let _ = enc.array(err_count as u64);
            for _ in 0..err_count {
                if let Ok(line) = log_rx.try_receive() {
                    let _ = enc.map(3);
                    let _ = enc.str("C"); let _ = enc.u64(line.ts_ms);
                    let _ = enc.str("l"); let _ = enc.u8(line.lvl as u8);   // 0 = info, 1 = error
                    let _ = enc.str("M"); let _ = enc.str(line.msg);
                }
            }
        }
        enc.writer().position()
    };

    defmt::debug!(
        "Data payload encoded: {} bytes in {} µs",
        end_pos,
        start.elapsed().as_micros()
    );

    Some(&tx_buf[..end_pos])
}