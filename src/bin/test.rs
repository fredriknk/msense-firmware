#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};

use embassy_time::{
    Timer,
    Instant,
    Duration,
};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{
        Channel, Receiver, Sender
    }, 
};

use minicbor::{Decoder, Encoder};
use minicbor::encode::write::Cursor;

use static_cell::StaticCell;

use sha_256::Sha256;

use rand::Rng;

use {defmt_rtt as _, panic_probe as _};

const DATASTORE_SIZE: usize = 6;

const BATTERY_SIZE: usize = 1;

static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, BATTERY_SIZE>> = StaticCell::new();


struct SingleSampleStorage {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    gas_resistance: f32,
    timestamp: u64,
}

struct BatteryStatus{
    battery_voltage: f32,
    battery_current: f32,
    battery_temperature: f32,
    timestamp: u64,
}

#[embassy_executor::task]
async fn gas_sensor_task(sender: Sender<'static, NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>) {

    loop {
        let gas_data = SingleSampleStorage {
            temperature: 25.0+((Instant::now().as_micros() as f32)%10.),
            humidity: 50.0+((Instant::now().as_micros() as f32)%10.),
            pressure: 1000.0+((Instant::now().as_micros() as f32)%10.),
            gas_resistance: 1000.0+((Instant::now().as_micros() as f32)%10.),
            timestamp: Instant::now().as_millis(),
        };

        let err = sender.try_send(gas_data);
        if err.is_err() {

        }
        else {
        }

        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn battery_status_task(sender: Sender<'static, NoopRawMutex, BatteryStatus, BATTERY_SIZE>) {

    loop {
        let battery_data = BatteryStatus {
            battery_voltage: 3.7+((Instant::now().as_micros() as f32)%10.)/10.,
            battery_current: 0.5+((Instant::now().as_micros() as f32)%10.)/100.,
            battery_temperature: 25.0+((Instant::now().as_micros() as f32)%10.),
            timestamp: Instant::now().as_millis(),
        };

        let err = sender.try_send(battery_data);
        if err.is_err() {
           
        }

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn data_receiver(battery_receiver: Receiver<'static, NoopRawMutex, BatteryStatus, BATTERY_SIZE>,gas_receiver: Receiver<'static, NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>) {
    let mut sha256: Sha256 = Sha256::new();
    let mut timestamp_offset: u64 = 0;
    loop {

        let starttime = Instant::now();
        let gas_measurements = gas_receiver.len();
        let battery_measurements = battery_receiver.len();
        defmt::info!("Gas Queue length: {},battqueue {}", gas_measurements,battery_measurements);
        //iterate over num gas_measurements
        let mut buffer = [0u8; 2048];
        let cursor = Cursor::new(&mut buffer[..]);
        let mut encoder = Encoder::new(cursor);
        let uuid: [u8; 16] = [62, 146, 111, 117, 211, 27, 72, 82, 179, 148, 140, 19, 245, 168, 187, 110];

        let token: &[u8; 16] = b"1234567890abcdef"; 
        let time = Instant::now().as_millis();

        // Create a mutable array for data
        let mut data = [0u8; 16 + 8]; // adjust to your length if necessary

        let _ = encoder.begin_map();
        let _ = encoder.str("d");
        let _ = encoder.map(4);
        let _ = encoder.str("UU");
        let _ = encoder.bytes(&uuid);
        let _ = encoder.str("C");
        let _ = encoder.u64(time+timestamp_offset);
        let _ = encoder.str("TO");
        let _ = encoder.bytes(&hash);
        let _ = encoder.str("RS");
        let _ = encoder.int((37 as i32).into());
        
        if gas_measurements!=0 {
            let _ = encoder.str("m");
            let _ = encoder.array(gas_measurements as u64);
            for _ in 0..gas_measurements {
                let gas_data = gas_receiver.try_receive();
                if gas_data.is_ok() {
                    let gas_data_unwrap = gas_data.unwrap();
                    let _ = encoder.map(5);
                    let _ = encoder.str("T");
                    let _ = encoder.f32(gas_data_unwrap.temperature);
                    let _ = encoder.str("H");
                    let _ = encoder.f32(gas_data_unwrap.humidity);
                    let _ = encoder.str("P");
                    let _ = encoder.f32(gas_data_unwrap.pressure);
                    let _ = encoder.str("G");
                    let _ = encoder.f32(gas_data_unwrap.gas_resistance);
                    let _ = encoder.str("C");
                    let _ = encoder.u64(gas_data_unwrap.timestamp+timestamp_offset);
                }
            }
        }
        if battery_measurements!=0{
            let _ = encoder.str("b");
            let _ = encoder.array(battery_measurements as u64);
            for _ in 0..battery_measurements {
                let battery_data_ = battery_receiver.try_receive();
                if battery_data_.is_ok() {
                    let battery_data = battery_data_.unwrap();
                    let _ = encoder.map(4);
                    let _ = encoder.str("A");
                    let _ = encoder.f32(battery_data.battery_current);
                    let _ = encoder.str("T");
                    let _ = encoder.f32(battery_data.battery_temperature);
                    let _ = encoder.str("V");
                    let _ = encoder.f32(battery_data.battery_voltage);
                    let _ = encoder.str("C");
                    let _ = encoder.u64(battery_data.timestamp+timestamp_offset);
                }
            }
        }
        let _ = encoder.end();

        // Obtain the current position
        let pos = encoder.writer().position();

        // Safely split the buffer at the current position
        let (encoded_data, _remaining_buffer) = buffer.split_at_mut(pos);

        // Log the encoded data
        defmt::info!("Length {:?}",pos);
        defmt::info!("Time spent in encoding {:?}",starttime.elapsed().as_micros());
        defmt::info!("Encoded data {:?}",encoded_data);
        Timer::after_millis(3000).await;
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_05, Level::Low, OutputDrive::Standard);

    let gas_channel = GAS_CHANNEL.init(Channel::new());
    let battery_status_channel = BATTERY_STATUS_CHANNEL.init(Channel::new());

    spawner.spawn(gas_sensor_task(gas_channel.sender())).unwrap();
    spawner.spawn(battery_status_task(battery_status_channel.sender())).unwrap();

    spawner.spawn(data_receiver( battery_status_channel.receiver(),gas_channel.receiver())).unwrap();

    loop {
        led.set_high();
        Timer::after_millis(300).await;
        led.set_low();
        Timer::after_millis(300).await;
    }
}
