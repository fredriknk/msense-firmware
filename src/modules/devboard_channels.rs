use static_cell::StaticCell;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Sender;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};

use super::config::{
    DATASTORE_SIZE,
    NUM_SAMPLES_PER_AGGREGATION, 
    NUM_SAMPLES_PER_BATTERY_READ,
    MS_PER_SAMPLE,
};

pub static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
pub static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

pub struct BatteryStatus{
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub battery_temperature: f32,
    pub timestamp: u64,
}
pub struct SingleSampleStorage {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
    pub gas_resistance: f32,
    pub timestamp: u64,
}

async fn pseudorandom() -> f32 {
    let micros = embassy_time::Instant::now().as_micros() % 100; // 0‥=99
    micros as f32 / 100.0  
}

#[embassy_executor::task]
pub async fn fake_gas_data_task(
    gas_channel: Sender<'static, NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>,
) {
    loop {
        embassy_time::Timer::after_millis(MS_PER_SAMPLE*NUM_SAMPLES_PER_AGGREGATION).await; // Wait before next iteration
        // Simulate gas data with random values
        defmt::debug!("Fake gas data added to channel");
        let gas_data = SingleSampleStorage {
            temperature: 25.0 + (pseudorandom().await * 0.5), // Random temperature between 25 and 30
            humidity: 30.0 + (pseudorandom().await * 1.0), // Random humidity between 50 and 60
            pressure: 1013.0 + (pseudorandom().await * 0.1), // Random pressure around 1013 hPa
            gas_resistance: 13000.0 + (pseudorandom().await * 25.0), // Random gas resistance
            timestamp: embassy_time::Instant::now().as_millis() as u64,
        };
        gas_channel.send(gas_data).await;
        
    }
}

#[embassy_executor::task]
pub async fn fake_battery_data_task(
    battery_channel: Sender<'static, NoopRawMutex, BatteryStatus, DATASTORE_SIZE>,
) {
    loop {
        embassy_time::Timer::after_millis(MS_PER_SAMPLE*NUM_SAMPLES_PER_BATTERY_READ).await; // Wait before next iteration
        defmt::debug!("Fake battery data added to channel");
        // Simulate battery data with random values
        let battery_data = BatteryStatus {
            battery_voltage: 4.0 + (pseudorandom().await * 0.01), // Random voltage around 3.7V
            battery_current: 0.005 + (pseudorandom().await * 0.005), // Random current between 0.5A and 0.7A
            battery_temperature: 25.0 + (pseudorandom().await * 5.0), // Random temperature between 25 and 30
            timestamp: embassy_time::Instant::now().as_millis() as u64,
        };
        battery_channel.send(battery_data).await;
        
    }
}

#[embassy_executor::task]
pub async fn ledblink_task(
    led_pin1:AnyPin,
    led_pin2:AnyPin,
) {
    let mut led1 = Output::new(led_pin1, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(led_pin2, Level::Low, OutputDrive::Standard);
    loop {
        led1.set_high();
        led2.set_low();
        embassy_time::Timer::after_secs(1).await;
        led1.set_low();
        led2.set_high();
        embassy_time::Timer::after_secs(1).await;
    }
}