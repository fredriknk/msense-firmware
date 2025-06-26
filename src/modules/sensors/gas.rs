use embedded_ads111x::{
    ADS111x,
    ADS111xConfig,
    InputMultiplexer,
    ProgramableGainAmplifier,
    DataRate,
};
use bosch_bme680::{
    AsyncBme680,
    Configuration,
    DeviceAddress,
    Oversampling,
    IIRFilter,
};
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
use embassy_time::{Instant,Timer};
use defmt::Debug2Format;
use embassy_nrf::{peripherals::SERIAL0, twim::Twim};

use static_cell::StaticCell;

use embassy_sync::{
    blocking_mutex::raw::{
        NoopRawMutex,
        CriticalSectionRawMutex,
    },
    channel::{
        Channel,
        Sender,
    },
    signal::Signal,
};

use super::battery::{BATTERY_SIGNAL,BatteryTrigger};
use super::super::config::{DATASTORE_SIZE, TEMP_INTERVAL, NUM_SAMPLES_PER_AGGREGATION, NUM_SAMPLES_PER_BATTERY_READ};

use crate::I2cDevice;


pub static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();

pub struct EnvData {
    /// Temperature in °C
    pub temperature: f32,
    /// Relative humidity in %
    pub humidity: f32,
    /// Pressure in hPa
    pub pressure: f32,
    /// counter
    pub bmecounter: u32,
    /// Gas resistance in Ohms
    pub gas_resistance: f32,
    /// Gas resistance counter
    pub gascounter: u32,
}


pub struct SingleSampleStorage {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
    pub gas_resistance: f32,
    pub timestamp: u64,
}

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

//Calculate the resistance of a gas sensor based on the voltage reading
fn gas_resistance(voltage : f32) -> f32 {

    let r2 = 12000.0;
    let vref = 2.0;
    let vout = -voltage;
    let r1 = r2 * (vref/vout - 1.0);
    r1
}

#[embassy_executor::task]
pub async fn tictoctrigger(        
    heater_pin: AnyPin, 
    sensor_pin: AnyPin,
    led_pin1:AnyPin,
    led_pin2:AnyPin,
) {
    let mut led1 = Output::new(led_pin1, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(led_pin2, Level::Low, OutputDrive::Standard);
    let mut power = Output::new(heater_pin, Level::Low, OutputDrive::Standard);
    let mut sensor = Output::new(sensor_pin, Level::Low, OutputDrive::Standard);

    loop{
        Timer::after_millis(29900).await;
        //led1.set_high();
        power.set_high();
        
        Timer::after_millis(98).await;
        
        led2.set_high();
        sensor.set_high();
        GAS_SIGNAL.signal(true);

        Timer::after_millis(3).await;
        
        sensor.set_low();
        power.set_low();
        led1.set_low();
        led2.set_low();
    }
}

#[embassy_executor::task]
pub async fn gas_sensor_task(
        i2c_dev1: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
        i2c_dev2: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
        sender: Sender<'static, NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>,
    ) {
    
    let bme680config =  Configuration::builder()
        .temperature_oversampling(Oversampling::By2)
        .pressure_oversampling(Oversampling::By16)
        .humidity_oversampling(Oversampling::By1)
        .filter(IIRFilter::Coeff1)
        .gas_config(None)
        .build();


    let mut bme680 = AsyncBme680::new(i2c_dev2, DeviceAddress::Primary, embassy_time::Delay,25);

    let _ = bme680.initialize(&bme680config).await;

    let adsconfig = ADS111xConfig::default()
        .mux(InputMultiplexer::AIN0AIN1)
        .dr(DataRate::SPS860)
        .pga(ProgramableGainAmplifier::V2_048);

    let mut adc = match ADS111x::new(i2c_dev1, 0x48u8, adsconfig){
        Err(e) => panic!("Error {:?}", e),
        Ok(x) => x,
    };

    let _ = adc.write_config(None).await;

    let mut counter: u64 = 0;

    let mut sensor_data    = EnvData {
        temperature: 0.0,
        humidity: 0.0,
        pressure: 0.0,
        bmecounter: 0,
        gas_resistance: 0.0,
        gascounter: 0,
    };
    loop {
        let _ = GAS_SIGNAL.wait().await;

        match adc.read_single_voltage(None).await {
            Ok(volt) => {
                let gas_res = gas_resistance(volt);
                sensor_data.gas_resistance += gas_res;
                sensor_data.gascounter += 1;
                defmt::debug!("Voltage: {:?}", volt);
                defmt::debug!("Gas Resistance: {:?}", gas_res);
            }
            Err(e) => {
                defmt::warn!("ADC read error: {}", Debug2Format(&e));
            }
        };

        if counter%TEMP_INTERVAL == 0 {
            match bme680.measure().await{
                Ok(data) => {              
                    sensor_data.temperature += data.temperature;
                    sensor_data.pressure += data.pressure;
                    sensor_data.humidity += data.humidity;
                    sensor_data.bmecounter += 1;
                
                    defmt::debug!("Temperature: {:?}", data.temperature);
                    defmt::debug!("Pressure: {:?}", data.pressure);
                    defmt::debug!("Humidity: {:?}", data.humidity);
                }
                Err(e) => {
                    defmt::warn!("BME680 measure error {}", Debug2Format(&e));
                }
            };
        }

        if counter%NUM_SAMPLES_PER_AGGREGATION == 0 {
            defmt::debug!("Sending data to channel...");
            
            let data_send = SingleSampleStorage {
                temperature: if sensor_data.bmecounter > 0 {
                    sensor_data.temperature / sensor_data.bmecounter as f32
                } else {
                    defmt::warn!("No valid BME680 measurements; using default temperature");
                    -999.0
                },
                humidity: if sensor_data.bmecounter > 0 {
                    sensor_data.humidity / sensor_data.bmecounter as f32
                } else {
                    defmt::warn!("No valid BME680 measurements; using default humidity");
                    -999.0
                },
                pressure: if sensor_data.bmecounter > 0 {
                    sensor_data.pressure / sensor_data.bmecounter as f32
                } else {
                    defmt::warn!("No valid BME680 measurements; using default pressure");
                    -999.0
                },
                gas_resistance: if sensor_data.gascounter > 0 {
                    sensor_data.gas_resistance / sensor_data.gascounter as f32
                } else {
                    defmt::warn!("No valid gas resistance measurements; using default gas resistance");
                    -999.0
                },
                timestamp: Instant::now().as_millis(),
            };

            let err = sender.try_send(data_send);
            if err.is_err() {
                defmt::warn!("Sample Queue full");
            }
            else {
                defmt::debug!("Sample Data sent");
            }
            sensor_data = EnvData {
                temperature: 0.0,
                humidity: 0.0,
                pressure: 0.0,
                bmecounter: 0,
                gas_resistance: 0.0,
                gascounter: 0,
            };
        }
        if counter%NUM_SAMPLES_PER_BATTERY_READ == 0 {
            BATTERY_SIGNAL.signal(BatteryTrigger::TriggerBatteryRead);
        }
        counter += 1;
    }
}