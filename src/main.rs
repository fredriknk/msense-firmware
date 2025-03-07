#![no_std]
#![no_main]

//! Example demonstrating control of the NPM1300 PMIC's leds

use core::{array, error, ops::Index, time, u64};

use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, gpio::{AnyPin, Input}, pac::rtc::regs::Counter, peripherals::{self, SERIAL0}, twim::{self, Twim}
};

use embassy_nrf::gpio::{
    Level,
    Output,
    OutputDrive,
    Pin,
};
use embassy_time::{
    Timer,
    Instant,
};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

use embassy_sync::{
    blocking_mutex::raw::{
        NoopRawMutex,
        CriticalSectionRawMutex,
    },
    channel::{
        Channel,
        Sender,
        Receiver
    },
    signal::Signal, 
    mutex::Mutex,
    blocking_mutex::raw::ThreadModeRawMutex,
};
use static_cell::StaticCell;

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
use {defmt_rtt as _, panic_probe as _};

use npm1300_rs::{
    leds::LedMode,
    NtcThermistorType,
    sysreg::VbusInCurrentLimit,
    charger::{
        ChargerTerminationVoltage,
        DischargeCurrentLimit,
        ChargerTerminationCurrentLevelSelect,
    },
    NPM1300,
};

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<SERIAL0>>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    SERIAL0 => twim::InterruptHandler<peripherals::SERIAL0>;
});

struct EnvData {
    /// Temperature in °C
    temperature: f32,
    /// Relative humidity in %
    humidity: f32,
    /// Pressure in hPa
    pressure: f32,
    /// counter
    bmecounter: u32,
    /// Gas resistance in Ohms
    gas_resistance: f32,
    /// Gas resistance counter
    gascounter: u32,

    
}
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

enum BatteryTrigger {
    TriggerBatteryRead,
    StartCharging,
}

const DATASTORE_SIZE: usize = 2;
const TEMP_INTERVAL: u64 = 2;
const BATTERYLOOPTIME: u64 = 10000;
const NUM_SAMPLES_PER_AGGREGATION: u64 = 12;
const NUM_SAMPLES_PER_BATTERY_READ: u64 = 24;

static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

static GAS_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, EnvData> = Signal::new();


#[embassy_executor::task]
async fn npm1300_task(
        i2c_dev: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
        sender: Sender<'static, NoopRawMutex, BatteryStatus, DATASTORE_SIZE>,
    ) {
    
    let mut npm1300 = NPM1300::new(i2c_dev, embassy_time::Delay);
    defmt::info!("Configuring LED modes...");
    let _ = npm1300.configure_led0_mode(LedMode::Host).await;
    defmt::info!("Configured LED0 mode to Host");
    let _ = npm1300.configure_led1_mode(LedMode::Charging).await;
    defmt::info!("Configured LED1 mode to Charging");
    let _ = npm1300.configure_led2_mode(LedMode::ChargingError).await;
    defmt::info!("Configured LED2 mode to ChargingError");

    let _ = npm1300.set_vbus_in_current_limit(VbusInCurrentLimit::MA1000).await;

    
    defmt::info!("Configuring NTC Resistor...");
    let _ = npm1300.configure_ntc_resistance(NtcThermistorType::Ntc10K, Some(3380.0)).await;
    let _ = npm1300.use_ntc_measurements().await;

    defmt::info!("Configuring Charging...");
    let _ = npm1300.set_charger_current(300).await;
    let _ = npm1300.set_termination_current_level(ChargerTerminationCurrentLevelSelect::SEL20).await;
    let _ = npm1300.set_normal_temperature_termination_voltage(ChargerTerminationVoltage::V4_20).await;
    let _ = npm1300.set_warm_temperature_termination_voltage(ChargerTerminationVoltage::V4_10).await;
    let _ = npm1300.set_discharge_current_limit(DischargeCurrentLimit::High).await;
    let _ = npm1300.enable_battery_charging().await;
    let _ = npm1300.enable_battery_recharge().await;
    let _ = npm1300.configure_ibat_measurement(true).await;

    loop {
        let battsig = BATTERY_SIGNAL.wait().await;
        match battsig {
            BatteryTrigger::TriggerBatteryRead => {
                defmt::info!("Triggering Battery Measurement");
                let vbat_voltage = npm1300.measure_vbat().await.unwrap();
                let _ = npm1300.measure_ntc().await;
                let ntc_temp = npm1300.get_ntc_measurement_result().await.unwrap();
                let die_temp = npm1300.measure_die_temperature().await.unwrap();
                let ibat_current = npm1300.measure_ibat().await.unwrap();
                let status = npm1300.get_charger_status().await.unwrap();
                defmt::info!("Charger Status: {:?}", status);
                defmt::info!("NTC Temp: {:?}, Die Temp: {:?}, VBAT Voltage: {:?}, IBAT Current: {:?}", ntc_temp,die_temp,vbat_voltage,ibat_current);

                let battery_data = BatteryStatus {
                    battery_voltage: vbat_voltage,
                    battery_current: ibat_current,
                    battery_temperature: ntc_temp,
                    timestamp: Instant::now().as_millis(),
                };

                let err = sender.try_send(battery_data);
                if err.is_err() {
                    defmt::warn!("Battery Queue full");
                }
                else {
                    defmt::info!("Battery Data sent");
                }
            },
            BatteryTrigger::StartCharging => {
                defmt::info!("Charging Battery");
                let _ = npm1300.set_vbus_in_current_limit(VbusInCurrentLimit::MA1000).await;
                BATTERY_SIGNAL.signal(BatteryTrigger::TriggerBatteryRead);
            }
        }
    };
}

fn gas_resistance(voltage : f32) -> f32 {

    let r2 = 51000.0;
    let vref = 2.0;
    let vout = -voltage;
    let r1 = r2 * (vref/vout - 1.0);
    r1
}

#[embassy_executor::task]
async fn gas_sensor_task(
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
        let volt =  adc.read_single_voltage(None).await.unwrap();
        let gas_res = gas_resistance(volt);
        sensor_data.gas_resistance = gas_res;
        sensor_data.gascounter += 1;

        if counter%TEMP_INTERVAL == 0 {
            let data = bme680.measure().await.unwrap();
            sensor_data.temperature += data.temperature;
            sensor_data.pressure += data.pressure;
            sensor_data.humidity += data.humidity;
            sensor_data.bmecounter += 1;
            defmt::debug!("Temperature: {:?}", data.temperature);
            defmt::debug!("Pressure: {:?}", data.pressure);
            defmt::debug!("Humidity: {:?}", data.humidity);
        }
        defmt::debug!("Voltage: {:?}", volt);
        defmt::debug!("Gas Resistance: {:?}", gas_res);
        if counter%NUM_SAMPLES_PER_AGGREGATION == 0 {
            defmt::info!("Sending data...");
            let data_send   = SingleSampleStorage {
                temperature: sensor_data.temperature / sensor_data.bmecounter as f32,
                humidity: sensor_data.humidity / sensor_data.bmecounter as f32,
                pressure: sensor_data.pressure / sensor_data.bmecounter as f32,
                gas_resistance: sensor_data.gas_resistance / sensor_data.gascounter as f32,
                timestamp: Instant::now().as_millis(),
            };

            let err = sender.try_send(data_send);
            if err.is_err() {
                defmt::warn!("Sample Queue full");
            }
            else {
                defmt::info!("Sample Data sent");
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

#[embassy_executor::task]
async fn tictoctrigger(        
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
        led1.set_high();
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
async fn charge_interrupt(host_pin: AnyPin) {
    let mut host = Input::new(host_pin, embassy_nrf::gpio::Pull::Down);
    loop {
        host.wait_for_rising_edge().await;
        BATTERY_SIGNAL.signal(BatteryTrigger::StartCharging);
        Timer::after_millis(2000).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Starting NPM1300 example");
    let p = embassy_nrf::init(Default::default());
    
    let led_pin1 = p.P0_06.degrade();
    let led_pin2 = p.P0_05.degrade();

    let heater_pin = p.P0_31.degrade();
    let sensor_pin = p.P0_27.degrade();

    let host_pin = p.P0_04.degrade();

    let sda_pin = p.P0_28.degrade();
    let scl_pin = p.P0_29.degrade();
    let twi_port = p.SERIAL0;

    let gas_channel = GAS_CHANNEL.init(Channel::new());
    let battery_status_channel = BATTERY_STATUS_CHANNEL.init(Channel::new());

    defmt::info!("Configuring TWIM...");
    let mut config = twim::Config::default();
    // Modify the i2c configuration fields if you dont have external i2c pullups
    config.sda_pullup = true;
    config.scl_pullup = true;
    let i2c = Twim::new(twi_port, Irqs, sda_pin, scl_pin, config);
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C_BUS.init(i2c_bus);
    let i2c_dev1 = I2cDevice::new(i2c_bus);

    spawner.spawn(npm1300_task(i2c_dev1,battery_status_channel.sender())).unwrap();

    spawner.spawn(charge_interrupt(host_pin)).unwrap();

    let i2c_dev2 = I2cDevice::new(i2c_bus);
    let i2c_dev3 = I2cDevice::new(i2c_bus);
    spawner.spawn(gas_sensor_task(i2c_dev2,i2c_dev3,gas_channel.sender())).unwrap();
    
    spawner.spawn(tictoctrigger(heater_pin,sensor_pin,led_pin1,led_pin2)).unwrap();

}
