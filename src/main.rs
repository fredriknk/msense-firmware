#![no_std]
#![no_main]

//! Example demonstrating control of the NPM1300 PMIC's leds

use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, gpio::AnyPin, peripherals::{self, SERIAL0}, twim::{self, Twim}
};

use embassy_nrf::gpio::{Level, Output, OutputDrive, Pin};
use embassy_time::Timer;

use embassy_embedded_hal::shared_bus::asynch::i2c::{I2cDevice};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use static_cell::StaticCell;

use embedded_ads111x::{ADS111x,ADS111xConfig,InputMultiplexer,ProgramableGainAmplifier,DataRate};
use bosch_bme680::{AsyncBme680,Configuration,DeviceAddress,Oversampling,IIRFilter};
use {defmt_rtt as _, panic_probe as _};

use npm1300_rs::{
    leds::LedMode,
    NtcThermistorType,
    charger::{
        ChargerTerminationVoltage,
        DischargeCurrentLimit,
        ChargerTerminationCurrentLevelSelect,
    },
    sysreg::{
        VbusInCurrentLimit,
    },
    NPM1300,
};

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<SERIAL0>>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    SERIAL0 => twim::InterruptHandler<peripherals::SERIAL0>;
});

#[embassy_executor::task]
async fn npm1300_task(i2c_dev: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>) {
    
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
    let _ = npm1300.set_termination_current_level(ChargerTerminationCurrentLevelSelect::SEL10).await;
    let _ = npm1300.set_normal_temperature_termination_voltage(ChargerTerminationVoltage::V4_20).await;
    let _ = npm1300.set_warm_temperature_termination_voltage(ChargerTerminationVoltage::V4_10).await;
    let _ = npm1300.set_discharge_current_limit(DischargeCurrentLimit::High).await;
    let _ = npm1300.enable_battery_charging().await;
    let _ = npm1300.enable_battery_recharge().await;
    let _ = npm1300.configure_ibat_measurement(true).await;

    loop {
        let _ = npm1300.enable_led0().await;
        let vbat_voltage = npm1300.measure_vbat().await.unwrap();
        let _ = npm1300.measure_ntc().await;
        let ntc_temp = npm1300.get_ntc_measurement_result().await.unwrap();
        let die_temp = npm1300.measure_die_temperature().await.unwrap();
        let ibat_current = npm1300.measure_ibat().await.unwrap();
        defmt::info!("NTC Temp: {:?}", ntc_temp);
        defmt::info!("Die Temp: {:?}", die_temp);
        defmt::info!("VBAT Voltage: {:?}", vbat_voltage);
        defmt::info!("IBAT Current: {:?}", ibat_current);
        let _ = npm1300.disable_led0().await;
        Timer::after_millis(10000).await;
    };
}

#[embassy_executor::task]
async fn gas_sensor_task(
        heater_pin: AnyPin, 
        sensor_pin: AnyPin,
        led_pin1:AnyPin,
        led_pin2:AnyPin,
        i2c_dev: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>
    ) {

    let mut led1 = Output::new(led_pin1, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(led_pin2, Level::Low, OutputDrive::Standard);
    let mut power = Output::new(heater_pin, Level::Low, OutputDrive::Standard);
    let mut sensor = Output::new(sensor_pin, Level::Low, OutputDrive::Standard);

    let adsconfig = ADS111xConfig::default()
        .mux(InputMultiplexer::AIN0GND)
        .dr(DataRate::SPS860)
        .pga(ProgramableGainAmplifier::V2_048);

    let mut adc = match ADS111x::new(i2c_dev, 0x48u8, adsconfig){
        Err(e) => panic!("Error {:?}", e),
        Ok(x) => x,
    };

    let _ = adc.write_config(None).await;

    loop {
        led1.set_high();
        power.set_high();
        Timer::after_millis(98).await;

        led2.set_high();
        sensor.set_high();
        let volt =  adc.read_single_voltage(None).await.unwrap();

        sensor.set_low();
        power.set_low();
        led1.set_low();
        led2.set_low();

        defmt::debug!("Voltage: {:?}", volt);
        Timer::after_millis(29900).await;
    }

}

#[embassy_executor::task]   
async fn bme680_task(i2c_dev: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>)
 {

    let bme680config =  Configuration::builder()
        .temperature_oversampling(Oversampling::By2)
        .pressure_oversampling(Oversampling::By16)
        .humidity_oversampling(Oversampling::By1)
        .filter(IIRFilter::Coeff1)
        .gas_config(None)
        .build();

    
    let mut bme680 = AsyncBme680::new(i2c_dev, DeviceAddress::Primary, embassy_time::Delay,25);

    let _ = bme680.set_configuration(&bme680config).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Starting NPM1300 example");
    let p = embassy_nrf::init(Default::default());
    
    let led_pin1 = p.P0_06.degrade();
    let led_pin2 = p.P0_05.degrade();

    let heater_pin = p.P0_27.degrade();
    let sensor_pin = p.P0_31.degrade();

    let sda_pin = p.P0_28.degrade();
    let scl_pin = p.P0_29.degrade();
    let twi_port = p.SERIAL0;

    let mut config = twim::Config::default();
    // Modify the i2c configuration fields if you dont have external i2c pullups
    config.sda_pullup = true;
    config.scl_pullup = true;

    defmt::info!("Configuring TWIM...");
    
    let i2c = Twim::new(twi_port, Irqs, sda_pin, scl_pin, config);
    let i2c_bus = Mutex::new(i2c);

    let i2c_bus = I2C_BUS.init(i2c_bus);

    let i2c_dev1 = I2cDevice::new(i2c_bus);
    spawner.spawn(npm1300_task(i2c_dev1)).unwrap();

    let i2c_dev2 = I2cDevice::new(i2c_bus);
    spawner.spawn(gas_sensor_task(heater_pin,sensor_pin,led_pin1,led_pin2,i2c_dev2)).unwrap();
    
    let i2c_dev3 = I2cDevice::new(i2c_bus);
    spawner.spawn(bme680_task(i2c_dev3)).unwrap();
    



}
