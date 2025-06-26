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

use static_cell::StaticCell;

use embassy_time::Instant;
use defmt::Debug2Format;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;


use super::super::config::DATASTORE_SIZE;

use embassy_time::{Timer, };
use embassy_nrf::{peripherals::SERIAL0, twim::Twim,
    gpio::{AnyPin, Input}};

pub static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

pub static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();
pub enum BatteryTrigger {
    TriggerBatteryRead,
    StartCharging,
}

pub struct BatteryStatus{
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub battery_temperature: f32,
    pub timestamp: u64,
}

#[embassy_executor::task]
pub async fn npm1300_task(
        i2c_dev: I2cDevice<'static, NoopRawMutex, Twim<'static, SERIAL0>>,
        sender: Sender<'static, NoopRawMutex, BatteryStatus, DATASTORE_SIZE>,
    ) {
    
    let mut npm1300 = NPM1300::new(i2c_dev, embassy_time::Delay);
    defmt::debug!("Configuring LED modes...");
    let _ = npm1300.configure_led0_mode(LedMode::Host).await;
    defmt::debug!("Configured LED0 mode to Host");
    let _ = npm1300.configure_led1_mode(LedMode::Charging).await;
    defmt::debug!("Configured LED1 mode to Charging");
    let _ = npm1300.configure_led2_mode(LedMode::ChargingError).await;
    defmt::debug!("Configured LED2 mode to ChargingError");

    let _ = npm1300.set_vbus_in_current_limit(VbusInCurrentLimit::MA1000).await;
    
    defmt::debug!("Configuring NTC Resistor...");
    let _ = npm1300.configure_ntc_resistance(NtcThermistorType::Ntc10K, Some(3380.0)).await;
    let _ = npm1300.use_ntc_measurements().await;

    defmt::debug!("Configuring Charging...");
    let _ = npm1300.set_charger_current(500).await;
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
                defmt::debug!("Triggering Battery Measurement");
                
                let vbat_voltage = match  npm1300.measure_vbat().await {
                    Ok(voltage) => {
                        defmt::debug!("VBAT Voltage: {:?}", voltage);
                        voltage
                    }
                    Err(e) => {
                        defmt::error!("Failed to measure VBAT: {}", Debug2Format(&e));
                        -9999.0
                    }
                };

                let _ = npm1300.measure_ntc().await;

                let ntc_temp = match npm1300.get_ntc_measurement_result().await {
                    Ok(temp) => temp,
                    Err(e) => {
                        defmt::error!("Failed to get NTC measurement result: {}", Debug2Format(&e));
                        -9999.0
                    }
                };

                let die_temp = match npm1300.measure_die_temperature().await {
                    Ok(temp) => temp,
                    Err(e) => {
                        defmt::error!("Failed to measure die temperature: {}", Debug2Format(&e));
                        -9999.0
                    }
                };

                let ibat_current = match npm1300.measure_ibat().await {
                    Ok(current) => current,
                    Err(e) => {
                        defmt::error!("Failed to measure IBAT: {}", Debug2Format(&e));
                        9999.0
                    }
                };

                match npm1300.get_charger_status().await {
                    Ok(status) => {
                        defmt::debug!("Charger Status: {:?}", status);
                    },
                    Err(e) => {
                        defmt::error!("Failed to get charger status: {}", Debug2Format(&e));
                    }
                };
                
                defmt::debug!("NTC Temp: {:?}, Die Temp: {:?}, VBAT Voltage: {:?}, IBAT Current: {:?}", ntc_temp,die_temp,vbat_voltage,ibat_current);

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
                    defmt::debug!("Battery Data sent to channel");
                }
            },

            BatteryTrigger::StartCharging => {
                defmt::info!("Charging Battery");
                match npm1300.set_vbus_in_current_limit(VbusInCurrentLimit::MA1000).await{
                    Ok(_) => {
                        defmt::info!("Set VBUS current limit to 1000mA");
                    },
                    Err(e) => {
                        defmt::error!("Failed to set VBUS current limit: {}", Debug2Format(&e));
                    }
                }
                BATTERY_SIGNAL.signal(BatteryTrigger::TriggerBatteryRead);
            }
        }
    };
}

#[embassy_executor::task]
pub async fn charge_interrupt(host_pin: AnyPin) {
    let mut host = Input::new(host_pin, embassy_nrf::gpio::Pull::Down);
    loop {
        host.wait_for_rising_edge().await;
        BATTERY_SIGNAL.signal(BatteryTrigger::StartCharging);
        Timer::after_millis(2000).await;
    }
}