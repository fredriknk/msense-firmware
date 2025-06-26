#![no_std]
#![no_main]
extern crate tinyrlibc;


use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, peripherals::{self, SERIAL0}, twim::{self, Twim}
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
    },
    signal::Signal, 
    mutex::Mutex,
};
use static_cell::StaticCell;

use defmt::{info};

use {defmt_rtt as _, panic_probe as _};

mod modules;
use modules::modem::{setup_modem,
                    lte_trigger_loop};//, sensors, tasks, util};
use modules::sensors::gas::{gas_sensor_task,
                            tictoctrigger};
use modules::sensors::types::{EnvData, 
                            SingleSampleStorage, 
                            BatteryStatus,};
use modules::tasks::system::{watchdog_task,button_interrupt};
use modules::sensors::battery::{npm1300_task,charge_interrupt};
use modules::network::{lte_task, };
use modules::config::{
    DATASTORE_SIZE,
    TEMP_INTERVAL,
    NUM_SAMPLES_PER_AGGREGATION,
    NUM_SAMPLES_PER_BATTERY_READ,
    NUM_MINUTES_PER_SEND,
};

mod board_types;
use board_types::Pins;

// pick exactly one board at compile-time
#[cfg(feature = "rev_1_3_2")] mod board_rev_1_3_2;  #[cfg(feature = "rev_1_3_2")] use board_rev_1_3_2 as board;
#[cfg(feature = "rev_2_0"  )] mod board_rev_2_0;    #[cfg(feature = "rev_2_0"  )] use board_rev_2_0   as board;

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<SERIAL0>>> = StaticCell::new();

static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

static _LTE_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, EnvData> = Signal::new();
static LTE_SIGNAL: Signal<CriticalSectionRawMutex, LteTrigger> = Signal::new();


bind_interrupts!(struct Irqs {
    SERIAL0 => twim::InterruptHandler<peripherals::SERIAL0>;
});

enum BatteryTrigger {
    TriggerBatteryRead,
    StartCharging,
}

enum LteTrigger {
    _TriggerLteConnect,
    TriggerLteSend,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Starting Msense Firmware");
    let p = embassy_nrf::init(Default::default());

    let board_types::Board { pins, wdt, twi_port } = board::split(p);
    
    let Pins {
        led1:   led_pin1,
        led2:   led_pin2,
        heater: heater_pin,
        sensor: sensor_pin,
        host:   host_pin,
        button: button_pin,
        sda:    sda_pin,
        scl:    scl_pin,
    } = pins;

    let wdt_handle = modules::tasks::system::init_watchdog(wdt);
    spawner.spawn(watchdog_task(wdt_handle)).unwrap();

    let _ = setup_modem().await.unwrap();

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
    let i2c_dev1 = I2cDevice::new(i2c_bus);// NPM1300 battery monitor
    let i2c_dev2 = I2cDevice::new(i2c_bus);// BME680 gas sensor
    let i2c_dev3 = I2cDevice::new(i2c_bus);// ADS1115 ADC

    spawner.spawn(npm1300_task(i2c_dev1,battery_status_channel.sender())).unwrap();
    spawner.spawn(charge_interrupt(host_pin)).unwrap();
    spawner.spawn(gas_sensor_task(i2c_dev2,i2c_dev3,gas_channel.sender())).unwrap();
    spawner.spawn(tictoctrigger(heater_pin,sensor_pin,led_pin1,led_pin2)).unwrap();
    spawner.spawn(lte_trigger_loop()).unwrap();
    spawner.spawn(button_interrupt(button_pin)).unwrap();

   defmt::info!("All systems go!");
    spawner.spawn(lte_task(gas_channel.receiver(), battery_status_channel.receiver())).unwrap();

}
