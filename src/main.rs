#![no_std]
#![no_main]
extern crate tinyrlibc;


use embassy_executor::Spawner;
use embassy_nrf::{ peripherals::{ SERIAL0}, twim::{ Twim}
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
};
use static_cell::StaticCell;

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
use modules::bus;

mod board_types;

// pick exactly one board at compile-time
#[cfg(feature = "rev_1_3_2")] mod board_rev_1_3_2;  #[cfg(feature = "rev_1_3_2")] use board_rev_1_3_2 as board;
#[cfg(feature = "rev_2_0"  )] mod board_rev_2_0;    #[cfg(feature = "rev_2_0"  )] use board_rev_2_0   as board;


static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

static _LTE_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, EnvData> = Signal::new();
static LTE_SIGNAL: Signal<CriticalSectionRawMutex, LteTrigger> = Signal::new();

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

    let wdt_handle = modules::tasks::system::init_watchdog(wdt);
    spawner.spawn(watchdog_task(wdt_handle)).unwrap();

    let _ = setup_modem().await.unwrap();

    let gas_channel = GAS_CHANNEL.init(Channel::new());
    let battery_status_channel = BATTERY_STATUS_CHANNEL.init(Channel::new());
    
    let bus::I2cHandles { npm1300, bme680, ads1115 } =
        bus::init(twi_port, pins.sda, pins.scl);

    spawner.spawn(npm1300_task(npm1300,battery_status_channel.sender())).unwrap();
    spawner.spawn(charge_interrupt(pins.host)).unwrap();
    spawner.spawn(gas_sensor_task(bme680,ads1115,gas_channel.sender())).unwrap();
    spawner.spawn(tictoctrigger(pins.heater,pins.sensor,pins.led1,pins.led2)).unwrap();
    spawner.spawn(lte_trigger_loop()).unwrap();
    spawner.spawn(button_interrupt(pins.button)).unwrap();

   defmt::info!("All systems go!");
    spawner.spawn(lte_task(gas_channel.receiver(), battery_status_channel.receiver())).unwrap();

}
