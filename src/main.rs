#![no_std]
#![no_main]

use embassy_executor::Spawner;
#[cfg(feature = "i2c")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::channel::Channel;

use {defmt_rtt as _, panic_probe as _};

mod modules;
use modules::modem::{setup_modem};
use modules::watchdog::{watchdog_task,init_watchdog};

#[cfg(feature = "sensors")]
use modules::sensors::gas::{gas_sensor_task,heater_timer,GAS_CHANNEL};
#[cfg(feature = "npm1300")]
use modules::sensors::battery::{npm1300_task,charge_interrupt,BATTERY_STATUS_CHANNEL};

#[cfg(feature = "devboard")]
use modules::devboard_channels::{
    GAS_CHANNEL,
    BATTERY_STATUS_CHANNEL,
    fake_gas_data_task,
    fake_battery_data_task,
    ledblink_task,
};

use modules::network::{lte_task, lte_trigger_loop,send_button};
#[cfg(feature = "i2c")]
use modules::bus;

use crate::modules::{
        config::{FW_REV,BUILD_UNIX,GIT_HASH}, error_log::{self}
    };


mod board_types;

// pick exactly one board at compile-time
#[cfg(feature = "rev_1_3_2")] mod board_rev_1_3_2;  #[cfg(feature = "rev_1_3_2")] use board_rev_1_3_2 as board;
#[cfg(feature = "rev_2_0"  )] mod board_rev_2_0;    #[cfg(feature = "rev_2_0"  )] use board_rev_2_0   as board;
#[cfg(feature = "devboard"  )] mod board_rev_devboard;    #[cfg(feature = "devboard"  )] use board_rev_devboard   as board;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let board_types::Board { 
        pins, 
        wdt, 
        #[cfg(feature = "i2c")]
        twi_port 
    } = board::split(p);
 
    

    #[cfg(feature = "watchdog")]
    spawner.spawn(watchdog_task(init_watchdog(wdt))).unwrap();
    
    error_log::init();
    log_param!(Status,      "Booting");
    defmt::info!("Starting up on board: {}", board::BOARD_NAME);
    log_param!(HwRev,      board::BOARD_NAME);
    defmt::info!("Firmware revision: {}", FW_REV);
    log_param!(FwRev,      FW_REV);
    defmt::info!("Build Unix timestamp: {}", BUILD_UNIX);
    log_param!(BuildUnix,   BUILD_UNIX);
    defmt::info!("Git hash: {}", GIT_HASH);
    log_param!(GitHash,    GIT_HASH);

    #[cfg(feature = "sensors")]
    let bus::I2cHandles { npm1300, bme680, ads1115 } =
        bus::init(twi_port, pins.sda, pins.scl);

    let _ = setup_modem().await.unwrap();

    let gas_channel = GAS_CHANNEL.init(Channel::new());
    let battery_status_channel = BATTERY_STATUS_CHANNEL.init(Channel::new());

    #[cfg(feature = "npm1300")]{
        spawner.spawn(npm1300_task(npm1300,battery_status_channel.sender())).unwrap();
        spawner.spawn(charge_interrupt(pins.host)).unwrap();
    }
    #[cfg(feature = "sensors")]{
        spawner.spawn(gas_sensor_task(bme680,ads1115,gas_channel.sender())).unwrap();
        spawner.spawn(heater_timer(pins.heater,pins.sensor,pins.led1,pins.led2)).unwrap();
    }
    #[cfg(feature = "devboard")]{
        spawner.spawn(fake_gas_data_task(gas_channel.sender())).unwrap();
        spawner.spawn(fake_battery_data_task(battery_status_channel.sender())).unwrap();
        spawner.spawn(ledblink_task(pins.led1, pins.led2)).unwrap();
    }

    spawner.spawn(lte_trigger_loop()).unwrap();
    spawner.spawn(send_button(pins.button)).unwrap();
    
    spawner.spawn(lte_task(
        gas_channel.receiver(),
        battery_status_channel.receiver(),
        error_log::receiver(),
    )).unwrap();

    log_param!(Status,      "Running");

}
