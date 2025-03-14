#![no_std]
#![no_main]
extern crate tinyrlibc;

use core::{time, u64};

use core::error;
use core::net::SocketAddr;
use cortex_m::peripheral::NVIC;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::interrupt;
use embassy_time::Timer;
use embassy_nrf::pac;
use nrf_modem::ConnectionPreference;
use nrf_modem::SystemMode;
use nrf_modem::{MemoryLayout};

use minicbor::{Decoder, Encoder};
use minicbor::encode::write::Cursor;

use embassy_nrf::gpio::{Level, Output, OutputDrive};

use embassy_nrf::{
    bind_interrupts, gpio::{AnyPin, Input}, peripherals::{self, SERIAL0}, twim::{self, Twim}
};

use embassy_nrf::gpio::{
    Pin,
};
use embassy_time::{
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
    },
    signal::Signal, 
    mutex::Mutex,
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
use uuid::Uuid;

use defmt::{info};




const DATASTORE_SIZE: usize = 30;
const TEMP_INTERVAL: u64 = 2; // How many gas samples are taken between every temp reading
const NUM_SAMPLES_PER_AGGREGATION: u64 = 20;
const NUM_SAMPLES_PER_BATTERY_READ: u64 = 40;

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<SERIAL0>>> = StaticCell::new();

static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

static GAS_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, EnvData> = Signal::new();

static LTE_SIGNAL: Signal<CriticalSectionRawMutex, LteTrigger> = Signal::new();


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

enum LteTrigger {
    TriggerLteConnect,
    TriggerLteSend,
}

//get hostname from env file
const HOST_ADDRESS : &str = "iot.nekko.no";

unsafe extern "C" {
    unsafe static __start_ipc: u8;
    unsafe static __end_ipc: u8;
}

pub async fn setup_modem() -> Result<(), nrf_modem::Error> {
    fn configure_modem_non_secure() -> u32 {
        // The RAM memory space is divided into 32 regions of 8 KiB.
        // Set IPC RAM to nonsecure
        const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
        const RAM_START: u32 = 0x2000_0000; // 256kb
        let ipc_start: u32 = unsafe { &__start_ipc as *const u8 } as u32;
        let ipc_reg_offset = (ipc_start - RAM_START) / SPU_REGION_SIZE;
        let ipc_reg_count =
            (unsafe { &__end_ipc as *const u8 } as u32 - ipc_start) / SPU_REGION_SIZE;
        let spu = embassy_nrf::pac::SPU;
        let range = ipc_reg_offset..(ipc_reg_offset + ipc_reg_count);
        defmt::debug!("marking region as non secure: {}", range);
        for i in range {
            spu.ramregion(i as usize).perm().write(|w| {
                w.set_execute(true);
                w.set_write(true);
                w.set_read(true);
                w.set_secattr(false);
                w.set_lock(false);
            })
        }

        // Set regulator access registers to nonsecure
        spu.periphid(4).perm().write(|w| w.set_secattr(false));
        // Set clock and power access registers to nonsecure
        spu.periphid(5).perm().write(|w| w.set_secattr(false));
        // Set IPC access register to nonsecure
        spu.periphid(42).perm().write(|w| w.set_secattr(false));
        ipc_start
    }
    let ipc_start = configure_modem_non_secure();
    // Interrupt Handler for LTE related hardware. Defer straight to the library.
    #[interrupt]
    #[allow(non_snake_case)]
    fn IPC() {
        nrf_modem::ipc_irq_handler();
    }

    let mut cp = unwrap!(cortex_m::Peripherals::take());

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init_with_custom_layout(
        SystemMode {
            lte_support: true,
            lte_psm_support: true,
            nbiot_support: false,
            gnss_support: false,
            preference: ConnectionPreference::None,
        },
        MemoryLayout {
            base_address: ipc_start,
            tx_area_size: 0x2000,
            rx_area_size: 0x2000,
            trace_area_size: 0,
        },
    )
    .await?;
    Ok(())
}


#[embassy_executor::task]
async fn blink_task(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after_millis(10).await;
        led.set_low();
        Timer::after_millis(10000).await;
    }
}

#[derive(Debug)]
pub struct XMonitorData<'a> {
    pub reg_status: &'a str,
    pub full_name: Option<&'a str>,
    pub short_name: Option<&'a str>,
    pub plmn: Option<&'a str>,
    pub tac: Option<&'a str>,
    pub act: Option<&'a str>,
    pub band: Option<&'a str>,
    pub cell_id: Option<&'a str>,
    pub phys_cell_id: Option<&'a str>,
    pub earfcn: Option<&'a str>,
    pub rsrp: Option<&'a str>,
    pub snr: Option<&'a str>,
    pub edrx_value: Option<&'a str>,
    pub active_time: Option<&'a str>,
    pub periodic_tau_ext: Option<&'a str>,
}

/// Strips surrounding quotes if the string starts and ends with a double quote.
fn trim_quotes(s: &str) -> &str {
    let s = s.trim();
    if s.len() >= 2 && s.starts_with('"') && s.ends_with('"') {
        &s[1..s.len() - 1]
    } else {
        s
    }
}

pub fn strip_at<'a>(prefix: &'a str, line: &'a str) -> Option<&'a str> {
    let line = &line.strip_prefix(prefix)?;
    let line = &line.strip_suffix("\r\nOK\r\n")?;
    let line = &line.trim();
    Some(line)
}

/// Attempt to parse a line containing `%XMONITOR data `.
/// Returns `None` if the prefix isn't found or if parsing fails in an unexpected way.
pub fn parse_xmonitor_response(line: &str) -> Option<XMonitorData> {
    let prefix = "%XMONITOR:";
    let line = line.trim();

    // Find the prefix.
    let start_idx = line.find(prefix)?;
    // Slice after "%XMONITOR:" to get the actual CSV portion.
    let after_prefix = &line[start_idx + prefix.len()..].trim();

    // Split on commas. Each entry might be quoted, so we'll trim them below.
    let mut split_iter = line.split(',').map(|s| s.trim());

    // The first field is the mandatory <reg_status>.
    let reg_status_raw = split_iter.next()?;
    let reg_status = trim_quotes(reg_status_raw);

    // All remaining fields are optional, so we capture them in sequence
    // as `Option<&str>`, trimming quotes where present.
    let full_name = split_iter.next().map(trim_quotes);
    let short_name = split_iter.next().map(trim_quotes);
    let plmn = split_iter.next().map(trim_quotes);
    let tac = split_iter.next().map(trim_quotes);
    let act = split_iter.next().map(trim_quotes);
    let band = split_iter.next().map(trim_quotes);
    let cell_id = split_iter.next().map(trim_quotes);
    let phys_cell_id = split_iter.next().map(trim_quotes);
    let earfcn = split_iter.next().map(trim_quotes);
    let rsrp = split_iter.next().map(trim_quotes);
    let snr = split_iter.next().map(trim_quotes);
    let edrx_value = split_iter.next().map(trim_quotes);
    let active_time = split_iter.next().map(trim_quotes);
    let periodic_tau_ext = split_iter.next().map(trim_quotes);

    Some(XMonitorData {
        reg_status,
        full_name,
        short_name,
        plmn,
        tac,
        act,
        band,
        cell_id,
        phys_cell_id,
        earfcn,
        rsrp,
        snr,
        edrx_value,
        active_time,
        periodic_tau_ext,
    })
}

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
                    defmt::info!("Battery Data sent to channel");
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
        sensor_data.gas_resistance += gas_res;
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
            defmt::info!("Sending data to channel...");
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
async fn lte_trigger_loop(     
) {
    loop{
        LTE_SIGNAL.signal(LteTrigger::TriggerLteSend);
        Timer::after_secs(600).await;
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
    defmt::info!("Starting Msense Firmware");
    let p = embassy_nrf::init(Default::default());

    let _ = setup_modem().await.unwrap();
    let response = nrf_modem::send_at::<64>("AT+CGMR").await.unwrap();
    defmt::info!("AT+CGMR response: {:?}", response.as_str());

    let response = nrf_modem::send_at::<128>("AT%XMODEMUUID").await.unwrap();
    defmt::info!("AT+CGMR response: {:?}", response.as_str());
    let uuid_bytes: [u8; 16];

    if let Some(uuid) = strip_at("%XMODEMUUID:", response.as_str()) {
        defmt::info!("UUID: {:?}", uuid);
        uuid_bytes = Uuid::parse_str(&uuid)
            .expect("Invalid UUID")
            .as_bytes()
            .clone();
        defmt::info!("UUID Bytes: {:?}", uuid_bytes);
    } else {
        uuid_bytes = [0u8; 16];
        defmt::warn!("Prefix not found in response.");
    }
    
    
    let led_pin1 = p.P0_06.degrade();
    let led_pin2 = p.P0_05.degrade();

    let heater_pin = p.P0_31.degrade();
    let sensor_pin = p.P0_27.degrade();

    let host_pin = p.P0_04.degrade();

    let sda_pin = p.P0_28.degrade();
    let scl_pin = p.P0_29.degrade();
    let twi_port = p.SERIAL0;

    let gas_channel = GAS_CHANNEL.init(Channel::new());
    let gas_receiver = gas_channel.receiver();
    let battery_status_channel = BATTERY_STATUS_CHANNEL.init(Channel::new());
    let battery_receiver = battery_status_channel.receiver();

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
    spawner.spawn(lte_trigger_loop()).unwrap();

    info!("All systems go!");
    
    loop { 
        let lte_sig = LTE_SIGNAL.wait().await;
        match lte_sig {
            LteTrigger::TriggerLteConnect => {
            }
            LteTrigger::TriggerLteSend => {
                info!("Connecting...");
                match nrf_modem::TlsStream::connect(
                    HOST_ADDRESS,
                    8443,
                    nrf_modem::PeerVerification::Enabled,
                    &[16842753],
                    Some(&[nrf_modem::CipherSuite::TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256]),
                    1 as u32,
                ).await{
                    Ok(stream) => {
                    defmt::info!("Connected to host");
                    
                    let response = nrf_modem::send_at::<128>("AT%XMONITOR").await.unwrap();
                    defmt::info!("AT%XMONITOR: {:?}", response.as_str());
                    let xmon = parse_xmonitor_response(&response.as_str()).unwrap();

                    let starttime = Instant::now();
                    let gas_measurements = gas_receiver.len();
                    let battery_measurements = battery_receiver.len();
                    defmt::info!("Gas Queue length: {},battqueue {}", gas_measurements,battery_measurements);
                    //iterate over num gas_measurements
                    let mut buffer = [0u8; 2048];
                    let cursor = Cursor::new(&mut buffer[..]);
                    let mut encoder = Encoder::new(cursor);

                    let time = Instant::now().as_millis();
                
                    // Create a mutable array for data
                    let mut data = [0u8; 16 + 8]; // adjust to your length if necessary
                
                    let _ = encoder.begin_map();
                    let _ = encoder.str("d");
                    let _ = encoder.map(7);
                    let _ = encoder.str("UU");
                    let _ = encoder.bytes(&uuid_bytes);
                    let _ = encoder.str("C");
                    let _ = encoder.u64(time);
                    let _ = encoder.str("RS");
                    let _ = encoder.int(xmon.rsrp.unwrap().parse::<i32>().unwrap().into());
                    let _ = encoder.str("SN");
                    let _ = encoder.int(xmon.snr.unwrap().parse::<i32>().unwrap().into());
                    let _ = encoder.str("PL");
                    let _ = encoder.str(xmon.plmn.unwrap());
                    let _ = encoder.str("TA");
                    let _ = encoder.str(xmon.tac.unwrap());
                    let _ = encoder.str("CI");
                    let _ = encoder.str(xmon.cell_id.unwrap());

                    
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
                                let _ = encoder.u64(gas_data_unwrap.timestamp);
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
                                let _ = encoder.u64(battery_data.timestamp);
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

                    stream
                        .write(encoded_data)
                        .await
                        .unwrap();
        
                    let mut buffer = [0; 1024];
                    let received = stream
                        .receive(&mut buffer)
                        .await
                        .unwrap();
        
                    defmt::info!("Received: {:?}", core::str::from_utf8(received).unwrap());
        
                    stream
                        .deactivate().await.unwrap();
                    },
                    Err(_e) => {
                        defmt::info!("Error connecting to host ");
                    }
                }

                Timer::after_secs(60*55).await;
            }
        }
    }
}
