#![no_std]
#![no_main]

//! Example demonstrating control of the NPM1300 PMIC's leds

use core::{time, u64};

use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, gpio::{AnyPin, Input}, peripherals::{self, SERIAL0}, twim::{self, Twim}
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

use core::mem::MaybeUninit;
use core::net::IpAddr;
use core::slice;
use core::convert::TryInto;

use defmt::{info, unwrap, warn};
use embassy_net::{Ipv4Cidr, Stack, StackResources,dns};
use embassy_net_nrf91::context::Status;
use embassy_net_nrf91::{context, Runner, State, TraceBuffer, TraceReader,Control};
use embassy_nrf::{interrupt};
use embassy_time::{Duration};
use embedded_io_async::Write;
use heapless::Vec;

use at_commands::builder::CommandBuilder;
use at_commands::parser::CommandParser;

use uuid::Uuid;

use sha_256::Sha256;



const DATASTORE_SIZE: usize = 30;
const TEMP_INTERVAL: u64 = 2; // How many gas samples are taken between every temp reading
const NUM_SAMPLES_PER_AGGREGATION: u64 = 12;
const NUM_SAMPLES_PER_BATTERY_READ: u64 = 24;

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim<SERIAL0>>> = StaticCell::new();

static GAS_CHANNEL: StaticCell<Channel<NoopRawMutex, SingleSampleStorage, DATASTORE_SIZE>> = StaticCell::new();
static BATTERY_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, BatteryStatus, DATASTORE_SIZE>> = StaticCell::new();

static GAS_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryTrigger> = Signal::new();

static GAS_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, EnvData> = Signal::new();


#[interrupt]
fn IPC() {
    embassy_net_nrf91::on_ipc_irq();
}

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

#[embassy_executor::task]
async fn modem_task(runner: Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, embassy_net_nrf91::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn control_task(
    control: &'static context::Control<'static>,
    config: context::Config<'static>,
    stack: Stack<'static>,
) {
    defmt::info!("Configuring modem...");
    unwrap!(control.configure(&config).await);
    unwrap!(
        control
            .run(|status| {
                stack.set_config_v4(status_to_config(status));
            })
            .await
    );
}

fn status_to_config(status: &Status) -> embassy_net::ConfigV4 {
    let Some(IpAddr::V4(addr)) = status.ip else {
        panic!("Unexpected IP address");
    };

    let gateway = match status.gateway {
        Some(IpAddr::V4(addr)) => Some(addr),
        _ => None,
    };

    let mut dns_servers = Vec::new();
    for dns in status.dns.iter() {
        if let IpAddr::V4(ip) = dns {
            unwrap!(dns_servers.push(*ip));
        }
    }

    embassy_net::ConfigV4::Static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(addr, 32),
        gateway,
        dns_servers,
    })
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

unsafe extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
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

/// Attempt to parse a line containing `%XMONITOR: ...`.
/// Returns `None` if the prefix isn't found or if parsing fails in an unexpected way.
pub fn parse_xmonitor_response(line: &str) -> Option<XMonitorData> {
    let prefix = "%XMONITOR:";
    let line = line.trim();

    // Find the prefix.
    let start_idx = line.find(prefix)?;
    // Slice after "%XMONITOR:" to get the actual CSV portion.
    let after_prefix = &line[start_idx + prefix.len()..].trim();

    // Split on commas. Each entry might be quoted, so we'll trim them below.
    let mut split_iter = after_prefix.split(',').map(|s| s.trim());

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

    info!("Hello World!");

    //unwrap!(spawner.spawn(blink_task(p.P0_06.degrade())));

    let ipc_mem = unsafe {
        let ipc_start = &__start_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_end = &__end_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_len = ipc_end.offset_from(ipc_start) as usize;
        slice::from_raw_parts_mut(ipc_start, ipc_len)
    };

    static STATE: StaticCell<State> = StaticCell::new();
    let (device, control, runner) =
        embassy_net_nrf91::new(STATE.init(State::new()), ipc_mem).await;

    unwrap!(spawner.spawn(modem_task(runner)));

    let config = embassy_net::Config::default();

    // Generate "random" seed. nRF91 has no RNG, TODO figure out something...
    let seed = 123456;

    // Init network stack
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::<2>::new()), seed);

    unwrap!(spawner.spawn(net_task(runner)));

    static CONTROL: StaticCell<context::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(context::Control::new(control, 0).await);

    unwrap!(spawner.spawn(control_task(
        control,
        context::Config {
            apn: b"iot.nat.es",
            auth_prot: context::AuthProt::Pap,
            auth: Some((b"orange", b"orange")),
            pin: None

        },
        stack
    )));

    stack.wait_config_up().await;

    let mut rx_buffer= [0u8; 4096];
    let mut tx_buffer= [0u8; 4096];

    let mut cmd: [u8; 256] = [0; 256];
    let mut buf: [u8; 256] = [0; 256];

    let op = CommandBuilder::create_set(&mut cmd, true)
        .named("%XMODEMUUID")
        .finish()
        .map_err(|_| context::Error::BufferTooSmall);
    let n = control.at_command(op.expect("REASON"), &mut buf).await;
    //print buffer as string
    defmt::info!("uuid: {=[u8]:a}", &buf[..n]);
    let (uuid,) = CommandParser::parse(&buf[..n-1]).expect_identifier(b"%XMODEMUUID:").expect_raw_string().expect_identifier(b"\r\nOK\r\n").finish().unwrap();
    defmt::info!("UUID: {}", &uuid);
    
    let uuid_bytes = Uuid::parse_str(&uuid).expect("Invalid UUID");
    defmt::info!("UUID: {:?}", uuid_bytes.as_bytes());

    let mut address = embassy_net::Stack::dns_query(&stack, <YOURSEVER>,embassy_net::dns::DnsQueryType::A).await;
    info!("DNS query: {:?}", address);
    // Resolve IP address
    let remote_ip = address.unwrap()[0];
    let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    let mut sha256: Sha256 = Sha256::new();

    let mut timestamp_offset: u64 = 0;
    
    loop { 
        let lte_sig = LTE_SIGNAL.wait().await;

        if socket.state()== embassy_net::tcp::State::Closed {
            info!("Socket is note open, connecting...");
            match socket.connect((host_addr, 51237)).await {
                Ok(()) => {
                    info!("Connected to {:?}", socket.remote_endpoint());
                }
                Err(e) => {
                    warn!("connect error: {:?}", e);
                    Timer::after_secs(3).await;
                    continue;
                }
            }
        }
        else {
            info!("Socket is {:?} continuing", socket.state());
        }

        match lte_sig {
            LteTrigger::TlsHandshake => {


            }
            LteTrigger::TriggerLteSend => {
                info!("Connecting...");
                let host_addr =  remote_ip;

                let at_cmd = b"AT%XMONITOR\r\n";
                let mut response_buffer = [0u8; 128];
                let response_len = control.at_command(at_cmd, &mut response_buffer).await;
                // Handle response (bytes received)
                if response_len > 0 {
                    // Convert bytes to a string slice.
                    if let Ok(resp_str) = core::str::from_utf8(&response_buffer[..response_len]) {
                        // Use the parse function:
                        if let Some(parsed_data) = parse_xmonitor_response(resp_str) {
                            // Now you can do something with the parsed data
                            // e.g.
                            info!(
                                "XMONITOR: reg_status={:?}, rsrp={:?}, snr={:?}, plmn={:?}, edrx_value={:?}, active_time={:?}, periodic_tau_ext={:?}",
                                parsed_data.reg_status,
                                parsed_data.rsrp,
                                parsed_data.snr,
                                parsed_data.plmn,
                                parsed_data.edrx_value,
                                parsed_data.active_time,
                                parsed_data.periodic_tau_ext,
                            );
                        } else {
                            info!("Received response, but not a valid %XMONITOR line or parse error: {}", resp_str);
                        }
                    } else {
                        info!("Received invalid UTF-8 data: {:?}", &response_buffer[..response_len]);
                    }
                }

                //let msg = b"Hello world!\n";
                
                let token: &[u8; 16] = b"1234567890abcdef"; 
                let time = Instant::now().as_millis();

                // Create a mutable array for data
                let mut data = [0u8; 16 + 8]; // adjust to your length if necessary
                data[..16].copy_from_slice(token);
                data[16..].copy_from_slice(&time.to_be_bytes()); // or to_le_bytes(), whichever fits your usage

                let hash: [u8; 32] = sha256.digest(&data);

                let mut msg:[u8;40] = [0u8; 8 + 32]; // adjust to your length if necessary
                msg[..8].copy_from_slice(&time.to_be_bytes()); // or to_le_bytes(), whichever fits your usage
                msg[8..].copy_from_slice(&hash);

                if let Err(e) = socket.write_all(&msg).await {
                    warn!("write error: {:?}", e);
                    break;
                }
                //info!("txd: {}", core::str::from_utf8(msg).unwrap());

                    // Buffer to store incoming data
                let mut read_buffer = [0u8; 1024];

                // Read data from the server
                match socket.read(&mut read_buffer).await {
                    Ok(0) => {
                        // Connection was closed by the server
                        info!("Connection closed by server.");
                    }
                    Ok(n) => {
                        // Successfully read 'n' bytes
                        info!("Received: {}", core::str::from_utf8(&read_buffer[..n]).unwrap());
                    }
                    Err(e) => {
                        // An error occurred while reading
                        warn!("read error: {:?}", e);
                    }
                }
                
                Timer::after_secs(60*55).await;
            }
        }
    }

}
