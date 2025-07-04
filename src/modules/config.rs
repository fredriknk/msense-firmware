pub const HOST_ADDRESS : &str = "iot.nekko.no"; //Server address
pub const TCP_PORT: u16 = 8443; //Server port
pub const CERT_CHAIN: u32 = 16842753; // Certificate chain ID in NRF

pub const DATASTORE_SIZE: usize = 30; // How many samples can be stored in the channel
pub const ERR_CAP: usize = 10; // How many error messages can be stored in the error log channel
pub const TEMP_INTERVAL: u64 = 1; // How many gas samples are taken between every temp reading
pub const MS_PER_SAMPLE: u64 = 30000; // How many milliseconds between every sample
pub const MS_FILAMENT_HEAT: u64 = 98; // How many milliseconds to heat the filament before taking a sample
pub const MS_SENSOR_HEAT: u64 = 3; // How many milliseconds to heat the sensor before taking a sample
pub const NUM_SAMPLES_PER_AGGREGATION: u64 = 10; // How many samples are aggregated before sending to the channel
pub const NUM_SAMPLES_PER_BATTERY_READ: u64 = 30; // How many samples are taken before reading the battery

pub const NUM_MINUTES_PER_SEND: u64 = 55; // How many minutes between sending data to the server
pub const NUM_SECONDS_TRY_NETWORK: u64 = 180; // How many seconds to wait before network search times out

pub const RX_SIZE: usize = 1024;
pub const TX_SIZE: usize = 2048;

pub const FW_REV: &str = env!("CARGO_PKG_VERSION");
pub const BUILD_UNIX: &str = env!("BUILD_UNIX");
pub const GIT_HASH: &str = env!("GIT_HASH");
