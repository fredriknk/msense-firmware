pub const HOST_ADDRESS: &str = env!("HOST_ADDRESS");
pub const TCP_PORT: u16 = 8443;
pub const CERT_CHAIN: u32 = 16842753; // Certificate chain ID in NRF

pub const DATASTORE_SIZE: usize = 44; // How many samples can be stored in the channel
pub const ERR_CAP: usize = 10; // How many error messages can be stored in the error log channel

pub const MS_PER_SAMPLE: u64 = 30000; // How many milliseconds between every gas sample
#[cfg(any(feature = "rev_1_3_2", feature = "rev_2_0"))]
pub const TEMP_INTERVAL: u64 = 2; // How many gas samples are taken between every temp reading
#[cfg(any(feature = "rev_1_3_2", feature = "rev_2_0"))]
pub const MS_FILAMENT_HEAT: u64 = 98; // How many milliseconds to heat the filament before taking a sample
#[cfg(any(feature = "rev_1_3_2", feature = "rev_2_0"))]
pub const MS_SENSOR_HEAT: u64 = 3; // How many milliseconds to heat the sensor before taking a sample
pub const NUM_SAMPLES_PER_AGGREGATION: u64 = 10; // How many samples are aggregated before sending to the channel

pub const NUM_SAMPLES_PER_BATTERY_READ: u64 = 120; // How many samples are taken before reading the battery

pub const NUM_MINUTES_PER_SEND: u64 = 55; // How many minutes between sending data to the server
pub const NUM_SECONDS_TRY_NETWORK: u64 = 180; // How many seconds to wait before network search times out
pub const LTE_WDT_MARGIN_SECONDS: u64 = 10 * 60; // LTE loop watchdog margin in seconds (only fires if loop stops running)
pub const LTE_WATCHDOG_TIMEOUT_SECONDS: u64 = NUM_MINUTES_PER_SEND * 60 + LTE_WDT_MARGIN_SECONDS;

pub const RX_SIZE: usize = 1024;
pub const TX_SIZE: usize = 2048;

pub const FW_REV: &str = env!("CARGO_PKG_VERSION");
pub const BUILD_UNIX: &str = env!("BUILD_UNIX");
pub const GIT_HASH: &str = env!("GIT_HASH");
