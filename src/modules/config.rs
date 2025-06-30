pub const HOST_ADDRESS : &str = "iot.nekko.no"; //Server address
pub const TCP_PORT: u16 = 8443; //Server port
pub const CERT_CHAIN: u32 = 16842753; // Certificate chain ID in NRF

pub const DATASTORE_SIZE: usize = 64; // How many samples can be stored in the channel
pub const ERR_CAP: usize = 64; // How many error messages can be stored in the error log channel
pub const TEMP_INTERVAL: u64 = 1; // How many gas samples are taken between every temp reading
pub const NUM_SAMPLES_PER_AGGREGATION: u64 = 2; // How many samples are aggregated before sending to the channel
pub const NUM_SAMPLES_PER_BATTERY_READ: u64 = 20; // How many samples are taken before reading the battery
pub const NUM_MINUTES_PER_SEND: u64 = 30; // How many minutes between sending data to the server

pub const RX_SIZE: usize = 1024;
pub const TX_SIZE: usize = 2048;