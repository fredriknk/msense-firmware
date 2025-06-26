pub const DATASTORE_SIZE: usize = 60;
pub const TEMP_INTERVAL: u64 = 1; // How many gas samples are taken between every temp reading
pub const NUM_SAMPLES_PER_AGGREGATION: u64 = 2; // How many samples are aggregated before sending to the channel
pub const NUM_SAMPLES_PER_BATTERY_READ: u64 = 20; // How many samples are taken before reading the battery
pub const NUM_MINUTES_PER_SEND: u64 = 30; // How many minutes between sending data to the server
pub const HOST_ADDRESS : &str = "iot.nekko.no";