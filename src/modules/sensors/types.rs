pub struct EnvData {
    /// Temperature in °C
    pub temperature: f32,
    /// Relative humidity in %
    pub humidity: f32,
    /// Pressure in hPa
    pub pressure: f32,
    /// counter
    pub bmecounter: u32,
    /// Gas resistance in Ohms
    pub gas_resistance: f32,
    /// Gas resistance counter
    pub gascounter: u32,
}
pub struct SingleSampleStorage {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
    pub gas_resistance: f32,
    pub timestamp: u64,
}

pub struct BatteryStatus{
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub battery_temperature: f32,
    pub timestamp: u64,
}