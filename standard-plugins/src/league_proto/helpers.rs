use serde::de::DeserializeOwned;
use std::ffi::{CStr, c_char};

pub fn parse_config<ConfigType: Default + DeserializeOwned>(
    config_raw: *const c_char,
) -> Result<ConfigType, Box<dyn std::error::Error>> {
    if config_raw.is_null() {
        return Ok(ConfigType::default());
    }
    let text = unsafe { CStr::from_ptr(config_raw) }.to_str()?;
    Ok(serde_yaml::from_str(text)?)
}
