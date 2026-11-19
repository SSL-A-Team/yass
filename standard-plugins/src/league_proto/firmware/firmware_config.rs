use serde::Deserialize;
use std::net::Ipv4Addr;

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct FirmwareConfig {
    pub address: Ipv4Addr,
    pub port: u16,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            address: Ipv4Addr::new(127, 0, 0, 1),
            port: 10301,
        }
    }
}
