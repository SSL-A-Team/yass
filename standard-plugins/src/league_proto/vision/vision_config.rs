use serde::Deserialize;
use std::net::Ipv4Addr;

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct VisionConfig {
    pub address: Ipv4Addr,
    pub port: u16,
    pub multicast_ttl: u32,
    pub detection_rate_hz: f64,
    pub geometry_period_s: f64,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            address: Ipv4Addr::new(224, 5, 23, 2),
            port: 10020,
            multicast_ttl: 1,
            detection_rate_hz: 60.0,
            geometry_period_s: 1.0,
        }
    }
}
