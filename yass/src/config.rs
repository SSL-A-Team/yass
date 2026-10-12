use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Deserialize, Serialize)]
pub struct PluginConfig {
    pub lib: String,
    pub name: String,

    #[serde(flatten)]
    pub extra_keys: HashMap<String, serde_yaml::Value>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct TeamConfig {
    pub firmware: Option<PluginConfig>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct FieldConfig {
    pub length: f32,
    pub width: f32,
    pub boundary_width: f32,
    pub goal_width: f32
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub field: FieldConfig,
    pub blue_team: TeamConfig,
    pub yellow_team: TeamConfig,
    pub vision_plugins: Vec<PluginConfig>,
}
