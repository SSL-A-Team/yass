use schemars::{JsonSchema, Schema, SchemaGenerator, json_schema};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, net::Ipv4Addr};

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct PluginConfig {
    pub lib: String,
    pub name: String,

    #[serde(flatten)]
    #[schemars(schema_with = "flatten_arbitrary_yaml_schema")]
    pub extra_keys: HashMap<String, serde_yaml::Value>,
}

fn flatten_arbitrary_yaml_schema(_gen: &mut SchemaGenerator) -> Schema {
    json_schema!({
        "description": "Defines a plugin. Additional properties will be passed through to the plugin.",
        "additionalProperties": true,
    })
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
#[serde(default)]
pub struct RobotConfig {
    pub radius: f32,
    pub height: f32,
    pub mouth_width: f32,
    pub mass: f32,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            radius: 0.09,
            height: 0.18,
            mouth_width: 0.08,
            mass: 1.0, // TODO: what's a reasonable default for this?
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct TeamConfig {
    pub firmware: Option<PluginConfig>,
    #[serde(default)]
    pub robot_props: RobotConfig,
    #[serde(default = "default_robot_count")]
    pub robot_count: u8,
}

fn default_robot_count() -> u8 {
    6
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
#[serde(default)]
pub struct FieldConfig {
    pub length: f32,
    pub width: f32,
    pub boundary_width: f32,
    pub goal_width: f32,
    pub goal_depth: f32,
    pub penalty_area_depth: f32,
    pub penalty_area_width: f32,
    pub center_circle_radius: f32,
    pub line_thickness: f32,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            length: 9.0,
            width: 6.0,
            boundary_width: 0.3,
            goal_width: 1.0,
            goal_depth: 0.16,
            penalty_area_depth: 1.0,
            penalty_area_width: 2.0,
            center_circle_radius: 0.5,
            line_thickness: 0.01,
        }
    }
}

impl From<&FieldConfig> for yass_corelib::FieldDimensions {
    fn from(field: &FieldConfig) -> Self {
        Self {
            length: field.length,
            width: field.width,
            boundary_width: field.boundary_width,
            goal_width: field.goal_width,
            goal_depth: field.goal_depth,
            penalty_area_depth: field.penalty_area_depth,
            penalty_area_width: field.penalty_area_width,
            center_circle_radius: field.center_circle_radius,
            line_thickness: field.line_thickness,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
#[serde(default)]
pub struct ControlConfig {
    pub ip: Ipv4Addr,
    pub port: u16,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            ip: Ipv4Addr::new(127, 0, 0, 1),
            port: 10300,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct Config {
    #[serde(default)]
    pub control: ControlConfig,
    #[serde(default)]
    pub field: FieldConfig,
    pub blue_team: TeamConfig,
    pub yellow_team: TeamConfig,
    pub vision_plugins: Vec<PluginConfig>,
}

mod default_configs {
    include!(concat!(env!("OUT_DIR"), "/default_configs.rs"));
}

pub fn default_config_names() -> impl Iterator<Item = &'static str> {
    default_configs::DEFAULT_CONFIGS.iter().map(|(name, _)| *name)
}

pub fn default_config_contents(name: &str) -> Option<&'static str> {
    default_configs::DEFAULT_CONFIGS
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, contents)| *contents)
}

pub fn load_config(name_or_path: &str) -> Result<Config, Box<dyn std::error::Error>> {
    let contents = match default_config_contents(name_or_path) {
        Some(contents) => contents.to_owned(),
        None => std::fs::read_to_string(name_or_path).map_err(|e| {
            format!(
                "Could not read config file '{name_or_path}': {e}. \
                 Expected a path or one of the default configs: {}",
                default_config_names().collect::<Vec<_>>().join(", ")
            )
        })?,
    };
    Ok(serde_yaml::from_str(&contents)
        .map_err(|e| format!("Failed to parse config '{name_or_path}': {e}"))?)
}

#[cfg(test)]
mod default_config_tests {
    use super::*;

    #[test]
    fn all_default_configs_parse() {
        assert!(default_config_names().next().is_some());
        for name in default_config_names() {
            load_config(name).unwrap_or_else(|e| panic!("Default config '{name}' failed: {e}"));
        }
    }
}
