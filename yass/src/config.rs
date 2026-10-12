use schemars::{JsonSchema, Schema, SchemaGenerator, json_schema};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct PluginConfig {
    pub lib: String,
    pub name: String,

    #[serde(flatten)]
    #[schemars(schema_with="flatten_arbitrary_yaml_schema")]
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
            mass: 1.0,  // TODO: what's a reasonable default for this?
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
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            length: 9.0,
            width: 6.0,
            boundary_width: 0.3,
            goal_width: 1.0,
            goal_depth: 0.16,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct Config {
    #[serde(default)]
    pub field: FieldConfig,
    pub blue_team: TeamConfig,
    pub yellow_team: TeamConfig,
    pub vision_plugins: Vec<PluginConfig>,
}
