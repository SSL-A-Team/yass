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
            mass: 2.6,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
pub struct TeamConfig {
    #[serde(default)]
    pub firmware: Option<PluginConfig>,
    #[serde(default)]
    pub robot_props: RobotConfig,
    #[serde(default = "default_robot_count")]
    pub robot_count: u8,
}

fn default_robot_count() -> u8 {
    6
}

impl Default for TeamConfig {
    fn default() -> Self {
        Self {
            firmware: None,
            robot_props: RobotConfig::default(),
            robot_count: default_robot_count(),
        }
    }
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

/// Physical properties of the ball.
#[derive(Debug, Deserialize, Serialize, Clone, JsonSchema)]
#[serde(default)]
pub struct BallConfig {
    /// Ball radius (m).
    pub radius: f32,
    /// Ball mass (kg).
    pub mass: f32,
    /// Coefficient of restitution for ball collisions.
    pub restitution: f32,
    /// Constant deceleration magnitude while the ball slides (m/s^2).
    pub sliding_deceleration: f32,
    /// Constant deceleration magnitude while the ball rolls (m/s^2).
    pub rolling_deceleration: f32,
    /// Ratio of the speed at which sliding turns into rolling to the initial kick speed.
    /// Must be in (0, 1). Theory for a solid sphere gives 5/7.
    pub sliding_to_rolling_ratio: f32,
}

impl Default for BallConfig {
    fn default() -> Self {
        Self {
            radius: 0.0213,
            mass: 0.04593,
            restitution: 0.65,
            sliding_deceleration: 2.5,
            rolling_deceleration: 0.3,
            sliding_to_rolling_ratio: 0.6,
        }
    }
}

impl BallConfig {
    pub fn validate(&self) -> Result<(), String> {
        let positive = [("radius", self.radius), ("mass", self.mass)];
        for (name, value) in positive {
            if value.is_nan() || value <= 0.0 {
                return Err(format!("ball.{name} must be positive, got {value}"));
            }
        }
        let non_negative = [
            ("restitution", self.restitution),
            ("sliding_deceleration", self.sliding_deceleration),
            ("rolling_deceleration", self.rolling_deceleration),
        ];
        for (name, value) in non_negative {
            if value.is_nan() || value < 0.0 {
                return Err(format!("ball.{name} must be non-negative, got {value}"));
            }
        }
        if self.sliding_deceleration <= self.rolling_deceleration {
            return Err(format!(
                "ball.sliding_deceleration ({}) must be greater than ball.rolling_deceleration ({})",
                self.sliding_deceleration, self.rolling_deceleration
            ));
        }
        if !(self.sliding_to_rolling_ratio > 0.0 && self.sliding_to_rolling_ratio < 1.0) {
            return Err(format!(
                "ball.sliding_to_rolling_ratio must be in (0, 1), got {}",
                self.sliding_to_rolling_ratio
            ));
        }
        Ok(())
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
    #[serde(default)]
    pub ball: BallConfig,
    #[serde(default)]
    pub blue_team: TeamConfig,
    #[serde(default)]
    pub yellow_team: TeamConfig,
    #[serde(default)]
    pub vision_plugins: Vec<PluginConfig>,
}

mod default_configs {
    include!(concat!(env!("OUT_DIR"), "/default_configs.rs"));
}

pub fn default_config_names() -> impl Iterator<Item = &'static str> {
    default_configs::DEFAULT_CONFIGS
        .iter()
        .map(|(name, _)| *name)
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
    let config: Config = serde_yaml::from_str(&contents)
        .map_err(|e| format!("Failed to parse config '{name_or_path}': {e}"))?;
    config
        .ball
        .validate()
        .map_err(|e| format!("Invalid config '{name_or_path}': {e}"))?;
    Ok(config)
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

    #[test]
    fn ball_config_validation() {
        assert!(BallConfig::default().validate().is_ok());
        let invalid = [
            BallConfig {
                radius: 0.0,
                ..Default::default()
            },
            BallConfig {
                sliding_to_rolling_ratio: 1.0,
                ..Default::default()
            },
            BallConfig {
                sliding_to_rolling_ratio: 0.0,
                ..Default::default()
            },
            BallConfig {
                rolling_deceleration: 2.5,
                sliding_deceleration: 2.5,
                ..Default::default()
            },
            BallConfig {
                rolling_deceleration: -0.1,
                ..Default::default()
            },
            BallConfig {
                mass: f32::NAN,
                ..Default::default()
            },
        ];
        for config in invalid {
            assert!(config.validate().is_err(), "accepted {config:?}");
        }
    }
}
