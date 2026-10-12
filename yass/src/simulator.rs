use crate::config::Config;
use crate::physics::engine::PhsyicsEngine;
use crate::plugin_manager::PluginManager;
use yass_corelib::*;

pub struct Simulator {
    pub plugins: PluginManager,
    physics: PhsyicsEngine,
    config: Config,
}

impl Simulator {
    pub fn new(config: &Config) -> Self {
        Self {
            plugins: PluginManager::new(),
            physics: PhsyicsEngine::new(&config),
            config: config.clone()
        }
    }

    pub fn run_frame(&mut self) {
        self.physics.run_frame();
    }

    pub fn get_state(&self) -> yass_corelib::SimulationState {
        yass_corelib::SimulationState {
            blue_robots: std::array::from_fn(|i| self.physics.get_blue_bot_state(i as u8)),
            yellow_robots: std::array::from_fn(|i| self.physics.get_yellow_bot_state(i as u8)),
            ball: self.physics.get_ball_state(),
        }
    }

    pub fn apply_robot_commands(&mut self, _color: yass_corelib::TeamColor, _commands: &[RobotCommand; 16]) {}
}
