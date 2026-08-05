use crate::physics_engine::PhsyicsEngine;
use crate::plugin_manager::PluginManager;
use yass_corelib::*;

pub struct Simulator {
    pub plugins: PluginManager,
    physics: PhsyicsEngine,
}

impl Simulator {
    pub fn new() -> Self {
        Self {
            plugins: PluginManager::new(),
            physics: PhsyicsEngine::new(),
        }
    }

    pub fn run_frame(&mut self) {
        self.physics.run_frame();
    }

    pub fn get_state(&self) -> yass_corelib::SimulationState {
        yass_corelib::SimulationState {
            blue_robots: std::array::from_fn(|i| yass_corelib::Robot {
                id: i as u8,
                present: false,
                pose: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                velocity: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                acceleration: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                ball_sense: false,
            }),
            yellow_robots: std::array::from_fn(|i| yass_corelib::Robot {
                id: i as u8,
                present: false,
                pose: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                velocity: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                acceleration: yass_corelib::VectorSE2 {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                ball_sense: false,
            }),
            ball: yass_corelib::Ball {
                position: yass_corelib::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                velocity: yass_corelib::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                acceleration: yass_corelib::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
        }
    }

    pub fn apply_robot_commands(&mut self, _color: yass_corelib::TeamColor, _commands: &[RobotCommand; 16]) {}
}
