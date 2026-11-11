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
            config: config.clone(),
        }
    }

    pub fn run_frame(&mut self) {
        self.physics.run_frame();
    }

    pub fn get_state(&self) -> yass_corelib::SimulationState {
        yass_corelib::SimulationState {
            blue_robots: std::array::from_fn(|i| {
                self.physics.get_bot_state(TeamColor::Blue, i as u8)
            }),
            yellow_robots: std::array::from_fn(|i| {
                self.physics.get_bot_state(TeamColor::Yellow, i as u8)
            }),
            ball: self.physics.get_ball_state(),
        }
    }

    pub fn apply_robot_commands(
        &mut self,
        color: yass_corelib::TeamColor,
        commands: &[RobotCommand; 16],
    ) {
        for command in commands {
            match command.motion_command {
                MotionCommand::Off => (),
                MotionCommand::Velocity { x, y, theta } => self.physics.set_bot_velocity(
                    color,
                    command.id,
                    VectorSE2 { x, y, theta },
                    command.motion_command_frame,
                ),
                MotionCommand::Acceleration { x, y, theta } => self.physics.set_bot_acceleration(
                    color,
                    command.id,
                    VectorSE2 { x, y, theta },
                    command.motion_command_frame,
                ),
                MotionCommand::Wrench { x, y, theta } => self.physics.set_bot_wrench(
                    color,
                    command.id,
                    VectorSE2 { x, y, theta },
                    command.motion_command_frame,
                ),
            }
            self.physics
                .set_bot_kick(color, command.id, command.kick_command);
            self.physics
                .set_bot_dribbler(color, command.id, command.dribbler_command);
        }
    }
}
