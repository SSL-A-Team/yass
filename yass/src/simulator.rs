use crate::config::Config;
use crate::physics::engine::PhsyicsEngine;
use crate::plugin_manager::PluginManager;
use yass_corelib::*;

pub struct Simulator {
    pub plugins: PluginManager,
    physics: PhsyicsEngine,
}

impl Simulator {
    pub fn new(config: &Config) -> Self {
        Self {
            plugins: PluginManager::new(),
            physics: PhsyicsEngine::new(&config),
        }
    }

    pub fn run_frame(&mut self, delta_t: std::time::Duration) {
        self.physics.run_frame(delta_t);
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
            if !command.active {
                continue;
            }
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

    pub fn teleport_ball(
        &mut self,
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
        vel_x: Option<f32>,
        vel_y: Option<f32>,
        vel_z: Option<f32>,
    ) {
        self.physics.teleport_ball(x, y, z, vel_x, vel_y, vel_z);
    }

    pub fn teleport_robot(
        &mut self,
        color: TeamColor,
        id: u8,
        x: Option<f32>,
        y: Option<f32>,
        theta: Option<f32>,
        vel_x: Option<f32>,
        vel_y: Option<f32>,
        vel_theta: Option<f32>,
    ) {
        self.physics
            .teleport_robot(color, id, x, y, theta, vel_x, vel_y, vel_theta);
    }
}
