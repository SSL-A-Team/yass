use yass_corelib::RobotCommand;

pub mod config;
mod physics_engine;
pub mod plugin_manager;
pub mod simulator;

pub fn init_commands() -> [RobotCommand; 16] {
    std::array::from_fn(|i| RobotCommand::default_with_id(i as u8))
}

pub fn reset_commands(commands: &mut [RobotCommand; 16]) {
    for command in commands.iter_mut() {
        command.motion_command = yass_corelib::MotionCommand::default();
        command.motion_command_frame = yass_corelib::MotionCommandFrame::default();
        command.kick_command = yass_corelib::KickCommand::default();
        command.dribbler_command = yass_corelib::DribblerCommand::default()
    }
}
