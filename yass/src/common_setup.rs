use std::net::SocketAddrV4;

use yass_corelib::RobotCommand;

use crate::{config::Config, control_interface::ControlInterface, simulator::Simulator};

pub struct CommonSetup {
    pub simulator: Simulator,
    pub control_interface: ControlInterface,
    pub blue_commands: [RobotCommand; 16],
    pub yellow_commands: [RobotCommand; 16],
}

impl CommonSetup {
    pub fn new(config: Config) -> Result<Self, Box<dyn std::error::Error>> {
        let mut simulator = Simulator::new(&config);
        let control_interface =
            ControlInterface::new(SocketAddrV4::new(config.control.ip, config.control.port))?;
        if let Some(blue_firmware) = config.blue_team.firmware.as_ref() {
            simulator.plugins.load_firmware_plugin(
                &blue_firmware.lib,
                blue_firmware.name.clone(),
                yass_corelib::TeamColor::Blue,
                serde_yaml::to_string(blue_firmware)?,
            )?;
        }
        if let Some(yellow_firmware) = config.yellow_team.firmware.as_ref() {
            simulator.plugins.load_firmware_plugin(
                &yellow_firmware.lib,
                yellow_firmware.name.clone(),
                yass_corelib::TeamColor::Yellow,
                serde_yaml::to_string(yellow_firmware)?,
            )?;
        }
        for vision_plugin_config in config.vision_plugins.iter() {
            simulator.plugins.load_vision_plugin(
                &vision_plugin_config.lib,
                vision_plugin_config.name.clone(),
                (&config.field).into(),
                serde_yaml::to_string(vision_plugin_config)?,
            )?;
        }
        let blue_commands = init_commands();
        let yellow_commands = init_commands();

        Ok(Self {
            simulator,
            control_interface,
            blue_commands,
            yellow_commands,
        })
    }

    pub fn step(&mut self, frame_duration: std::time::Duration) -> Result<(), Box<dyn std::error::Error>> {
        self.control_interface.run(&mut self.simulator)?;
        let state = self.simulator.get_state();
        self.simulator.plugins.run_vision_plugins(state);
        reset_commands(&mut self.blue_commands);
        reset_commands(&mut self.yellow_commands);
        self.simulator
            .plugins
            .run_blue_firmware(state, frame_duration, &mut self.blue_commands);
        self.simulator
            .plugins
            .run_yellow_firmware(state, frame_duration, &mut self.yellow_commands);
        self.simulator
            .apply_robot_commands(yass_corelib::TeamColor::Blue, &self.blue_commands);
        self.simulator
            .apply_robot_commands(yass_corelib::TeamColor::Yellow, &self.yellow_commands);
        self.simulator.run_frame(frame_duration);
        Ok(())
    }
}

pub fn init_commands() -> [RobotCommand; 16] {
    std::array::from_fn(|i| RobotCommand {
        id: i as u8,
        active: false,
        ..Default::default()
    })
}

pub fn reset_commands(commands: &mut [RobotCommand; 16]) {
    for command in commands.iter_mut() {
        command.active = false;
        command.motion_command = yass_corelib::MotionCommand::default();
        command.motion_command_frame = yass_corelib::MotionCommandFrame::default();
        command.kick_command = yass_corelib::KickCommand::default();
        command.dribbler_command = yass_corelib::DribblerCommand::default()
    }
}
