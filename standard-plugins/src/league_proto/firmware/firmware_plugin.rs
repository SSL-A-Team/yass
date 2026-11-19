use crate::league_proto::league_protos::robot_move_command::Command;
use crate::league_proto::league_protos::{RobotControl, RobotFeedback};
use crate::league_proto::{
    firmware::firmware_config::FirmwareConfig, league_protos::RobotControlResponse,
};
use prost::Message;
use std::{
    io::ErrorKind,
    net::{SocketAddr, SocketAddrV4, UdpSocket},
};
use yass_corelib::{
    DribblerCommand, MotionCommand, MotionCommandFrame, RobotCommand, SimulationState, TeamColor,
};
pub struct FirmwarePlugin {
    color: TeamColor,
    socket: UdpSocket,
    recv_buffer: [u8; 2048], // TODO(mbarulic): determine true max-needed size
    send_buffer: [u8; 2048], // TODO(mbarulic): determine true max-needed size
}

impl FirmwarePlugin {
    pub fn new(
        color: TeamColor,
        config: FirmwareConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        println!("{config:?}");
        let socket = UdpSocket::bind(SocketAddrV4::new(config.address, config.port))?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            color,
            socket,
            recv_buffer: [0u8; 2048],
            send_buffer: [0u8; 2048],
        })
    }

    pub fn run(
        &mut self,
        state: &SimulationState,
        delta_t_us: u64,
        commands_out: &mut [RobotCommand; 16],
    ) {
        if let Ok(Some((bytes_read, sender))) = self.get_newest_command_packet() {
            if let Ok(message) = RobotControl::decode(&self.recv_buffer[..bytes_read]) {
                for command in message.robot_commands {
                    let command_out = &mut commands_out[command.id as usize];
                    if let Some(robot_move_command) = command.move_command {
                        if let Some(move_command) = robot_move_command.command {
                            match move_command {
                                Command::WheelVelocity(_) => {
                                    eprintln!("Wheel velocities not yet supported.")
                                }
                                Command::LocalVelocity(move_local_velocity) => {
                                    println!("Local vel: {0}, {1}, {2}", move_local_velocity.forward, move_local_velocity.left, move_local_velocity.angular);
                                    command_out.motion_command = MotionCommand::Velocity {
                                        x: move_local_velocity.forward,
                                        y: move_local_velocity.left,
                                        theta: move_local_velocity.angular,
                                    };
                                    command_out.motion_command_frame = MotionCommandFrame::Local;
                                }
                                Command::GlobalVelocity(move_global_velocity) => {
                                    command_out.motion_command = MotionCommand::Velocity {
                                        x: move_global_velocity.x,
                                        y: move_global_velocity.y,
                                        theta: move_global_velocity.angular,
                                    };
                                    command_out.motion_command_frame = MotionCommandFrame::Global;
                                }
                            }
                        }
                    }
                    command_out.kick_command.speed = command.kick_speed.unwrap_or(0.0);
                    command_out.kick_command.angle = command.kick_angle.unwrap_or(0.0);
                    if let Some(dribbler_speed) = command.dribbler_speed {
                        command_out.dribbler_command = DribblerCommand::Rotation {
                            speed: dribbler_speed,
                        };
                    }
                }
            }
            let mut response = RobotControlResponse::default();
            let sim_bots = match self.color {
                TeamColor::Blue => &state.blue_robots,
                TeamColor::Yellow => &state.yellow_robots,
            };
            for robot in sim_bots {
                if !robot.present {
                    continue;
                }
                let mut feedback = RobotFeedback::default();
                feedback.id = robot.id as u32;
                feedback.dribbler_ball_contact = Some(robot.ball_sense);
                response.feedback.push(feedback);
            }
            response
                .encode(&mut &mut self.send_buffer[..])
                .unwrap_or_else(|e| eprintln!("Failed to encode response: {e}"));
            if let Err(e) = self
                .socket
                .send_to(&self.send_buffer[..response.encoded_len()], sender)
            {
                eprintln!("Failed to send feedback: {e}");
            }
        }
    }

    fn get_newest_command_packet(&mut self) -> std::io::Result<Option<(usize, SocketAddr)>> {
        let mut latest = None;

        loop {
            match self.socket.recv_from(&mut self.recv_buffer) {
                Ok((len, src)) => {
                    // Overwrite previous packet data in `buf` and record the new metadata
                    latest = Some((len, src));
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                    // The OS receive buffer is now empty; return the last packet we read
                    return Ok(latest);
                }
                Err(e) => return Err(e),
            }
        }
    }
}
