use std::{
    io::ErrorKind,
    net::{SocketAddrV4, UdpSocket},
};

use crate::simulator::Simulator;
use prost::Message;
use yass_corelib::TeamColor;
use yass_league_protos::SimulatorCommand;

pub struct ControlInterface {
    socket: UdpSocket,
    recv_buffer: [u8; 2048],
}

impl ControlInterface {
    pub fn new(addr: SocketAddrV4) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = UdpSocket::bind(addr)?;
        socket.set_nonblocking(true)?;
        println!("Control listening on {0}", addr);
        Ok(Self {
            socket: socket,
            recv_buffer: [0u8; 2048],
        })
    }

    pub fn run(&mut self, sim: &mut Simulator) -> std::io::Result<()> {
        loop {
            match self.socket.recv(&mut self.recv_buffer) {
                Ok(bytes_read) => {
                    self.handle_packet(sim, bytes_read)?;
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                    break;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn handle_packet(&mut self, sim: &mut Simulator, bytes_read: usize) -> std::io::Result<()> {
        let command = SimulatorCommand::decode(&self.recv_buffer[..bytes_read])?;
        let Some(control_packet) = command.control else {
            return Ok(());
        };
        if let Some(teleport_ball) = control_packet.teleport_ball {
            sim.teleport_ball(
                teleport_ball.x,
                teleport_ball.y,
                teleport_ball.z,
                teleport_ball.vx,
                teleport_ball.vy,
                teleport_ball.vz,
            );
        }
        for teleport_robot in control_packet.teleport_robot {
            if teleport_robot.id.team.is_none() || teleport_robot.id.id.is_none() {
                continue;
            }
            let color = match teleport_robot.id.team {
                Some(1) => TeamColor::Yellow,
                Some(2) => TeamColor::Blue,
                _ => {
                    continue;
                }
            };
            let Some(id) = teleport_robot.id.id else {
                continue;
            };
            sim.teleport_robot(
                color,
                id as u8,
                teleport_robot.x,
                teleport_robot.y,
                teleport_robot.orientation,
                teleport_robot.v_x,
                teleport_robot.v_y,
                teleport_robot.v_angular,
            );
        }
        Ok(())
    }
}
