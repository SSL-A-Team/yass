use std::{
    net::{Ipv4Addr, SocketAddrV4, UdpSocket},
    time::{Duration, Instant},
};

use crate::league_proto::vision::helpers::*;
use prost::Message;
use yass_corelib::*;

use crate::league_proto::league_protos::{
    SslDetectionFrame, SslGeometryData, SslSource, SslWrapperPacket,
};

pub struct VisionPlugin {
    socket: UdpSocket,
    pub destination: SocketAddrV4,
    detection_period: Duration,
    geometry_period: Duration,
    geometry: SslGeometryData,
    frame_number: u32,
    last_detection_sent: Option<Instant>,
    last_geometry_sent: Option<Instant>,
}

impl VisionPlugin {
    pub fn new(
        field: &FieldDimensions,
        config: VisionConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        if !config.address.is_multicast() {
            return Err(format!("{} is not a multicast address", config.address).into());
        }
        if !(config.detection_rate_hz > 0.0) {
            return Err("detection_rate_hz must be positive".into());
        }
        if !(config.geometry_period_s > 0.0) {
            return Err("geometry_period_s must be positive".into());
        }

        let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0))?;
        socket.set_multicast_ttl_v4(config.multicast_ttl)?;
        // Allow clients on this machine to receive the stream.
        socket.set_multicast_loop_v4(true)?;
        // Never stall the simulation loop on the network.
        socket.set_nonblocking(true)?;

        Ok(Self {
            socket,
            destination: SocketAddrV4::new(config.address, config.port),
            detection_period: Duration::from_secs_f64(1.0 / config.detection_rate_hz),
            geometry_period: Duration::from_secs_f64(config.geometry_period_s),
            geometry: make_geometry(field),
            frame_number: 0,
            last_detection_sent: None,
            last_geometry_sent: None,
        })
    }

    pub fn run(&mut self, state: &SimulationState) {
        let now = Instant::now();

        if is_due(self.last_geometry_sent, self.geometry_period, now) {
            self.last_geometry_sent = Some(now);
            let packet = SslWrapperPacket {
                detection: None,
                geometry: Some(self.geometry.clone()),
                source: Some(SslSource::Other as i32),
            };
            self.send(&packet);
        }

        if is_due(self.last_detection_sent, self.detection_period, now) {
            self.last_detection_sent = Some(now);
            let timestamp = unix_time_s();
            let packet = SslWrapperPacket {
                detection: Some(SslDetectionFrame {
                    frame_number: self.frame_number,
                    t_capture: timestamp,
                    t_sent: timestamp,
                    t_capture_camera: None,
                    t_offsets: vec![],
                    camera_id: 0,
                    balls: vec![make_ball(&state.ball)],
                    robots_yellow: make_robots(&state.yellow_robots),
                    robots_blue: make_robots(&state.blue_robots),
                }),
                geometry: None,
                source: Some(SslSource::Other as i32),
            };
            self.frame_number = self.frame_number.wrapping_add(1);
            self.send(&packet);
        }
    }

    fn send(&self, packet: &SslWrapperPacket) {
        // Vision is best-effort with no retransmission, so log the failure and keep going.
        if let Err(e) = self
            .socket
            .send_to(&packet.encode_to_vec(), self.destination)
        {
            eprintln!("league_proto_vision: failed to send packet: {e}");
        }
    }
}
