use serde::Deserialize;
use yass_corelib::*;

use std::{
    ffi::{CStr, c_char},
    net::Ipv4Addr,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use crate::league_proto::league_protos::{
    SslDetectionBall, SslDetectionRobot, SslFieldCircularArc, SslFieldLineSegment,
    SslFieldShapeType, SslGeometryData, SslGeometryFieldSize, Vector2f,
};

const METERS_TO_MM: f32 = 1000.0;

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct VisionConfig {
    pub address: Ipv4Addr,
    pub port: u16,
    pub multicast_ttl: u32,
    pub detection_rate_hz: f64,
    pub geometry_period_s: f64,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            address: Ipv4Addr::new(224, 5, 23, 2),
            port: 10020,
            multicast_ttl: 1,
            detection_rate_hz: 60.0,
            geometry_period_s: 1.0,
        }
    }
}

pub fn is_due(last_sent: Option<Instant>, period: Duration, now: Instant) -> bool {
    last_sent.is_none_or(|last| now.duration_since(last) >= period)
}

pub fn unix_time_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

pub fn make_ball(ball: &Ball) -> SslDetectionBall {
    SslDetectionBall {
        confidence: 1.0,
        area: None,
        x: ball.position.x * METERS_TO_MM,
        y: ball.position.y * METERS_TO_MM,
        z: Some(ball.position.z * METERS_TO_MM),
        pixel_x: 0.0,
        pixel_y: 0.0,
    }
}

pub fn make_robots(robots: &[Robot]) -> Vec<SslDetectionRobot> {
    robots
        .iter()
        .filter(|r| r.present)
        .map(|r| SslDetectionRobot {
            confidence: 1.0,
            robot_id: Some(r.id as u32),
            x: r.pose.x * METERS_TO_MM,
            y: r.pose.y * METERS_TO_MM,
            orientation: Some(r.pose.theta),
            pixel_x: 0.0,
            pixel_y: 0.0,
            height: None,
        })
        .collect()
}

pub fn make_geometry(field: &FieldDimensions) -> SslGeometryData {
    let mm = |m: f32| (m * METERS_TO_MM).round() as i32;
    SslGeometryData {
        field: SslGeometryFieldSize {
            field_length: mm(field.length),
            field_width: mm(field.width),
            goal_width: mm(field.goal_width),
            goal_depth: mm(field.goal_depth),
            boundary_width: mm(field.boundary_width),
            // The simulator uses the same boundary width on every side of the field.
            boundary_width_goal_line: Some(mm(field.boundary_width)),
            field_lines: make_field_lines(field),
            field_arcs: make_field_arcs(field),
            penalty_area_depth: Some(mm(field.penalty_area_depth)),
            penalty_area_width: Some(mm(field.penalty_area_width)),
            center_circle_radius: Some(mm(field.center_circle_radius)),
            line_thickness: Some(mm(field.line_thickness)),
            ..Default::default()
        },
        calib: vec![],
        models: None,
    }
}

/// Generates the standard SSL field markings. All points are line centers, in mm.
pub fn make_field_lines(field: &FieldDimensions) -> Vec<SslFieldLineSegment> {
    let hl = field.length / 2.0 * METERS_TO_MM;
    let hw = field.width / 2.0 * METERS_TO_MM;
    let pd = field.penalty_area_depth * METERS_TO_MM;
    let hpw = field.penalty_area_width / 2.0 * METERS_TO_MM;
    let thickness = field.line_thickness * METERS_TO_MM;

    use SslFieldShapeType as T;
    let lines = [
        (T::TopTouchLine, (-hl, hw), (hl, hw)),
        (T::BottomTouchLine, (-hl, -hw), (hl, -hw)),
        (T::LeftGoalLine, (-hl, -hw), (-hl, hw)),
        (T::RightGoalLine, (hl, -hw), (hl, hw)),
        (T::HalfwayLine, (0.0, -hw), (0.0, hw)),
        (T::CenterLine, (-hl, 0.0), (hl, 0.0)),
        (T::LeftPenaltyStretch, (-hl + pd, -hpw), (-hl + pd, hpw)),
        (T::RightPenaltyStretch, (hl - pd, -hpw), (hl - pd, hpw)),
        (
            T::LeftFieldLeftPenaltyStretch,
            (-hl, -hpw),
            (-hl + pd, -hpw),
        ),
        (T::LeftFieldRightPenaltyStretch, (-hl, hpw), (-hl + pd, hpw)),
        (T::RightFieldLeftPenaltyStretch, (hl, hpw), (hl - pd, hpw)),
        (
            T::RightFieldRightPenaltyStretch,
            (hl, -hpw),
            (hl - pd, -hpw),
        ),
    ];

    lines
        .into_iter()
        .map(|(shape, (x1, y1), (x2, y2))| SslFieldLineSegment {
            name: shape.as_str_name().to_string(),
            p1: Vector2f { x: x1, y: y1 },
            p2: Vector2f { x: x2, y: y2 },
            thickness,
            r#type: Some(shape as i32),
        })
        .collect()
}

pub fn make_field_arcs(field: &FieldDimensions) -> Vec<SslFieldCircularArc> {
    vec![SslFieldCircularArc {
        name: SslFieldShapeType::CenterCircle.as_str_name().to_string(),
        center: Vector2f { x: 0.0, y: 0.0 },
        radius: field.center_circle_radius * METERS_TO_MM,
        a1: 0.0,
        a2: std::f32::consts::TAU,
        thickness: field.line_thickness * METERS_TO_MM,
        r#type: Some(SslFieldShapeType::CenterCircle as i32),
    }]
}

pub fn parse_config(config: *const c_char) -> Result<VisionConfig, Box<dyn std::error::Error>> {
    if config.is_null() {
        return Ok(VisionConfig::default());
    }
    let text = unsafe { CStr::from_ptr(config) }.to_str()?;
    Ok(serde_yaml::from_str(text)?)
}
