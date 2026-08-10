use std::ffi::{c_char, c_void};

#[repr(C)]
#[derive(Clone, Copy)]
pub enum TeamColor {
    Blue,
    Yellow,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VectorSE2 {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Robot {
    pub id: u8,
    pub present: bool,
    pub pose: VectorSE2,
    pub velocity: VectorSE2,
    pub acceleration: VectorSE2,
    pub ball_sense: bool,
}

// #[repr(C)]
// pub struct RobotArray {
//     pub data: * const Robot,
//     pub size: usize
// }

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Ball {
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SimulationState {
    pub blue_robots: [Robot; 16],
    pub yellow_robots: [Robot; 16],
    pub ball: Ball,
}

#[repr(C)]
#[derive(Default)]
pub enum MotionCommand {
    #[default]
    Off,
    Velocity { x: f64, y: f64, theta: f64 },
    Acceleration { x: f64, y: f64, theta: f64},
    Wrench { x: f64, y: f64, theta: f64 },
}

#[repr(C)]
#[derive(Default)]
pub enum MotionCommandFrame {
    #[default]
    Local,
    Global,
}

#[repr(C)]
#[derive(Default)]
pub enum KickCommand {
    /// Kicker does not fire
    #[default]
    Off,
    /// Kicker fires immediately at the given speed, in m/s
    KickImmediate { speed: f64 },
    /// Kicker fires if the ball is touching the front of the robot, in m/s
    KickOnTouch { speed: f64 },
}

#[repr(C)]
#[derive(Default)]
pub enum DribblerCommand {
    /// Ball will not stick to the dribbler
    #[default]
    Off,
    /// Ball will glue to the front of the robot
    Sticky,
    /// Ball will spin towards the robot, in rad/s
    Rotation { speed: f64 },
    /// Ball will be pulled towards the front of the robot with the given force, in N
    Force { force: f64 },
}

#[repr(C)]
#[derive(Default)]
pub struct RobotCommand {
    pub id: u8,
    pub motion_command: MotionCommand,
    pub motion_command_frame: MotionCommandFrame,
    pub kick_command: KickCommand,
    pub dribbler_command: DribblerCommand,
}

pub type FirmwareLoadCallback = unsafe extern "C" fn(color: TeamColor, config: *const c_char) -> *mut c_void;
pub type FirmwareUnloadCallback = unsafe extern "C" fn(user_data: *mut c_void);
pub type FirmwareFrameCallback = unsafe extern "C" fn(
    state: SimulationState,
    delta_t_us: u64,
    user_data: *mut c_void,
    commands_out: &mut [RobotCommand; 16],
);

pub type VisionLoadCallback = unsafe extern "C" fn(config: *const c_char) -> *mut c_void;
pub type VisionUnloadCallback = unsafe extern "C" fn(user_data: *mut c_void);
pub type VisionFrameCallback = unsafe extern "C" fn(state: SimulationState, user_data: *mut c_void);
