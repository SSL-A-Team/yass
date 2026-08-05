use libloading::{Library, Symbol};
use std::env;
use std::error::Error;
use yass_corelib;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        panic!("Bad args");
    }

    let lib_path = args[1].clone();

    println!("Loading plugin from {lib_path}");

    let plugin_library = unsafe {
        Library::new(lib_path)?
    };
    let firmware_load_callback: Symbol<yass_corelib::FirmwareLoadCallback> = unsafe {
        plugin_library.get(b"yass_load_firmware_plugin\0")?
    };
    let firmware_unload_callback: Symbol<yass_corelib::FirmwareUnloadCallback> = unsafe {
        plugin_library.get(b"yass_unload_firmware_plugin\0")?
    };
    let firmware_callback: Symbol<yass_corelib::FirmwareFrameCallback> = unsafe {
        plugin_library.get(b"yass_run_firmware_frame\0")?
    };

    let user_data = unsafe {
        firmware_load_callback(yass_corelib::TeamColor::Blue)
    };

    let state = yass_corelib::SimulationState {
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
    };
    let delta_t_us = 1000;
    let mut commands: [yass_corelib::RobotCommand; 16] = std::array::from_fn(|i| yass_corelib::RobotCommand {
        id: i as u8,
        motion_command: yass_corelib::MotionCommand::Off,
        motion_command_frame: yass_corelib::MotionCommandFrame::Local,
        kick_command: yass_corelib::KickCommand::Off,
        dribbler_command: yass_corelib::DribblerCommand::Off,
    });
    
    unsafe {
        firmware_callback(state, delta_t_us, user_data, &mut commands);
    }
    if let yass_corelib::KickCommand::KickImmediate{speed} = commands[0].kick_command {
        println!("Received kick command with speed: {speed}");
    }

    unsafe {
        firmware_unload_callback(user_data);
    }

    Ok(())
}
