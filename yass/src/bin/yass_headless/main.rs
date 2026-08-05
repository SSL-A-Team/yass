use std::error::Error;
use std::env;
use yass::plugin_manager::PluginManager;
use yass_corelib;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        panic!("Bad args");
    }

    let lib_path = args[1].clone();

    println!("Loading plugin from {lib_path}");

    let mut manager = PluginManager::new();

    manager.load_firmware_plugin(lib_path, yass_corelib::TeamColor::Blue);

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

    manager.run_blue_firmware(state, delta_t_us, &mut commands);

    Ok(())
}
