use clap::Parser;
use std::error::Error;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use yass::config::Config;
use yass::simulator::Simulator;
use yass::{init_commands, reset_commands};

#[derive(Parser, Debug)]
#[command(author, version, about = "A RoboCup SSL Simulator")]
struct Args {
    #[arg(short='c', long)]
    config: PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut simulator = Simulator::new();

    let args = Args::parse();

    let config_file_contents = std::fs::read_to_string(args.config)?;
    let config: Config = serde_yaml::from_str(&config_file_contents)?;

    if let Some(blue_firmware) = config.blue_team.firmware.as_ref() {
        simulator.plugins.load_firmware_plugin(
            blue_firmware.lib.clone(),
            blue_firmware.name.clone(),
            yass_corelib::TeamColor::Blue,
            serde_yaml::to_string(blue_firmware)?
        )?;
    }
    if let Some(yellow_firmware) = config.yellow_team.firmware.as_ref() {
        simulator.plugins.load_firmware_plugin(
            yellow_firmware.lib.clone(),
            yellow_firmware.name.clone(),
            yass_corelib::TeamColor::Yellow,
            serde_yaml::to_string(yellow_firmware)?
        )?;
    }
    for vision_plugin_config in config.vision_plugins.iter() {
        simulator.plugins.load_vision_plugin(
            vision_plugin_config.lib.clone(),
            vision_plugin_config.name.clone(),
            serde_yaml::to_string(vision_plugin_config)?
        )?;
    }

    let mut blue_commands = init_commands();
    let mut yellow_commands = init_commands();

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let frame_duration = Duration::from_millis(1);
    let delta_t_us = frame_duration.as_micros() as u64;

    simulator.plugins.print_plugin_info();

    println!("Simulator running...");

    while running.load(Ordering::SeqCst) {
        let start = Instant::now();
        let state = simulator.get_state();
        simulator.plugins.run_vision_plugins(state);
        reset_commands(&mut blue_commands);
        reset_commands(&mut yellow_commands);
        simulator
            .plugins
            .run_blue_firmware(state, delta_t_us, &mut blue_commands);
        simulator
            .plugins
            .run_yellow_firmware(state, delta_t_us, &mut yellow_commands);
        simulator.apply_robot_commands(yass_corelib::TeamColor::Blue, &blue_commands);
        simulator.apply_robot_commands(yass_corelib::TeamColor::Yellow, &yellow_commands);
        simulator.run_frame();
        let elapsed = start.elapsed();
        if let Some(remaining) = frame_duration.checked_sub(elapsed) {
            thread::sleep(remaining);
        }
    }

    println!("Simulator closing.");

    Ok(())
}
