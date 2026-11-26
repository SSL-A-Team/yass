use clap::Parser;
use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use yass::common_setup::CommonSetup;
use yass::config::load_config;

#[derive(Parser, Debug)]
#[command(author, version, about = "A RoboCup SSL Simulator")]
struct Args {
    /// Name of a built-in default config (e.g. "div_b") or a path to a config file
    #[arg(short = 'c', long, default_value = "div_b")]
    config: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let config = load_config(&args.config)?;

    let mut sim = CommonSetup::new(config)?;

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let frame_duration = Duration::from_millis(1);

    sim.simulator.plugins.print_plugin_info();

    println!("Simulator running...");

    while running.load(Ordering::SeqCst) {
        let start = Instant::now();
        sim.step(frame_duration)?;
        let elapsed = start.elapsed();
        if let Some(remaining) = frame_duration.checked_sub(elapsed) {
            thread::sleep(remaining);
        }
    }

    println!("Simulator closing.");

    Ok(())
}
