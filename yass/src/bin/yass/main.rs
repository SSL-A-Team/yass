mod rendering;

use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};

use clap::Parser;
use kiss3d::prelude::*;
use yass::{
    common_setup::CommonSetup,
    config::{Config, load_config},
};
use yass_corelib::SimulationState;

use crate::rendering::Renderer;

#[derive(Parser, Debug)]
#[command(author, version, about = "A RoboCup SSL Simulator")]
struct Args {
    /// Name of a built-in default config (e.g. "div_b") or a path to a config file
    #[arg(short = 'c', long, default_value = "div_b")]
    config: String,
}
#[kiss3d::main]
async fn main() {
    let args = Args::parse();

    let config = match load_config(&args.config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to load config: {e}");
            return;
        }
    };

    let shared_state = Arc::new(Mutex::new(SimulationState::default()));

    let (err_tx, err_rx) = mpsc::channel();
    let (ready_tx, ready_rx) = mpsc::channel();

    let running = Arc::new(AtomicBool::new(true));

    let thread_config = config.clone();
    let thread_shared_state = shared_state.clone();
    let thread_running = running.clone();
    let sim_thread_handle = thread::spawn(move || {
        let res = sim_thread_run(thread_config, thread_running, ready_tx, thread_shared_state);
        if let Err(send_err) = err_tx.send(res.map_err(|e| format!("{e}"))) {
            eprintln!("Failed to send simulation thread error: {send_err}");
        }
    });

    if let Ok(Err(e)) = ready_rx.recv() {
        eprintln!("Failed to initialize simulator: {e}");
        return;
    }

    if let Ok(Err(e)) = err_rx.try_recv() {
        eprintln!("Failed to initialize simulator: {e}");
        return;
    }

    let mut window = Window::new("yass").await;
    window.set_background_color(LIGHT_SKY_BLUE);
    let mut camera = OrbitCamera3d::default();
    camera.set_up_axis(Vec3::Z);
    camera.set_pitch((55.0 as f32).to_radians());
    camera.set_yaw((90.0 as f32).to_radians());
    camera.set_dist(12.0);
    let mut scene = SceneNode3d::empty();

    let mut renderer = Renderer::new(&mut scene, &config);

    while window.render_3d(&mut scene, &mut camera).await {
        if let Ok(Err(sim_err)) = err_rx.try_recv() {
            eprintln!("Error from simulation thread: {sim_err}");
            break;
        }
        let s = *shared_state.lock().unwrap();

        renderer.update(&mut scene, s);
    }

    running.store(false, Ordering::Relaxed);
    if let Err(_) = sim_thread_handle.join() {
        eprintln!("Failed to finish simulation thread.");
    }
}

fn sim_thread_run(
    config: Config,
    running: Arc<AtomicBool>,
    ready_tx: mpsc::Sender<Result<(), String>>,
    shared_state: Arc<Mutex<SimulationState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut sim = match CommonSetup::new(config) {
        Ok(s) => s,
        Err(e) => {
            ready_tx.send(Err(e.to_string()))?;
            return Err(e);
        }
    };
    let frame_duration = Duration::from_millis(1);
    sim.simulator.plugins.print_plugin_info();
    ready_tx.send(Ok(()))?;
    loop {
        if !running.load(Ordering::Relaxed) {
            break;
        }
        let start = Instant::now();
        sim.step(frame_duration)?;
        *shared_state.lock().unwrap() = sim.simulator.get_state();
        let elapsed = start.elapsed();
        if let Some(remaining) = frame_duration.checked_sub(elapsed) {
            thread::sleep(remaining);
        }
    }
    Ok(())
}
