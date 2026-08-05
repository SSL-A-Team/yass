use protobuf;
use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn run_vision_frame(_state: SimulationState) {
    println!("run_vision_frame from league-std-plugins");
}
