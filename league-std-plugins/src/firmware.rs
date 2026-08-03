use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn run_firmware_frame(
    state: SimulationState,
    color: TeamColor,
    delta_t_us: u64,
    commands_out: &mut [RobotCommand; 16],
) {
    println!("run_firmware_frame from league-std-plugins");
    commands_out[0].kick_command = yass_corelib::KickCommand::KickImmediate { speed: 20.0 };
}
