use std::ffi::{c_char, c_void};
use std::ptr::null_mut;
use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn yass_load_ateam_firmware(
    _color: TeamColor,
    _config: *const c_char,
) -> *mut c_void {
    null_mut()
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_ateam_firmware(
    _state: SimulationState,
    _delta_t_us: u64,
    _user_data: *mut c_void,
    _commands_out: &mut [RobotCommand; 16],
) {
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_ateam_firmware(_user_data: *mut c_void) {}
