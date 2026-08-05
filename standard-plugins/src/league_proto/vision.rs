use std::{
    ffi::{CStr, c_char, c_void},
    ptr::null_mut,
};
use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn yass_load_league_proto_vision(_config: *const c_char) -> *mut c_void {
    unsafe {
        let c_str = CStr::from_ptr(_config);
        let owned_string = c_str.to_string_lossy().into_owned();
        println!("{owned_string}");
    }
    null_mut()
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_league_proto_vision(_state: SimulationState, _user_data: *mut c_void) {
    // println!("run_vision_frame from league-std-plugins");
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_league_proto_vision(_user_data: *mut c_void) {}
