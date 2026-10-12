use std::alloc::{Layout, alloc, dealloc};
use std::ffi::{CStr, c_char, c_void};

use yass_corelib::*;

static USER_DATA_LAYOUT: Layout = Layout::new::<String>();

#[unsafe(no_mangle)]
pub extern "C" fn yass_load_league_proto_firmware(
    color: TeamColor,
    _config: *const c_char,
) -> *mut c_void {
    unsafe {
        if !_config.is_null() {
            let c_str = CStr::from_ptr(_config);
            let owned_string = c_str.to_string_lossy().into_owned();
            println!("{owned_string}");
        }
    }
    unsafe {
        let data = alloc(USER_DATA_LAYOUT) as *mut String;
        if data.is_null() {
            panic!("Could not allocate user data memory.");
        }
        *data = match color {
            TeamColor::Blue => String::from("Blue"),
            TeamColor::Yellow => String::from("Yellow"),
        };
        data as *mut c_void
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_league_proto_firmware(
    _state: SimulationState,
    _delta_t_us: u64,
    user_data: *mut c_void,
    commands_out: &mut [RobotCommand; 16],
) {
    let color_string = unsafe { (*(user_data as *mut String)).clone() };
    println!("run_firmware_frame from league-std-plugins");
    println!("Color is {color_string}");
    commands_out[0].kick_command = yass_corelib::KickCommand::KickImmediate { speed: 20.0 };
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_league_proto_firmware(user_data: *mut c_void) {
    unsafe {
        dealloc(user_data as *mut u8, USER_DATA_LAYOUT);
    }
}
