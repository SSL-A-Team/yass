mod firmware_config;
mod firmware_plugin;

use crate::league_proto::helpers::parse_config;
use firmware_plugin::FirmwarePlugin;
use std::{
    ffi::{c_char, c_void},
    ptr::null_mut,
};
use yass_corelib::*;

/// # Safety
/// yass is responsible for calling this function with a valid config pointer
#[unsafe(no_mangle)]
pub unsafe extern "C" fn yass_load_league_proto_firmware(
    color: TeamColor,
    config: *const c_char,
) -> *mut c_void {
    let plugin = unsafe { parse_config(config).and_then(|config| FirmwarePlugin::new(color, config)) };
    match plugin {
        Ok(plugin) => Box::into_raw(Box::new(plugin)) as *mut c_void,
        Err(e) => {
            eprintln!("league_proto_firmware: failed to initialize: {e}");
            null_mut()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_league_proto_firmware(
    state: SimulationState,
    delta_t_us: u64,
    user_data: *mut c_void,
    commands_out: &mut [RobotCommand; 16],
) {
    if user_data.is_null() {
        return;
    }
    let plugin = unsafe { &mut *(user_data as *mut FirmwarePlugin) };
    plugin.run(&state, delta_t_us, commands_out);
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_league_proto_firmware(user_data: *mut c_void) {
    if user_data.is_null() {
        return;
    }
    drop(unsafe { Box::from_raw(user_data as *mut FirmwarePlugin) });
}
