mod helpers;
mod vision_plugin;

use helpers::*;
use std::{
    ffi::{c_char, c_void},
    ptr::null_mut,
};
use vision_plugin::VisionPlugin;
use yass_corelib::FieldDimensions;
use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn yass_load_league_proto_vision(
    field: FieldDimensions,
    config: *const c_char,
) -> *mut c_void {
    let plugin = parse_config(config).and_then(|config| VisionPlugin::new(&field, config));
    match plugin {
        Ok(plugin) => {
            println!(
                "league_proto_vision: publishing SSL vision on {}",
                plugin.destination
            );
            Box::into_raw(Box::new(plugin)) as *mut c_void
        }
        Err(e) => {
            eprintln!("league_proto_vision: failed to initialize: {e}");
            null_mut()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_league_proto_vision(state: SimulationState, user_data: *mut c_void) {
    if user_data.is_null() {
        return;
    }
    let plugin = unsafe { &mut *(user_data as *mut VisionPlugin) };
    plugin.run(&state);
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_league_proto_vision(user_data: *mut c_void) {
    if user_data.is_null() {
        return;
    }
    // Dropping the plugin closes the socket.
    drop(unsafe { Box::from_raw(user_data as *mut VisionPlugin) });
}
