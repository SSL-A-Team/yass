use core::ops::{Deref, Drop};
use libloading::os;
use libloading::{AsFilename, Library, Symbol};
use std::ffi::c_void;
use std::rc::Rc;
use yass_corelib::*;

pub struct PluginManager {
    blue_firmware_plugin: Option<FirmwarePlugin>,
    yellow_firmware_plugin: Option<FirmwarePlugin>,
    vision_plugins: Vec<VisionPlugin>,
}

#[cfg(unix)]
type RawSymbol<T> = os::unix::Symbol<T>;
#[cfg(windows)]
type RawSymbol<T> = os::windows::Symbol<T>;

struct FirmwarePlugin {
    library: Rc<Library>,
    frame: RawSymbol<FirmwareFrameCallback>,
    user_data: *mut c_void,
}

struct VisionPlugin {
    library: Rc<Library>,
    frame: RawSymbol<VisionFrameCallback>,
    user_data: *mut c_void,
}

impl FirmwarePlugin {
    pub fn new(path: impl AsFilename, color: yass_corelib::TeamColor) -> FirmwarePlugin {
        unsafe {
            let library = Rc::new(libloading::Library::new(path).unwrap());
            let frame = library
                .get::<FirmwareFrameCallback>(b"yass_run_firmware_frame\0")
                .unwrap()
                .into_raw();
            let load = library
                .get::<FirmwareLoadCallback>(b"yass_load_firmware_plugin\0")
                .unwrap();
            let user_data = load(color);
            Self {
                library,
                frame,
                user_data,
            }
        }
    }
}

impl Drop for FirmwarePlugin {
    fn drop(&mut self) {
        unsafe {
            let unload: Symbol<FirmwareUnloadCallback> =
                self.library.get(b"yass_unload_firmware_plugin").unwrap();
            unload(self.user_data);
        }
    }
}

impl VisionPlugin {
    pub fn new(path: impl AsFilename) -> VisionPlugin {
        unsafe {
            let library = Rc::new(libloading::Library::new(path).unwrap());
            let frame = library
                .get::<VisionFrameCallback>(b"yass_run_vision_frame\0")
                .unwrap()
                .into_raw();
            let load = library
                .get::<VisionLoadCallback>(b"yass_load_vision_plugin\0")
                .unwrap();
            let user_data = load();
            Self {
                library,
                frame,
                user_data,
            }
        }
    }
}

impl Drop for VisionPlugin {
    fn drop(&mut self) {
        unsafe {
            let unload: Symbol<FirmwareUnloadCallback> =
                self.library.get(b"yass_unload_vision_plugin").unwrap();
            unload(self.user_data);
        }
    }
}

impl PluginManager {
    pub fn new() -> PluginManager {
        PluginManager {
            blue_firmware_plugin: None,
            yellow_firmware_plugin: None,
            vision_plugins: vec![]
        }
    }

    pub fn load_firmware_plugin(&mut self, path: impl AsFilename, color: yass_corelib::TeamColor) {
        match color {
            yass_corelib::TeamColor::Blue => {
                self.blue_firmware_plugin = Some(FirmwarePlugin::new(path, color))
            }
            yass_corelib::TeamColor::Yellow => {
                self.yellow_firmware_plugin = Some(FirmwarePlugin::new(path, color))
            }
        }
    }

    pub fn load_vision_plugin(&mut self, path: impl AsFilename) {
        self.vision_plugins.push(VisionPlugin::new(path));
    }

    pub fn run_blue_firmware(
        &self,
        state: yass_corelib::SimulationState,
        delta_t_us: u64,
        commands: &mut [RobotCommand; 16],
    ) {
        if let Some(plugin) = self.blue_firmware_plugin.as_ref() {
            unsafe {
                plugin.frame.deref()(state, delta_t_us, plugin.user_data, commands);
            }
        }
    }

    pub fn run_yellow_firmware(
        &self,
        state: yass_corelib::SimulationState,
        delta_t_us: u64,
        commands: &mut [RobotCommand; 16],
    ) {
        if let Some(plugin) = self.yellow_firmware_plugin.as_ref() {
            unsafe {
                plugin.frame.deref()(state, delta_t_us, plugin.user_data, commands);
            }
        }
    }

    pub fn run_vision_plugins(&self, state: yass_corelib::SimulationState) {
        for plugin in self.vision_plugins.iter() {
            unsafe {
                plugin.frame.deref()(state, plugin.user_data);
            }
        }
    }
}
