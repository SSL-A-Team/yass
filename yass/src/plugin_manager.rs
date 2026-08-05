use core::ops::{Deref, Drop};
use libloading::os;
use libloading::{AsFilename, Error, Library, Symbol};
use std::ffi::{CString, c_void};
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
    name: String,
    frame: RawSymbol<FirmwareFrameCallback>,
    user_data: *mut c_void,
}

struct VisionPlugin {
    library: Rc<Library>,
    name: String,
    frame: RawSymbol<VisionFrameCallback>,
    user_data: *mut c_void,
}

impl FirmwarePlugin {
    pub fn new(
        path: impl AsFilename,
        name: String,
        color: yass_corelib::TeamColor,
        config: String,
    ) -> Result<FirmwarePlugin, Box<dyn std::error::Error>> {
        unsafe {
            let library = Rc::new(libloading::Library::new(path)?);
            let frame = library
                .get::<FirmwareFrameCallback>(format!("yass_run_{name}"))?
                .into_raw();
            let load = library.get::<FirmwareLoadCallback>(format!("yass_load_{name}"))?;
            let user_data = load(color, CString::new(config)?.as_ptr());
            Ok(Self {
                library,
                name,
                frame,
                user_data,
            })
        }
    }
}

impl Drop for FirmwarePlugin {
    fn drop(&mut self) {
        unsafe {
            let name = self.name.clone();
            let unload: Result<Symbol<FirmwareUnloadCallback>, Error> =
                self.library.get(format!("yass_unload_{name}"));
            if let Ok(func) = unload {
                func(self.user_data);
            }
        }
    }
}

impl VisionPlugin {
    pub fn new(
        path: impl AsFilename,
        name: String,
        config: String,
    ) -> Result<VisionPlugin, Box<dyn std::error::Error>> {
        unsafe {
            let library = Rc::new(libloading::Library::new(path)?);
            let frame = library
                .get::<VisionFrameCallback>(format!("yass_run_{name}"))?
                .into_raw();
            let load = library.get::<VisionLoadCallback>(format!("yass_load_{name}"))?;
            let user_data = load(CString::new(config)?.as_ptr());
            Ok(Self {
                library,
                name,
                frame,
                user_data,
            })
        }
    }
}

impl Drop for VisionPlugin {
    fn drop(&mut self) {
        unsafe {
            let name = self.name.clone();
            let unload: Result<Symbol<FirmwareUnloadCallback>, Error> =
                self.library.get(format!("yass_unload_{name}"));
            if let Ok(func) = unload {
                func(self.user_data);
            }
        }
    }
}

impl PluginManager {
    pub fn new() -> PluginManager {
        PluginManager {
            blue_firmware_plugin: None,
            yellow_firmware_plugin: None,
            vision_plugins: vec![],
        }
    }

    pub fn print_plugin_info(&self) {
        println!("Blue Firmware Plugin:");
        match self.blue_firmware_plugin.as_ref() {
            None => println!("   None"),
            Some(p) => println!("   {} (from {:?})", p.name, p.library),
        }
        println!("Yellow Firmware Plugin:");
        match self.yellow_firmware_plugin.as_ref() {
            None => println!("   None"),
            Some(p) => println!("   {} (from {:?})", p.name, p.library),
        }
        println!("Vision Plugins:");
        if self.vision_plugins.is_empty() {
            println!("   None")
        } else {
            for p in self.vision_plugins.iter() {
                println!("   {} (from {:?}", p.name, p.library);
            }
        }
    }

    pub fn load_firmware_plugin(
        &mut self,
        path: impl AsFilename,
        name: String,
        color: yass_corelib::TeamColor,
        config: String,
    ) -> Result<(), Box<dyn std::error::Error>> {
        match color {
            yass_corelib::TeamColor::Blue => {
                self.blue_firmware_plugin = Some(FirmwarePlugin::new(path, name, color, config)?)
            }
            yass_corelib::TeamColor::Yellow => {
                self.yellow_firmware_plugin = Some(FirmwarePlugin::new(path, name, color, config)?)
            }
        }
        Ok(())
    }

    pub fn load_vision_plugin(
        &mut self,
        path: impl AsFilename,
        name: String,
        config: String,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.vision_plugins
            .push(VisionPlugin::new(path, name, config)?);
        Ok(())
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
