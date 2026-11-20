use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn find_default_configs() -> Vec<(String, PathBuf)> {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let configs_dir = manifest_dir.join("assets").join("default_configs");
    println!("cargo:rerun-if-changed={}", configs_dir.display());

    let mut entries: Vec<(String, PathBuf)> = fs::read_dir(&configs_dir)
        .unwrap_or_else(|e| panic!("Failed to read {}: {e}", configs_dir.display()))
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| {
            path.is_file()
                && matches!(
                    path.extension().and_then(|e| e.to_str()),
                    Some("yaml") | Some("yml")
                )
        })
        .map(|path| {
            let name = path.file_stem().unwrap().to_str().unwrap().to_owned();
            (name, path)
        })
        .collect();
    entries.sort();
    for pair in entries.windows(2) {
        if pair[0].0 == pair[1].0 {
            panic!("Duplicate default config name: {}", pair[0].0);
        }
    }
    entries
}

fn bake_default_configs() {
    let entries = find_default_configs();

    let mut code = String::from("pub static DEFAULT_CONFIGS: &[(&str, &str)] = &[\n");
    for (name, path) in &entries {
        println!("cargo:rerun-if-changed={}", path.display());
        code.push_str(&format!(
            "    ({name:?}, include_str!({:?})),\n",
            path.display().to_string()
        ));
    }
    code.push_str("];\n");

    let out_path = Path::new(&env::var("OUT_DIR").unwrap()).join("default_configs.rs");
    fs::write(out_path, code).unwrap();
}

fn main() {
    bake_default_configs();
}
