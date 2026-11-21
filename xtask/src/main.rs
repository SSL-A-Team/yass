//! Project automation tasks. Run with `cargo xtask <command>`.

use std::env;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

type Result<T = ()> = std::result::Result<T, Box<dyn std::error::Error>>;

const USAGE: &str = "\
Usage: cargo xtask <command>

Commands:
  headers [OUT_DIR]  Generate corelib C, C++ and Cython headers
                     (default OUT_DIR: target/release/include)
  deb                Build the workspace in release mode, generate headers,
                     and package yass as a .deb";

/// (cbindgen language name, header file extension)
const HEADER_LANGUAGES: [(&str, &str); 3] = [("c", "h"), ("cxx", "hpp"), ("cython", "pxd")];

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    let result = match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["headers"] => generate_headers(&release_dir().join("include")),
        ["headers", out_dir] => generate_headers(Path::new(out_dir)),
        ["deb"] => build_deb(),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask lives in a workspace subdirectory")
        .to_path_buf()
}

/// The release profile output directory, honoring CARGO_TARGET_DIR like cargo and cargo-deb do.
fn release_dir() -> PathBuf {
    let target_dir = env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| workspace_root().join("target"));
    workspace_root().join(target_dir).join("release")
}

fn generate_headers(out_dir: &Path) -> Result {
    let corelib_dir = workspace_root().join("corelib");
    std::fs::create_dir_all(out_dir)?;
    for (lang, ext) in HEADER_LANGUAGES {
        let config_path = corelib_dir.join(format!("cbindgen_{lang}.toml"));
        let config = cbindgen::Config::from_file(&config_path)
            .map_err(|e| format!("loading {}: {e}", config_path.display()))?;
        let output_path = out_dir.join(format!("yass_corelib.{ext}"));
        cbindgen::Builder::new()
            .with_crate(&corelib_dir)
            .with_config(config)
            .generate()
            .map_err(|e| format!("generating {lang} bindings: {e}"))?
            .write_to_file(&output_path);
        println!("Generated {}", output_path.display());
    }
    Ok(())
}

fn build_deb() -> Result {
    cargo(&["build", "--release", "--workspace", "--exclude", "xtask"])?;
    generate_headers(&release_dir().join("include"))?;
    cargo(&["deb", "-p", "yass", "--no-build"])
}

fn cargo(args: &[&str]) -> Result {
    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    let status = Command::new(&cargo)
        .args(args)
        .current_dir(workspace_root())
        .status()
        .map_err(|e| format!("running {cargo}: {e}"))?;
    if !status.success() {
        return Err(format!("`cargo {}` failed with {status}", args.join(" ")).into());
    }
    Ok(())
}
