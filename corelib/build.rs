use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let package_name = env::var("CARGO_PKG_NAME").unwrap();

    let cbindgen_languages: [(&str, PathBuf); 3] = [
        (
            "c",
            PathBuf::from(&crate_dir).join(format!("{}.h", package_name)),
        ),
        (
            "cxx",
            PathBuf::from(&crate_dir).join(format!("{}.hpp", package_name)),
        ),
        (
            "cython",
            PathBuf::from(&crate_dir).join(format!("{}.pxd", package_name)),
        )
    ];

    for (lang, output_path) in cbindgen_languages {
        let config_path = PathBuf::from(&crate_dir).join(format!("cbindgen_{lang}.toml"));
        let config = cbindgen::Config::from_file(config_path)
            .expect(&format!("Could not load cbindgen config for {lang}"));
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(config)
            .generate()
            .expect(&format!("Unable to generate bindings for {lang}"))
            .write_to_file(output_path);
    }
}
