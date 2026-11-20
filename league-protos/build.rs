use std::io::Result;

fn main() -> Result<()> {
    let protos = [
        "ssl-protocol-defs/proto/simulation/ssl_simulation_robot_control.proto",
        "ssl-protocol-defs/proto/simulation/ssl_simulation_control.proto",
        "ssl-protocol-defs/proto/simulation/ssl_simulation_robot_feedback.proto",
        "ssl-protocol-defs/proto/vision/ssl_vision_wrapper.proto",
    ];

    prost_build::Config::new()
        .default_package_filename("league_protos")
        .compile_protos(&protos, &["ssl-protocol-defs/proto"])?;
    Ok(())
}
