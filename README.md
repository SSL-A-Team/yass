# yass (Yet Another SSL Simulator)

yass is a simulator for [the RoboCup Small Size League](https://ssl.robocup.org/).

This simulator is written in Rust with a plugin-based architecture to prioritize customizability for teams.

## Installation

Download and install the debian archive from [the Releases page](https://github.com/SSL-A-Team/yass/releases).

Once installed, you should have three new programs available:

`yass` runs the simulator with a graphical interface.

`yass_headless` runs the simulator without a graphical interface.

`yass_config_schema` prints the full description of available config fields.

## Usage

All of the options for the simulator are managed through a configuration file. Communication for vision feedback and robot control is handled with plugins. This allows teams to customize how their robots work and how the simulator communicates to your coach system. Each team in the simulator can even run with different plugins.

By default, yass launches with a Division B field and plugins that implement the league-standard protobuf network interfaces.

You can run yass with a Division A field by selecting the `div_a` built-in config.

```shell
yass -c div_a
```

Or, you can pass your own config file to change whatever settings you want.

```shell
yass -c your_config.yaml
```

## Build and Run From Source

This project uses cargo and can be built and run with common cargo commands.

For example:

```shell
cargo run --bin yass_headless
```

## Building the Debian Install Archive

To generate the debian installation archive, run the following command from the repo root folder.

```shell
cargo xtask deb
```
