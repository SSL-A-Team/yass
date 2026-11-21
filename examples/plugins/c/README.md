# Example C Plugins

## Building the example

To build the C example plugins, run the following commands from this directory.

```shell
cmake -B build .
cmake --build build
```

## Using the example plugins

To use these example plugins, create a config file referencing the built plugins.

```yaml
blue_team:
  firmware:
    lib: /PATH_TO_REPO/examples/plugins/cpp/build/libexample_c_plugins.so
    name: example_firmware
yellow_team:
  firmware:
    lib: /PATH_TO_REPO/examples/plugins/cpp/build/libexample_c_plugins.so
    name: example_firmware
vision_plugins:
- lib: /PATH_TO_REPO/examples/plugins/cpp/build/libexample_c_plugins.so
  name: example_vision
```

Then, run yass with the config file you created above.

```shell
yass_headless -c <path to config file>
```
