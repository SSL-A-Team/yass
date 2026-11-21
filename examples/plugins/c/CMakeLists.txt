cmake_minimum_required(VERSION 3.12)
project(ExampleCPlugin LANGUAGES C)

add_library(example_c_plugins SHARED 
    src/example_firmware.c
    src/example_vision.c
)

target_include_directories(example_c_plugins PUBLIC 
    "/usr/include/yass"
)
