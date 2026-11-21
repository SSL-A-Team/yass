cmake_minimum_required(VERSION 3.12)
project(ExampleCxxPlugin LANGUAGES CXX)

add_library(example_cxx_plugins SHARED 
    src/example_firmware.cpp
    src/example_vision.cpp
)

target_include_directories(example_cxx_plugins PUBLIC 
    "/usr/include/yass"
)
