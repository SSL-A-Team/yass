cmake_minimum_required(VERSION 3.12)
project(ExampleCxxPlugin LANGUAGES CXX)

add_library(example_cxx_plugin SHARED 
    src/plugin.cpp
)

target_include_directories(example_cxx_plugin PUBLIC 
    include
)
