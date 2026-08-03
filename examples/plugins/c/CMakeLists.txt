cmake_minimum_required(VERSION 3.12)
project(ExampleCPlugin LANGUAGES C)

add_library(example_c_plugin SHARED 
    src/plugin.c
)

target_include_directories(example_c_plugin PUBLIC 
    include
)
