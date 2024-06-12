cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(template C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#############################
# Define the library
#############################

add_library(lidar INTERFACE)

target_include_directories(lidar INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

add_subdirectory(src)

#############################
# Add the example
#############################

add_subdirectory(example)
