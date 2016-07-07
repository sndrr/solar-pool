set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)

cmake_minimum_required (VERSION 2.6)
project (solar-pool C CXX)

# The version number.
set (solar-pool_VERSION_MAJOR 0)
set (solar-pool_VERSION_MINOR 5)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
        "${PROJECT_SOURCE_DIR}/solar-pool.h.in"
        "${PROJECT_BINARY_DIR}/solar-pool.h"
)

# add the binary tree to the search path for include files
include_directories("${PROJECT_BINARY_DIR}")

add_executable(solar-pool solar-pool.cpp)