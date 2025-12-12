cmake_minimum_required(VERSION 3.16)
project(LaneLock VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(LANELOCK_BUILD_TESTS "Build the Google Test suites" ON)
option(LANELOCK_BUILD_APPS "Build the lanelock_sim command-line tool" ON)

add_library(lanelock_core
  src/road.cpp
  src/vehicle.cpp
  src/noise.cpp
  src/sensors.cpp
  src/actuator.cpp
  src/pid.cpp
  src/controller.cpp
  src/metrics.cpp
  src/simulator.cpp
  src/scenario.cpp
)
add_library(lanelock::core ALIAS lanelock_core)
target_include_directories(lanelock_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
  target_compile_options(lanelock_core PRIVATE /W4)
else()
  target_compile_options(lanelock_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
endif()
