cmake_minimum_required(VERSION 3.20.0)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(Tests LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 17)

include_directories(.. ../src)

add_executable(test_worldbuilder test_worldbuilder.cpp)
target_link_libraries(test_worldbuilder brat)

add_executable(test_simulation test_simulation.cpp)
target_link_libraries(test_simulation brat)

add_executable(test_tasks test_tasks.cpp)
target_link_libraries(test_tasks brat)
