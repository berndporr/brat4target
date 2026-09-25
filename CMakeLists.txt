cmake_minimum_required(VERSION 3.25.0)

project(brat4 LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)

add_subdirectory(lib)
add_subdirectory(test)

include(GNUInstallDirs)

include_directories(
        ${CMAKE_SOURCE_DIR}
        ${TURBOJPEG_INCLUDE_DIRS}
        ${LIBCAMERA_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
)

add_compile_options(-Wall -pedantic)

enable_testing()

pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  message("detected architecture aarch64, building executables")
  add_executable(reactive reactive.cpp)
  target_link_libraries(reactive ${OpenCV_LIBS} box2d zetabot c1lidarrpi brat gpiod PkgConfig::LIBCAMERA LSM6DSOX_Rpi i2c)
endif()
