cmake_minimum_required(VERSION 3.25.0)

project(brat LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_BUILD_TYPE Debug)

set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")
add_subdirectory(test)
add_subdirectory(targetloc)

option(BUILD_FASTDDS "Building FastDDS directory" 0)

include(GNUInstallDirs)

pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  message("detected architecture aarch64, building executables")
  add_executable(reactive reactive.cpp)
  target_link_libraries(reactive ${OpenCV_LIBS} box2d zetabot c1lidarrpi brat gpiod)
endif()

add_compile_options(-Wall -pedantic)

# enable_testing()
