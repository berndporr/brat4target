cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(QCustomplotTestApp
	DESCRIPTION "Tests scatterplots with QCustomplot"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall")

find_package(PkgConfig)
find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
	${CMAKE_SOURCE_DIR}
)

include(GNUInstallDirs)

add_executable(fakelidarviewer
  window.cpp
  main.cpp
)

target_link_libraries(fakelidarviewer Qt6::Widgets QCustomPlotQt6)
