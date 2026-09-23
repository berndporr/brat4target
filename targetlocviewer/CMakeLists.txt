cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(TargetLocViewer
	DESCRIPTION "QT app to visulalise the 3D detection"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall")

find_package(PkgConfig)
find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot)

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

message(STATUS "opencv:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "turbojpeg:")
message(STATUS "    version: ${TURBOJPEG_VERSION}")
message(STATUS "    libraries: ${TURBOJPEG_LINK_LIBRARIES}")
message(STATUS "    include path: ${TURBOJPEG_INCLUDE_DIRS}")

include_directories(
	${CMAKE_SOURCE_DIR}
	${TURBOJPEG_INCLUDE_DIRS}
	${LIBCAMERA_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	..
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include(GNUInstallDirs)

add_executable(targetlocviewer
  window.cpp
  main.cpp
  ../targetloc.cpp
  ../targetdet.cpp
)

target_link_libraries(targetlocviewer Qt6::Widgets QCustomPlotQt6)
target_link_libraries(targetlocviewer PkgConfig::LIBCAMERA)
target_link_libraries(targetlocviewer ${OpenCV_LIBS})
target_link_libraries(targetlocviewer cam2opencv)
target_link_libraries(targetlocviewer ${TURBOJPEG_LINK_LIBRARIES})
