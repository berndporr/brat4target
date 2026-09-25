include(GNUInstallDirs)

set (CMAKE_CXX_STANDARD 17)

file(GLOB SRC "*.cpp")

file(GLOB HEAD "*.h")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Wall -pedantic)

find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

include_directories(
        ${CMAKE_SOURCE_DIR}
        ${TURBOJPEG_INCLUDE_DIRS}
        ${LIBCAMERA_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
)

add_library(brat ${SRC} ${HEAD})

target_link_libraries(brat box2d ${OpenCV_LIBS} pthread)
target_link_libraries(brat PkgConfig::LIBCAMERA)
target_link_libraries(brat cam2opencv)
target_link_libraries(brat ${TURBOJPEG_LINK_LIBRARIES})

message("box2d installed ${BOX2D_LIBS}")

set_target_properties(brat PROPERTIES POSITION_INDEPENDENT_CODE TRUE 

PUBLIC_HEADER "${HEAD}")

install(TARGETS brat 
	EXPORT brat-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message("library and archive destination " ${CMAKE_INSTALL_LIBDIR})

