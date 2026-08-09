include(GNUInstallDirs)

set (CMAKE_CXX_STANDARD 17)

file(GLOB SRC "*.cpp")

file(GLOB HEAD "*.h")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

add_library(brat ${SRC} ${HEAD})

target_link_libraries(brat box2d ${OpenCV_LIBS} pthread)

message("box2d installed ${BOX2D_LIBS}")

set_target_properties(brat PROPERTIES POSITION_INDEPENDENT_CODE TRUE 

PUBLIC_HEADER "${HEAD}")

install(TARGETS brat 
	EXPORT brat-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message("library and archive destination " ${CMAKE_INSTALL_LIBDIR})

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
