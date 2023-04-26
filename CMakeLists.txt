cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(adaptive_road LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(UNIX AND NOT APPLE AND CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "Using gcc/g++ compiler (Linux)")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -frounding-math -Wall -Wno-sign-compare -O3 ")
endif()

if(APPLE)
	message(STATUS "Using Clang compiler on (Apple)")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-sign-compare -Wno-deprecated-declarations -O3")
endif()


include(cmake/CGAL.cmake)
include(cmake/Eigen3.cmake)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h")

find_package(CGAL PATHS /usr/include/CGAL)

add_executable(adaptive_road ${SOURCES} ${HEADERS} )
target_link_libraries(adaptive_road ${LIBRARIES} CGAL::CGAL)
