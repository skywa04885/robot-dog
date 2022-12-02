project(backend)
cmake_minimum_required(VERSION 3.10)

file(GLOB_RECURSE CC_SOURCES ${PROJECT_SOURCE_DIR} *.cc)

set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_CXX_STANDARD 20)

find_package(Boost REQUIRED COMPONENTS log_setup log)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM3 REQUIRED gtkmm-3.0)

include_directories(${GTKMM3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories("./vendor/rplidar_sdk/include")

link_directories(${GTKMM3_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories("./vendor/rplidar_sdk")
add_definitions(${GTKMM3_CFLAGS_OTHER})
add_definitions(-DCL_TARGET_OPENCL_VERSION=300)

add_executable(backend ${CC_SOURCES})

target_link_libraries(backend ${GTKMM3_LIBRARIES})
target_link_libraries(backend ${Boost_LIBRARIES})
target_link_libraries(backend rplidar)