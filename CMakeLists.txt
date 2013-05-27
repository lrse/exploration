cmake_minimum_required(VERSION 2.8)

project(HybNav)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror=return-type -Wfatal-errors")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

option(ENABLE_DISPLAY "Shows GUI (if disabled, sends images to videos)" ON)
option(ENABLE_SYROTEK "Tunes constants for real SyRoTeK arena" OFF)
configure_file(${CMAKE_SOURCE_DIR}/config.h.cmake ${CMAKE_SOURCE_DIR}/config.h)


execute_process(COMMAND ${PROJECT_SOURCE_DIR}/gslwrap/generate WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/gslwrap)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
INCLUDE(UsePlayerC++)

#include(FindPkgConfig)
#pkg_search_module(PLAYER REQUIRED playerc++)

message("found lib ${OpenCV_LIB_DIR}")
include_directories(${GSL_INCLUDE_DIR} ${PROJECT_SOURCE_DIR} ${PLAYER_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${PLAYERCPP_INCLUDE_DIRS})

file(GLOB HybNav_SRCS *.cpp)
SET(SOURCES ${SOURCES} ${HybNav_SRCS})
link_directories(${PLAYERCPP_LIBRARY_DIRS})
add_executable(main ${SOURCES})
target_link_libraries(main gslwrap ${GSL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PLAYER_LDFLAGS} ${PLAYERCPP_LINK_LIBS})

add_subdirectory(gslwrap)
