cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)
project(HybNav)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

execute_process(COMMAND ${PROJECT_SOURCE_DIR}/gslwrap/generate WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/gslwrap)
add_subdirectory(gslwrap)
file(GLOB HybNav_SRCS *.cpp)
SET(SOURCES ${SOURCES} ${HybNav_SRCS})
add_executable(main ${SOURCES})
#add_executable(main main.cpp exabot.cpp plotter.cpp metric_map.cpp)

pkg_search_module(PLAYER REQUIRED playerc++)
pkg_search_module(GSL REQUIRED gsl)
pkg_search_module(OPENCV REQUIRED opencv)

include_directories(${PLAYER_INCLUDE_DIRS} ${GSL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
link_directories(${PLAYER_LIBRARY_DIRS} ${GSL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
target_link_libraries(main gslwrap ${PLAYER_LIBRARIES} ${PLAYER_LDFLAGS} ${GSL_LIBRARIES} ${GSL_LDFLAGS} ${OPENCV_LIBRARIES} ${OPENCV_LDFLAGS} -lboost_system)



