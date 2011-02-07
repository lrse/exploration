#execute_process(COMMAND ./generate)

file(GLOB GSLWRAP_SRCS *.cpp)
add_library(gslwrap ${GSLWRAP_SRCS})
include_directories(${PROJECT_SOURCE_DIR})
