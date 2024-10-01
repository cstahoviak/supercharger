cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(supercharger CXX)

# Create a macro-defined option with a default value of OFF
option(DEBUG_INFO "Turn on Debug Info" OFF)
IF(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES Debug)
  set(DEBUG_INFO ON)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES Debug)

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "DEBUG_INFO: ${DEBUG_INFO}")

# NOTE: Need to use GLOB_RECURSE to grab files in subdiirectories
# file(GLOB_RECURSE LIB_SOURCES cpp/src/*.cpp)
# file(GLOB_RECURSE LIB_HEADERS cpp/include/*.h)

include_directories("cpp/include")

set(LIB_SOURCES
  "cpp/src/algorithm/algorithm.cpp"
  "cpp/src/algorithm/astar.cpp"
  "cpp/src/algorithm/dijkstras.cpp"
  "cpp/src/algorithm/naive.cpp"
  "cpp/src/optimizer/naive.cpp"
  "cpp/src/optimizer/nloptimizer.cpp"
  "cpp/src/optimizer/optimizer.cpp"
  "cpp/src/network.cpp"
  "cpp/src/node.cpp"
  "cpp/src/planner.cpp"
)

set(LIB_HEADERS
  "cpp/include/algorithm/algorithm.h"
  "cpp/include/algorithm/astar.h"
  "cpp/include/algorithm/dijkstras.h"
  "cpp/include/algorithm/naive.h"
  "cpp/include/optimizer/naive.h"
  "cpp/include/optimizer/nloptimizer.h"
  "cpp/include/optimizer/optimizer.h"
  "cpp/include/logging.h"
  "cpp/include/math.h"
  "cpp/include/network.h"
  "cpp/include/node.h"
  "cpp/include/planner.h"
)

find_package(NLopt REQUIRED)

# Create the superchargerlib
add_library(superchargerlib ${LIB_SOURCES} ${LIB_HEADERS})
# NOTE: This is required by the python bindings (specifically bindings related
# to overloads of operator<<), but I don't really understand why?
set_property(TARGET superchargerlib PROPERTY POSITION_INDEPENDENT_CODE ON)  #-fPIC

# Create the supercharger app (add project executable)
add_executable(${PROJECT_NAME} "cpp/app/supercharger.cpp")

# Add NLOpt
# TODO: Haven't gotten it to work with FetchContent yet.
# include(FetchContent)
# FetchContent_Declare(
#   NLopt
#   URL https://github.com/stevengj/nlopt/archive/refs/tags/v2.8.0.tar.gz
# )
# FetchContent_MakeAvailable(NLopt)

# Link our library against the app
target_link_libraries(${PROJECT_NAME}
  superchargerlib
  nlopt
)

if( DEBUG_INFO )
  # Add Debug info to both library and app
  target_compile_definitions(superchargerlib PRIVATE DEBUG_INFO)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_INFO)
endif()

# Add googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  # URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
)
FetchContent_MakeAvailable(googletest)

# Tell CMake that we'll have unit tests 
enable_testing()

# Add the tests
add_subdirectory(cpp/test)

# Add the python bindings
message(STATUS "Building the python bindings.")
add_subdirectory(cpp/bindings)