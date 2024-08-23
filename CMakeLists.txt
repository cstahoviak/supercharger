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

include_directories(include)

# NOTE: Need to use GLOB_RECURSE to grab files in subdiirectories
# file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
# file(GLOB_RECURSE LIB_HEADERS include/*.h)

set(LIB_SOURCES
  "src/algorithm/algorithm.cpp"
  "src/algorithm/brute_force.cpp"
  "src/algorithm/dijkstras.cpp"
  "src/network.cpp"
  "src/planner.cpp"
)

set(LIB_HEADERS
  "include/algorithm/algorithm.h"
  "include/algorithm/brute_force.h"
  "include/algorithm/dijkstras.h"
  "include/logging.h"
  "include/math.h"
  "include/network.h"
  "include/planner.h"
  "include/stop.h"
)

# Create the superchargerlib
add_library(superchargerlib ${LIB_SOURCES} ${LIB_HEADERS})

# Create the supercharger app (add project executable)
file( GLOB APP_SOURCES app/*.cpp )
add_executable(${PROJECT_NAME} ${APP_SOURCES})

# Link our library against the app
target_link_libraries(${PROJECT_NAME} superchargerlib)

if( DEBUG_INFO )
  # Add Debug info to both library and app
  target_compile_definitions(superchargerlib PRIVATE DEBUG_INFO)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_INFO)
endif()

# Add googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)

# Tell CMake that we'll have unit tests 
enable_testing()

# Add the tests
add_subdirectory(test)
