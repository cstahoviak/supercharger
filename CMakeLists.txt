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

# NOTE: Need to use GLOB_RECURSE to grab files in subdirectories
# file(GLOB_RECURSE LIB_SOURCES cpp/src/supercharger/*.cpp)
# file(GLOB_RECURSE LIB_HEADERS cpp/include/supercharger/*.h)

include_directories("cpp/include")

set(LIB_SOURCES
  "cpp/src/supercharger/algorithm/astar.cpp"
  "cpp/src/supercharger/algorithm/dijkstras.cpp"
  "cpp/src/supercharger/algorithm/naive.cpp"
  "cpp/src/supercharger/algorithm/planner.cpp"
  "cpp/src/supercharger/optimize/naive.cpp"
  "cpp/src/supercharger/optimize/nlopt.cpp"
  "cpp/src/supercharger/optimize/optimizer.cpp"
  "cpp/src/supercharger/network.cpp"
  "cpp/src/supercharger/node.cpp"
  "cpp/src/supercharger/supercharger.cpp"
  "cpp/src/supercharger/types.cpp"
)

set(LIB_HEADERS
  "cpp/include/supercharger/algorithm/astar.h"
  "cpp/include/supercharger/algorithm/dijkstras.h"
  "cpp/include/supercharger/algorithm/naive.h"
  "cpp/include/supercharger/algorithm/planner.h"
  "cpp/include/supercharger/optimize/naive.h"
  "cpp/include/supercharger/optimize/nlopt.h"
  "cpp/include/supercharger/optimize/optimizer.h"
  "cpp/include/supercharger/logging.h"
  "cpp/include/supercharger/math.h"
  "cpp/include/supercharger/network.h"
  "cpp/include/supercharger/node.h"
  "cpp/include/supercharger/supercharger.h"
  "cpp/include/supercharger/types.h"
  "cpp/include/supercharger/utils.h"
)

# Create the superchargerlib
add_library(superchargerlib ${LIB_SOURCES} ${LIB_HEADERS})
# NOTE: This is required by the python bindings (specifically bindings related
# to overloads of operator<<), but I don't really understand why?
set_property(TARGET superchargerlib PROPERTY POSITION_INDEPENDENT_CODE ON)  #-fPIC

if( DEBUG_INFO )
  # Add Debug info to our library
  target_compile_definitions(superchargerlib PRIVATE DEBUG_INFO)
endif()

# Add third party libraries
add_subdirectory(cpp/thirdparty)

# Build all app executables
set(APP_SOURCES
  "cpp/app/algorithm_comparison.cpp"
  "cpp/app/supercharger.cpp"
)
foreach( testsourcefile ${APP_SOURCES} )
    # Cut off the file extension and directory path
    get_filename_component( testname ${testsourcefile} NAME_WE )
    add_executable( ${testname} ${testsourcefile} )
    # Linking
    target_link_libraries( ${testname} 
      superchargerlib
      nlopt
      # glog::glog
    )
    # Add debug info for each target
    if( DEBUG_INFO )
        target_compile_definitions(${testname} PRIVATE DEBUG_INFO)
    endif()
endforeach( testsourcefile ${APP_SOURCES} )

# Tell CMake that we'll have unit tests 
enable_testing()

# Add the tests
add_subdirectory(cpp/test)

# Add the python bindings
message(STATUS "Building the python bindings.")
add_subdirectory(cpp/bindings)