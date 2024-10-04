cmake_minimum_required(VERSION 3.5)

set(TARGET py${PROJECT_NAME})

find_package(pybind11 REQUIRED)

# The SOURCES are the bindings
set(SOURCES
  "algorithm.cpp"
  "math.cpp"
  "network.cpp"
  "node.cpp"
  "optimize.cpp"
  "planner.cpp"

  "module.cpp"
)

pybind11_add_module(${TARGET} SHARED ${SOURCES})

target_include_directories(${TARGET} PRIVATE 
  "${PROJECT_SOURCE_DIR}/cpp/include"
)

target_link_libraries(${TARGET} PRIVATE
  superchargerlib
  nlopt
)

# Install the bindings to the python directory
set(OUTPUT_PATH "${PROJECT_SOURCE_DIR}/python/src/supercharger")
set_target_properties(${TARGET} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_PATH}/$<0:>
  LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_PATH}/$<0:>
)