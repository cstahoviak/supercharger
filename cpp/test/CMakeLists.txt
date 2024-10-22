cmake_minimum_required(VERSION 3.10.2)

set(TEST_NAME test_${PROJECT_NAME})

set(TEST_SOURCES
  "algorithm/test_dijkstras.cpp"
  "optimize/test_nlopt.cpp"
  "test_math.cpp"
)

# Create the test executable
add_executable(${TEST_NAME} ${TEST_SOURCES})

# Linking our tests against both gtest and our library
target_link_libraries(${TEST_NAME} PUBLIC
  gtest_main
  superchargerlib
  nlopt
)

# Add the tests
add_test(
  NAME ${TEST_NAME}
  COMMAND ${TEST_NAME}
)