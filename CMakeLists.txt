cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(supercharger CXX)

include_directories(include)

file(GLOB LIB_SOURCES src/*.cpp)
file(GLOB LIB_HEADERS include/*.h)

# Create the superchargerlib
add_library(superchargerlib ${LIB_SOURCES} ${LIB_HEADERS})

# Create the supercharger app (add project executable)
file( GLOB APP_SOURCES app/*.cpp )
add_executable(${PROJECT_NAME} ${APP_SOURCES})

# Link our library against the app
target_link_libraries(${PROJECT_NAME} superchargerlib)
