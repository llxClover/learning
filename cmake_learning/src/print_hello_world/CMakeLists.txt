cmake_minimum_required(VERSION 3.0)

project(print_hello_world)

set(CMAKE_CXX_STANDARD 11)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_library(${PROJECT_NAME} SHARED src/print_hello_world.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(hello_world src/print_hello_world.cpp)

target_link_libraries(hello_world ${PROJECT_NAME})
