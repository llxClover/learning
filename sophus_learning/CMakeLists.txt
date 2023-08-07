cmake_minimum_required(VERSION 3.0)

project(sophus_learning)

set(CMAKE_CXX_STANDARD 17)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)



include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/Sophus)
include_directories("/usr/include/eigen3")
include_directories("/usr/local/include")

add_library(sophusLearning SHARED src/sophus_learning.cpp)
add_executable(sophus_learning src/sophus_learning.cpp)
target_link_libraries(sophus_learning sophusLearning fmt)

add_library(trahectoryError SHARED src/trajectory_error.cpp)
add_executable(trahectory_error src/trajectory_error.cpp)
target_link_libraries(trahectory_error sophusLearning fmt)