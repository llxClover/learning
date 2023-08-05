cmake_minimum_required(VERSION 3.0)

project(eigen_learning)

set(CMAKE_CXX_STANDARD 17)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# # 生成动态库
add_library(${PROJECT_NAME} SHARED src/eigen_learning.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/include/eigen3")
add_executable(eigen_test src/eigen_learning.cpp)

target_link_libraries(eigen_learning ${PROJECT_SOURCE_DIR}/lib/lib${PROJECT_NAME}.so)


# cmake报错:主要是生成的可执行文件名称和生成的库文件名称是一样而导致的错误
# add_executable cannot create target "eigen_learning" because another target
#   with the same name already exists.  The existing target is a shared library
#   created in source directory "/home/llx/learning/eigen_learning".  See
#   documentation for policy CMP0002 for more details.
