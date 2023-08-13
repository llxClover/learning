cmake_minimum_required(VERSION 3.0)

project(nonlinear_optimization_learning)


set(CMAKE_CXX_STANDARD 17)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories("/usr/include/eigen3")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories("/usr/include/python3.8")
include_directories("/usr/lib/python3/dist-packages/numpy/core/include")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/local/include")

# gauss_newton.cpp
add_library(gaussNewton SHARED src/gauss_newton.cpp)
add_executable(gauss_newton src/gauss_newton.cpp)
target_link_libraries(gauss_newton gaussNewton ${OpenCV_LIBS} ${PYTHON_LIBRARIES})