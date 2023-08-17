cmake_minimum_required(VERSION 3.0)

project(nonlinear_optimization_learning)


set(CMAKE_CXX_STANDARD 20)

# 设置g2o编译时的cmake_modules路径,否则会找不到g2o-config.cmake，G2OConfig.cmake
LIST( APPEND CMAKE_MODULE_PATH /home/llx/Downloads/g2o/cmake_modules )

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories("/usr/include/eigen3")

#ceres
find_package(Ceres REQUIRED)
include_directories("usr/local/include/ceres")

#g2o
find_package(G2O REQUIRED)
include_directories("usr/local/include/g2o")

#glog
find_package(GLOG REQUIRED)
include_directories("usr/local/include/glog")

#python
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
target_link_libraries(gauss_newton 
                      gaussNewton 
                      ${OpenCV_LIBS} 
                      ${PYTHON_LIBRARIES})

# ceres_learning.cpp
add_library(ceresLearning SHARED src/ceres_learning.cpp)
add_executable(ceres_learning src/ceres_learning.cpp)
target_link_libraries(ceres_learning 
                      ceresLearning 
                      ${OpenCV_LIBS} 
                      ${PYTHON_LIBRARIES} 
                      ${CERES_LIBRARIES})

# ceres_learning_v1.cpp
add_library(ceresLearningv1 SHARED src/ceres_learning_v1.cpp)
add_executable(ceres_learningv1 src/ceres_learning_v1.cpp)
target_link_libraries(ceres_learningv1 
                      ceresLearningv1 
                      ${OpenCV_LIBS} 
                      ${PYTHON_LIBRARIES} 
                      ${CERES_LIBRARIES})

# g2o_learning.cpp
add_library(g2oLearning SHARED src/g2o_learning.cpp)
add_executable(g2o_learning src/g2o_learning.cpp)
target_link_libraries(g2o_learning 
                      g2oLearning 
                      ${OpenCV_LIBS} 
                      ${PYTHON_LIBRARIES} 
                      ${G2O_CORE_LIBRARIES} 
                      ${G2O_STUFF_LIBRARY} 
                      ${GLOG_LIBRARIES})
