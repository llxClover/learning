cmake_minimum_required(VERSION 3.0)

project(lio_sam)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             # converts between ROS Image messages and OpenCV images.
             cv_bridge

             # tf
             tf2

             # pcl lib
             pcl_conversions

             # msgs
             std_msgs
             sensor_msgs
             nav_msgs
             geometry_msgs

             # 生成自定义msgs
             message_generation
             # 可视化marker
             visualization_msgs
             )


include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(include ${catkin_LIBRARY_DIRS})

# opencv
find_package(OpenCV REQUIRED QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
link_directories(${EIGEN3_LIBRARY_DIRS})

# pcl
find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# GTSAM
find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

# boost
find_package(Boost REQUIRED COMPONENTS timer)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# glog
find_package(GLOG REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
include_directories("usr/local/include/glog")
link_directories(${GLOG_LIBRARY_DIRS})

# msg
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

# srv
add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

# 生成自定义msg
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

###########
## Build ##
###########
add_executable(${PROJECT_NAME}_feature_extraction_node src/feature_extraction.cpp)
add_dependencies(${PROJECT_NAME}_feature_extraction_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_feature_extraction_node ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES} 
                      ${GLOG_LIBRARIES} 
                      glog::glog)