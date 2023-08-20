#pragma once
#ifndef _UTILITY_H
#define _UTILITY_H

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
// eigen
#include <Eigen/Core>

// system
#include <algorithm>
#include <array>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

typedef pcl::PointXYZI PointType;

enum class SensorType { RSLIDAR, VELODYNE };

class ParamServer {

public:
  ros::NodeHandle nh;

  std::string robot_id;

  // topics
  std::string point_cloud_topic;
  std::string imu_topic;
  std::string odom_topic;
  std::string rtk_topic;

  // frames
  std::string lidar_frame;
  std::string imu_frame;
  std::string rtk_frame;
  std::string map_frame;
  std::string odom_frame;
  std::string base_link_frame;

  // RTK setting
  bool use_imu_heading_init;
  bool use_gps_elevation;
  float gps_cov_threshold;
  float pose_cov_threshold;

  // save pcd
  bool save_pcd;
  std::string save_pcd_directory;

  // lidar configuration
  SensorType sensor;
  int N_SCAN;
  int HORIZON_RESOLUTION;
  int downsample_rate;
  float lidar_min_range;
  float lidar_max_range;

  // imu configuration
  float imu_acc_noise;
  float imu_acc_bias;
  float imu_gyr_noise;
  float imu_gyr_bias;
  float imu_gravity;
  float imu_rpy_weight;
  std::vector<double> ext_rot_v;
  std::vector<double> ext_rpy_v;
  std::vector<double> ext_trans_v;
  Eigen::Matrix3d ext_rot;
  Eigen::Matrix3d ext_rpy;
  Eigen::Matrix3d ext_trans;
  Eigen::Quaterniond ext_q;

  // loam
  float edge_threshold;
  float surf_threshold;
  int edge_feature_min_valid_num;
  int surf_feature_min_valid_num;

  // voxel filter
  float odom_surf_leaf_size;
  float mapping_corner_leaf_size;
  float mapping_surf_leaf_size;

  // tollerance
  float z_tollerance;
  float rotation_tollerance;

  // cpu
  int core_num;
  double mapping_process_interval;

  // surrounding map
  float surrounding_keyframe_adding_dist_threshold;
  float surrounding_keyframe_adding_angle_threshold;
  float surrounding_keyframe_density;
  float surrounding_keyframe_search_radius;

  // loop closure
  bool loop_closure_enable_flag;
  float loop_closure_frequency;
  int surrounding_keyframe_size;
  float history_keyframe_search_radius;
  float history_keyframe_search_time_diff;
  int history_keyframe_search_num;
  float history_keyframe_fitness_score;

  // global map visualization radius
  float global_map_visualization_search_radius;
  float global_map_visualization_pose_density;
  float global_map_visualization_leaf_size;

public:
  ParamServer() {
    nh.param<std::string>("/robot_id", robot_id, "roboat");
    // topic
    nh.param<std::string>("lio_sam/point_clould_topic", point_cloud_topic,
                          "points_raw");
    nh.param<std::string>("lio_sam/imu_topic", imu_topic, "imu_correct");
    nh.param<std::string>("lio_sam/odom_topic", odom_topic, "odometry/imu");
    nh.param<std::string>("lio_sam/rtk_topic", rtk_topic, "odometry/gps");
    // frame
    nh.param<std::string>("lio_sam/lidar_frame", lidar_frame, "base_link");
    nh.param<std::string>("lio_sam/base_link_frame", base_link_frame,
                          "base_link");
    nh.param<std::string>("lio_sam/odometry_frame", odom_frame, "odom");
    nh.param<std::string>("lio_sam/map_frame", map_frame, "map");

    nh.param<bool>("lio_sam/use_imu_heading_init", use_imu_heading_init, false);
    nh.param<bool>("lio_sam/use_gps_elevation", use_gps_elevation, false);
    nh.param<float>("lio_sam/gps_cov_threshold", gps_cov_threshold, 2.0);
    nh.param<float>("lio_sam/pose_cov_threshold", pose_cov_threshold, 25.0);

    nh.param<bool>("lio_sam/save_pcd", save_pcd, false);
    nh.param<std::string>("lio_sam/save_pcd_directory", save_pcd_directory,
                          "Downloads/LIO-SAM");

    std::string lidar_type;
    nh.param<std::string>("lio_sam/sensor", lidar_type, "VELODYNE");
    if (lidar_type == "VELODYNE") {
      sensor = SensorType::VELODYNE;
    } else if (lidar_type == "RSLIDAR") {
      sensor = SensorType::RSLIDAR;
    } else {
      ROS_ERROR_STREAM(
          "=== 未知的激光雷达型号,请重新配置参数(VELODYNE,RSLIDAR)");
      ros::shutdown();
    }

    nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    nh.param<int>("lio_sam/HORIZON_RESOLUTION", HORIZON_RESOLUTION, 1800);
    nh.param<int>("lio_sam/downsample_rate", downsample_rate, 1);
    nh.param<float>("lio_sam/lidar_min_range", lidar_min_range, 1.0);
    nh.param<float>("lio_sam/lidar_max_range", lidar_max_range, 100.0);

    nh.param<float>("lio_sam/imu_acc_noise", imu_acc_noise, 0.01);
    nh.param<float>("lio_sam/imu_gyr_noise", imu_gyr_noise, 0.001);
    nh.param<float>("lio_sam/imu_acc_bias", imu_acc_bias, 0.0002);
    nh.param<float>("lio_sam/imu_gyr_bias", imu_gyr_bias, 0.00003);
    nh.param<float>("lio_sam/imu_gravity", imu_gravity, 9.80511);
    nh.param<float>("lio_sam/imu_rpy_weight", imu_rpy_weight, 0.01);
    nh.param<std::vector<double>>("lio_sam/ext_rot_v", ext_rot_v,
                                  std::vector<double>());
    nh.param<std::vector<double>>("lio_sam/ext_rpy_v", ext_rpy_v,
                                  std::vector<double>());
    nh.param<std::vector<double>>("lio_sam/ext_trans_v", ext_trans_v,
                                  std::vector<double>());
    ext_rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        ext_rot_v.data(), 3, 3);
    ext_rpy = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        ext_rpy_v.data(), 3, 3);
    ext_trans =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            ext_trans_v.data(), 3, 1);
    ext_q = Eigen::Quaterniond(ext_rpy).inverse();

    nh.param<float>("lio_sam/edge_threshold", edge_threshold, 0.1);
    nh.param<float>("lio_sam/surf_threshold", surf_threshold, 0.1);
    nh.param<int>("lio_sam/edge_feature_min_valid_num",
                  edge_feature_min_valid_num, 10);
    nh.param<int>("lio_sam/surf_feature_min_valid_num",
                  surf_feature_min_valid_num, 100);

    nh.param<float>("lio_sam/odom_surf_leaf_size", odom_surf_leaf_size, 0.2);
    nh.param<float>("lio_sam/mapping_corner_leaf_size",
                    mapping_corner_leaf_size, 0.2);
    nh.param<float>("lio_sam/mapping_surf_leaf_size", mapping_surf_leaf_size,
                    0.2);

    nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance,
                    FLT_MAX);

    nh.param<int>("lio_sam/core_num", core_num, 4);
    nh.param<double>("lio_sam/mapping_process_interval",
                     mapping_process_interval, 0.15);

    nh.param<float>("lio_sam/surrounding_keyframe_adding_dist_threshold",
                    surrounding_keyframe_adding_dist_threshold, 1.0);
    nh.param<float>("lio_sam/surrounding_keyframe_adding_angle_threshold",
                    surrounding_keyframe_adding_angle_threshold, 0.2);
    nh.param<float>("lio_sam/surrounding_keyframe_density",
                    surrounding_keyframe_density, 1.0);
    nh.param<float>("lio_sam/surrounding_keyframe_search_radius",
                    surrounding_keyframe_search_radius, 50.0);

    nh.param<bool>("lio_sam/loop_closure_enable_flag", loop_closure_enable_flag,
                   false);
    nh.param<float>("lio_sam/loop_closure_frequency", loop_closure_frequency,
                    1.0);
    nh.param<int>("lio_sam/surrounding_keyframe_size",
                  surrounding_keyframe_size, 50);
    nh.param<float>("lio_sam/history_keyframe_search_radius",
                    history_keyframe_search_radius, 10.0);
    nh.param<float>("lio_sam/history_keyframe_search_time_diff",
                    history_keyframe_search_time_diff, 30.0);
    nh.param<int>("lio_sam/history_keyframe_search_num",
                  history_keyframe_search_num, 25);
    nh.param<float>("lio_sam/history_keyframe_fitness_score",
                    history_keyframe_fitness_score, 0.3);

    nh.param<float>("lio_sam/global_map_visualization_search_radius",
                    global_map_visualization_search_radius, 1e3);
    nh.param<float>("lio_sam/global_map_visualization_pose_density",
                    global_map_visualization_pose_density, 10.0);
    nh.param<float>("lio_sam/global_map_visualization_leaf_size",
                    global_map_visualization_leaf_size, 1.0);

    usleep(100);
  }

  sensor_msgs::Imu ImuConverter(sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out;

    // rotate acc
    Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                        imu_in.linear_acceleration.y,
                        imu_in.linear_acceleration.z);
    acc = ext_rot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();

    // rotate gyr
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                        imu_in.angular_velocity.z);
    acc = ext_rot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();

    // rotate rpy
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
                              imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * ext_q;
    imu_out.orientation.w = q_final.w();
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();

    if (std::sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() +
                  q_final.z() * q_final.z() + q_final.w() * q_final.w()) <
        0.1) {
      ROS_ERROR("===  请使用9轴IMU ===");
      ros::shutdown();
    }

    return imu_out;
  }
};

/**
 * 发布 pcl格式的点云
 * 输入:ros_pub, cloudPoints, stamp, frame_id
 */
template <typename T>
sensor_msgs::PointCloud2 PublishCloud(const ros::Publisher &pub, const T &cloud,
                                      ros::Time stamp, std::string frame) {
  sensor_msgs::PointCloud2 tmp_pc2;
  pcl::toROSMsg(cloud, tmp_pc2);
  tmp_pc2.header.stamp = stamp;
  tmp_pc2.header.frame_id = frame;
  if (pub.getNumSubscribers() != 0) {
    pub.publish(tmp_pc2);
    return tmp_pc2;
  }
}

/**
 * 获得rosmsg的时间
 * 输入：rosmsg
 * 返回：秒(s)
 */
template <typename T> double RosTime(T msg) {
  return msg->header.stamp.toSec();
}

/**
 * 将imu的陀螺仪（角速度计）测量值，转换为ros数据格式
 * 输入：imu_msgs, [返回] &ang_x, &ang_y, &ang_z
 */
template <typename T>
void ImuAng2RosAng(sensor_msgs::Imu &imu, T *ang_x, T *ang_y, T *ang_z) {
  *ang_x = imu.angular_velocity.x;
  *ang_y = imu.angular_velocity.y;
  *ang_z = imu.angular_velocity.z;
}

/**
 * 将imu的加速度计测量值，转换为ros数据格式
 * 输入：imu_msgs, [返回] &acc_x, &acc_y, &acc_z
 */
template <typename T>
void ImuAcc2RosAcc(sensor_msgs::Imu &imu, T *acc_x, T *acc_y, T *acc_z) {
  *acc_x = imu.linear_acceleration.x;
  *acc_y = imu.linear_acceleration.y;
  *acc_z = imu.linear_acceleration.z;
}

/**
 * 将imu的Orientation测量值，转换为rpy
 * 输入：imu_msgs, [返回] &r, &p, &y
 */
template <typename T>
void ImuOrientation2RosPPY(sensor_msgs::Imu &imu, T *r, T *p, T *y) {
  tf2::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z,
                    imu.orientation.w);
  tf2::Matrix3x3(q).getRPY(*r, *p, *y);
}

/**
 * 点云到雷达的距离，range值
 * 输入： 单独点云点
 * 返回：距离
 */
float PointDistance(PointType p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * 2个点云点之间的欧氏距离
 * 输入： 2个点云点
 * 返回：之间的欧氏距离
 */

float PointDistance(PointType &p1, PointType &p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y) +
                   (p1.z - p2.z) * (p1.z - p2.z));
}
#endif