/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/xx/xx/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     map_optimazation.cpp
 * @brief          :     一步一步实现LIO-SAM的mapOptimazation.cpp
 * ===================================================================*/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"
#include "utility.h"

/**
 * 定义6D pose的点云格式[x,y,z,r,p,y,i,r,t]
 */
struct PointXYZRPYIRT {
  PCL_ADD_POINT4D;  // 宏定义（pcl库的）里面分别有 x、y、z 还有一个对齐变量
  PCL_ADD_INTENSITY;
  int ring;
  double timestamp;  // 记录相对于当前帧第一个激光点的时差，第一个点time=0
  float roll;
  float pitch;
  float yaw;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保new操作符内存对齐操作
} EIGEN_ALIGN16;  // 内存16字节对齐，EIGEN SSE优化要求

// 注册点类型宏  固定步骤 :先是上面定义的结构体的名称,后面是各变量
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRPYIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, roll, roll)(
        float, pitch, pitch)(float, yaw, yaw)(float, intensity, intensity)(
        int, ring, ring)(double, timestamp, timestamp))

class MapOptmization : public ParamServer {
 private:
  ros::Subscriber sub_point_cloud_;

  ros::Time pointcloud_time_stamp_;
  double pointcloud_time_cur_;
  lio_sam::cloud_info::Ptr pointcloud_in_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_corner_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surface_;

  std::mutex mutex_;

 private:
  void PointcloudCallback(const lio_sam::cloud_infoConstPtr &msg_in);

  void UpdateInitialGuess();

  void ExtractSurroundingKeyFrames();

  void DownsampleCurrentScan();

  void Scan2MapOptimization();

  void SaveKeyFrameAndFactor();

  void CorrectPoses();

  void PublishOdometry();

  void PublishFrames();

 public:
  MapOptmization();
  ~MapOptmization();
};

MapOptmization::MapOptmization() {
  sub_point_cloud_ = nh.subscribe<lio_sam::cloud_info>(
      "lio_sam/feature/cloud_info", 1, &MapOptmization::PointcloudCallback,
      this, ros::TransportHints().tcpNoDelay());
  pointcloud_in_.reset(new lio_sam::cloud_info());
  pointcloud_corner_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pointcloud_surface_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

MapOptmization::~MapOptmization() {}

void MapOptmization::PointcloudCallback(
    const lio_sam::cloud_infoConstPtr &msg_in) {
  pointcloud_time_stamp_ = msg_in->header.stamp;
  pointcloud_time_cur_ = pointcloud_time_stamp_.toSec();

  pointcloud_in_ = msg_in;
  // note: 这里不可以使用pcl::moveFromROSMsg
  // note: move的都不是 constPtr
  // void fromROSMsg(const sensor_msgs::PointCloud2 &, pcl : PointCloud<T> &);
  // void moveFromROSMsg(sensor_msgs::PointCloud2 &, pcl : PointCloud<T> &);
  pcl::fromROSMsg(msg_in->cloud_corner, *pointcloud_corner_);
  pcl::fromROSMsg(msg_in->cloud_surface, *pointcloud_surface_);

  std::lock_guard<std::mutex> lock(mutex_);

  static double time_last_processing = -1;
  if (pointcloud_time_cur_ - time_last_processing >= mapping_process_interval) {
    time_last_processing = pointcloud_time_cur_;

    UpdateInitialGuess();

    ExtractSurroundingKeyFrames();

    DownsampleCurrentScan();

    Scan2MapOptimization();

    SaveKeyFrameAndFactor();

    CorrectPoses();

    PublishOdometry();

    PublishFrames();
  }
}

void MapOptmization::UpdateInitialGuess(){}

void MapOptmization::ExtractSurroundingKeyFrames(){}

void MapOptmization::DownsampleCurrentScan(){}

void MapOptmization::Scan2MapOptimization(){}

void MapOptmization::SaveKeyFrameAndFactor(){}

void MapOptmization::CorrectPoses(){}

void MapOptmization::PublishOdometry(){}

void MapOptmization::PublishFrames(){}