/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/21/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     image_projection.cpp
 * @brief          :     一步一步实现，并验证LIO-SAM的imageProjection.cpp
 * ===================================================================*/

#include "lio_sam/cloud_info.h"
#include "utility.h"

/**
 * 定义Velodyne的点云格式
 */
struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D; // 宏定义（pcl库的）里面分别有 x、y、z 还有一个对齐变量
  PCL_ADD_INTENSITY;
  int ring;
  double timestamp; // 记录相对于当前帧第一个激光点的时差，第一个点time=0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW //确保new操作符内存对齐操作
} EIGEN_ALIGN16; // 内存16字节对齐，EIGEN SSE优化要求

//注册点类型宏  固定步骤 :先是上面定义的结构体的名称,后面是各变量
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        int, ring, ring)(double, timestamp, timestamp))

class ImageProjection : public ParamServer {
private:
  ros::Subscriber sub_pointcloud_;
  // ros::Subscriber sub_imu_;

private:
  std::deque<sensor_msgs::PointCloud2> point_cloud_queue_;

  sensor_msgs::PointCloud2 front_point_cloud_;
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr pcl_point_cloud_in_;

  std_msgs::Header front_point_cloud_header_;
  double front_point_cloud_start_time_;
  double front_point_cloud_end_time_;

private:
  void
  PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool CachePointCloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool Deskew();

public:
  ImageProjection(/* args */);
  ~ImageProjection();
};

ImageProjection::ImageProjection(/* args */) {
  sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
      point_cloud_topic, 10, &ImageProjection::PointCloudCallback, this);
}

/**
 *
 */
bool ImageProjection::CachePointCloud(
    const sensor_msgs::PointCloud2ConstPtr &ros_point_cloud_in) {
  // 将收到的一帧激光点云存起来
  point_cloud_queue_.emplace_back(*ros_point_cloud_in);
  // 点云数据太少，不行啊！
  if (point_cloud_queue_.size() <= 2) {
    return false;
  }
  // 取最早的一帧激光数据
  front_point_cloud_ = std::move(point_cloud_queue_.front());
  point_cloud_queue_.pop_front();
  if (sensor == SensorType::VELODYNE) {
    pcl::moveFromROSMsg(front_point_cloud_, *pcl_point_cloud_in_);
  } else {
    ROS_ERROR_STREAM("===> 未知的激光雷达型号 <===");
    ros::shutdown();
  }

  front_point_cloud_header_ = front_point_cloud_.header;
  front_point_cloud_start_time_ = front_point_cloud_.header.stamp.toSec();
  front_point_cloud_end_time_ =
      front_point_cloud_start_time_ + pcl_point_cloud_in_->end()->timestamp;
}

bool ImageProjection::Deskew() {}

void ImageProjection::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &pointcloud_in) {

  if (!ImageProjection::CachePointCloud(pointcloud_in)) {
    ROS_WARN("===> 第1帧点云还未进行缓存 <===");
    return;
  }
  if (!ImageProjection::Deskew()) {
    ROS_WARN("===> 原始点云未进行运动畸变去除 <===");
    return;
  }
}

ImageProjection::~ImageProjection() {}
