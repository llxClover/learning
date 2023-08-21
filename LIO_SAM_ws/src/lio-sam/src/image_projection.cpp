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
    const sensor_msgs::PointCloud2ConstPtr &pointcloud_in) {
      
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
