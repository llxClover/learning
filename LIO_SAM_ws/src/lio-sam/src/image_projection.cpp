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
  ros::Subscriber sub_imu_;

private:
  std::deque<sensor_msgs::PointCloud2> point_cloud_queue_;
  std::deque<sensor_msgs::Imu> imu_queue_;

  sensor_msgs::PointCloud2 front_point_cloud_;
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr pcl_front_point_cloud_in_;

  std_msgs::Header front_point_cloud_header_;
  double front_point_cloud_start_time_;
  double front_point_cloud_end_time_;

  bool ring_flag_;
  bool timestamp_flag_;

  std::mutex imu_lock_;
  std::mutex point_cloud_lock_;

private:
  void
  PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool CachePointCloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool Deskew();

  void ImuCallback(const sensor_msgs::ImuConstPtr &imu_in);

public:
  ImageProjection(/* args */);
  ~ImageProjection();
};

ImageProjection::ImageProjection(/* args */) {
  sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
      point_cloud_topic, 10, &ImageProjection::PointCloudCallback, this);
  ring_flag_ = false;
  timestamp_flag_ = false;

  sub_imu_ = nh.subscribe<sensor_msgs::Imu>(
      imu_topic, 100, &ImageProjection::ImuCallback, this);
}

/**
 * 将收到的ros格式的imu存入imu数据队列(期间要进行坐标变换)
*/
void ImageProjection::ImuCallback(const sensor_msgs::ImuConstPtr &imu_in) {
  // note: 【重要】将imu数据转换到lidar坐标系
  sensor_msgs::Imu this_imu = ImuConverter(*imu_in);
  // note: 队列添加数据时，该队列线程上锁，数据不可用
  std::lock_guard<std::mutex> lock_1(imu_lock_);
  imu_queue_.emplace_back(this_imu);
}

/**
 * 将收到的ros格式的激光点云存入点云数据队列，并取出队列中最早的一帧数据进行
 * 点云距离有效性检验，ring、timestamp通道检验
 */
bool ImageProjection::CachePointCloud(
    const sensor_msgs::PointCloud2ConstPtr &ros_point_cloud_in) {
  // 将收到的一帧激光点云存起来
  // ?: 这个锁是我自己加的，有问题的话，删除，目前只是看看为什么imu/odom加锁，而lidar不？
  std::lock_guard<std::mutex> lock_point_cloud_(point_cloud_lock_);
  point_cloud_queue_.emplace_back(*ros_point_cloud_in);
  // 点云数据太少，不行啊！
  if (point_cloud_queue_.size() <= 2) {
    return false;
  }
  // 取最早的一帧激光数据
  front_point_cloud_ = std::move(point_cloud_queue_.front());
  point_cloud_queue_.pop_front();
  if (sensor == SensorType::VELODYNE) {
    pcl::moveFromROSMsg(front_point_cloud_, *pcl_front_point_cloud_in_);
  } else {
    ROS_ERROR_STREAM("===> 未知的激光雷达型号 <===");
    ros::shutdown();
  }

  front_point_cloud_header_ = front_point_cloud_.header;
  front_point_cloud_start_time_ = front_point_cloud_.header.stamp.toSec();
  front_point_cloud_end_time_ = front_point_cloud_start_time_ +
                                pcl_front_point_cloud_in_->end()->timestamp;
  // note: 检验是否存在无效的点云数据nan
  if (ros_point_cloud_in->is_dense == false) {
    ROS_ERROR(
        "===> point cloud is not dense format, please remove NaN points!");
  }
  // note: 检验ring是否存在
  if (!ring_flag_) {
    for (auto filed : front_point_cloud_.fields) {
      if (filed.name == "ring" || filed.name == "r") {
        ring_flag_ = true;
        break;
      }
    }

    if (ring_flag_ == false) {
      ROS_ERROR("===> point cloud 'ring' channel is unavailable !");
      ros::shutdown();
    }
  }
  // note: 检验timestamp是否存在
  if (!timestamp_flag_) {
    for (auto filed : front_point_cloud_.fields) {
      if (filed.name == "time" || filed.name == "t" ||
          filed.name == "timestamp") {
        timestamp_flag_ = true;
        break;
      }
    }

    if (timestamp_flag_ == false) {
      ROS_WARN("===> point cloud 'timestamp' is unavailable!");
    }
  }

  return true;
}

/**
 * 使用IMU数据进行点云的运动畸变去除
 */
bool ImageProjection::Deskew() {}

void ImageProjection::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &pointcloud_in) {

  if (!ImageProjection::CachePointCloud(pointcloud_in)) {
    ROS_WARN("===> 点云格式存在问题，无法加入队列 <===");
    return;
  }
  if (!ImageProjection::Deskew()) {
    ROS_WARN("===> 原始点云未进行运动畸变去除 <===");
    return;
  }
}

ImageProjection::~ImageProjection() {}
