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

constexpr int queue_length = 2000;

class ImageProjection : public ParamServer {
private:
  ros::Subscriber sub_pointcloud_;
  ros::Subscriber sub_imu_;

private:
  std::deque<sensor_msgs::PointCloud2> point_cloud_queue_;
  std::deque<sensor_msgs::Imu> imu_queue_;
  std::deque<nav_msgs::Odometry> odom_queue_;

  sensor_msgs::PointCloud2 front_point_cloud_;
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr pcl_front_point_cloud_in_;

  // 自定义点云数据类型cloud_info.msg：去过畸变的点云，点云特征，初始估计位姿，关键帧，起始帧的index等
  lio_sam::cloud_info cloud_info_;
  std_msgs::Header front_point_cloud_header_;
  double front_point_cloud_start_time_;
  double front_point_cloud_end_time_;

  bool ring_flag_;
  bool timestamp_flag_;
  bool odom_deskew_flag_;

  Eigen::Vector3f odom_incre_;
  Eigen::Vector3f rpy_incre_;

  int imu_pointer_current_;
  double *imu_t_;
  // 存放一帧imu数据点的旋转角
  Eigen::Vector3d *imu_rot_ang_;

  std::mutex imu_lock_;
  std::mutex point_cloud_lock_;
  std::mutex odom_lock_;

private:
  void
  PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool CachePointCloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in);
  bool Deskew();
  bool ImuDeskew();
  bool OdomDeskew();
  void ImuCallback(const sensor_msgs::ImuConstPtr &imu_in);

  void AllocateMemory();
  void ResetParameters();

public:
  ImageProjection();
  ~ImageProjection();
};

ImageProjection::ImageProjection() {
  sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
      point_cloud_topic, 10, &ImageProjection::PointCloudCallback, this);
  ring_flag_ = false;
  timestamp_flag_ = false;

  sub_imu_ = nh.subscribe<sensor_msgs::Imu>(
      imu_topic, 100, &ImageProjection::ImuCallback, this);
}

void ImageProjection::AllocateMemory() {
  pcl_front_point_cloud_in_.reset(new pcl::PointCloud<VelodynePointXYZIRT>());

  cloud_info_.start_ring_index.assign(N_SCAN, 0);
  cloud_info_.end_ring_index.assign(N_SCAN, 0);
  cloud_info_.point_column_index.assign(HORIZON_RESOLUTION, 0);
  cloud_info_.point_range.assign(N_SCAN * HORIZON_RESOLUTION, 0);

  imu_t_ = new double[queue_length];
  imu_rot_ang_ = new Eigen::Vector3d[queue_length];
}

void ImageProjection::ResetParameters() {
  pcl_front_point_cloud_in_->clear();
  imu_pointer_current_ = 0;
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
  // ?:
  // 这个锁是我自己加的，有问题的话，删除，目前只是看看为什么imu/odom加锁，而lidar不？
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
 * 进行点云的运动畸变去除
 * 1. imu辅助
 * 2. odom辅助
 */
bool ImageProjection::Deskew() {

  // 保证imu数据包括住整个lidar点云帧
  if (imu_queue_.empty() ||
      imu_queue_.front().header.stamp.toSec() > front_point_cloud_start_time_ ||
      imu_queue_.back().header.stamp.toSec() < front_point_cloud_end_time_) {
    ROS_DEBUG("===> imu去畸变: 等待imu数据中...... <===");
    return false;
  }

  // imu辅助去除运动畸变
  ImuDeskew();

  OdomDeskew();

  return true;
}

/**
 * 进行点云的运动畸变去除
 * 1. imu辅助
 */
bool ImageProjection::ImuDeskew() {
  std::lock_guard<std::mutex> locker_1(imu_lock_);
  cloud_info_.imu_available = false;
  while (!imu_queue_.empty()) {
    // 0.01 对应100Hz,imu一般为200-500Hz，也就是说至少2个imu点在lidar最前面
    if (imu_queue_.front().header.stamp.toSec() <
        front_point_cloud_start_time_ - 0.01) {
      imu_queue_.pop_front();
    } else
      break;
  }
  // mark: 此处更改源代码
  // if (imu_queue_.empty()) {
  //   return false;
  // }

  if (imu_queue_.size() < 2) {
    return false;
  }

  imu_pointer_current_ = 0;

  for (auto imu : imu_queue_) {
    double cur_imu_t = imu.header.stamp.toSec();
    // 提取imu的位姿
    // 将imu的四元数转化成RPY，实际上就是当前lidar帧优化时的初始pose的一部分值
    if (cur_imu_t < front_point_cloud_start_time_) {
      ImuOrientation2RosPPY(imu, &cloud_info_.imu_roll_init,
                            &cloud_info_.imu_pitch_init,
                            &cloud_info_.imu_yaw_init);
    }
    // 0.01 对应100Hz,imu一般为200-500Hz，也就是说至少2个imu点在lidar最后面
    if (cur_imu_t > front_point_cloud_end_time_ + 0.01)
      break;

    // 第一帧imu旋转角初始化
    if (imu_pointer_current_ == 0) {
      imu_t_[0] = cur_imu_t;
      imu_rot_ang_->setZero();
      imu_pointer_current_++;
      continue;
    }

    // 提取imu的角速度
    Eigen::Vector3d rot_ang;

    ImuAng2RosAng(imu, &rot_ang[0], &rot_ang[1], &rot_ang[2]);

    // 当前时刻旋转角 = 前一时刻旋转角 + 角速度 * 时差
    double time_diff = cur_imu_t - imu_t_[imu_pointer_current_ - 1];
    imu_rot_ang_[imu_pointer_current_][0] =
        imu_rot_ang_[imu_pointer_current_ - 1][0] + rot_ang[0] * time_diff;
    imu_rot_ang_[imu_pointer_current_][1] =
        imu_rot_ang_[imu_pointer_current_ - 1][1] + rot_ang[1] * time_diff;
    imu_rot_ang_[imu_pointer_current_][2] =
        imu_rot_ang_[imu_pointer_current_ - 1][2] + rot_ang[2] * time_diff;

    imu_pointer_current_++;
  }

  // mark: 此处更改源代码
  // --imu_pointer_current_; // ?:这句的作用，我不得其解

  // if (imu_pointer_current_ <= 0)
  //   return false;

  cloud_info_.imu_available = true;
}

bool ImageProjection::OdomDeskew() {
  std::lock_guard<std::mutex> lock_2(odom_lock_);
  cloud_info_.odom_available = false;

  while (!odom_queue_.empty()) {
    // 0.01 对应100Hz,imu一般为200-500Hz，也就是说至少2个imu点在lidar最前面
    if (odom_queue_.front().header.stamp.toSec() <
        front_point_cloud_start_time_ - 0.01) {
      odom_queue_.pop_front();
    } else
      break;
  }

  if (odom_queue_.size() < 2) {
    return false;
  }

  // 要求必须有当前激光帧时刻之前的imu里程计数据
  if (odom_queue_.front().header.stamp.toSec() > front_point_cloud_start_time_)
    return false;

  // 提取当前激光帧起始时刻的imu里程计
  nav_msgs::Odometry start_odom;
  for (auto odom : odom_queue_) {
    if (odom.header.stamp.toSec() > front_point_cloud_start_time_) {
      start_odom = odom;
      break;
    }
  }

  tf2::Quaternion q(
      start_odom.pose.pose.orientation.x, start_odom.pose.pose.orientation.y,
      start_odom.pose.pose.orientation.z, start_odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  // 获得odom开始时刻的 x y z r p y
  Eigen::Affine3f trans_start = pcl::getTransformation(
      start_odom.pose.pose.position.x, start_odom.pose.pose.position.y,
      start_odom.pose.pose.position.z, r, p, y);

  cloud_info_.init_guess_x = start_odom.pose.pose.position.x;
  cloud_info_.init_guess_y = start_odom.pose.pose.position.y;
  cloud_info_.init_guess_z = start_odom.pose.pose.position.z;
  cloud_info_.imu_roll_init = r;
  cloud_info_.imu_pitch_init = p;
  cloud_info_.imu_yaw_init = y;

  cloud_info_.odom_available = true;

  odom_deskew_flag_ = false;

  // 要求必须当前激光帧结束之后，必须有imu里程计数据
  if (odom_queue_.back().header.stamp.toSec() < front_point_cloud_end_time_)
    return false;
  // 提取当前激光帧结束时刻的imu里程计
  nav_msgs::Odometry end_odom;
  for (auto odom : odom_queue_) {
    if (odom.header.stamp.toSec() > front_point_cloud_end_time_) {
      end_odom = odom;
      break;
    }
  }
  // ?:为什么
  // 如果起止时刻对应imu里程计的方差不等，返回
  if (static_cast<int>(std::round(start_odom.pose.covariance[0])) !=
      static_cast<int>(std::round(end_odom.pose.covariance[0])))
    return false;

  tf2::Quaternion q(
      end_odom.pose.pose.orientation.x, end_odom.pose.pose.orientation.y,
      end_odom.pose.pose.orientation.z, end_odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  // 获得odom结束时刻的 x y z r p y
  Eigen::Affine3f trans_end = pcl::getTransformation(
      end_odom.pose.pose.position.x, end_odom.pose.pose.position.y,
      end_odom.pose.pose.position.z, r, p, y);
  // ?:思考一下这里的变换
  Eigen::Affine3f trans_start_end = trans_start.inverse() * trans_end;

  pcl::getTranslationAndEulerAngles(
      trans_start_end, odom_incre_[0], odom_incre_[1], odom_incre_[2],
      rpy_incre_[0], rpy_incre_[1], rpy_incre_[2]);

  odom_deskew_flag_ = true;
}

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
