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

  float transform_to_be_mapped_[6];

  Eigen::Affine3f incremental_odom_affine_front_;
  Eigen::Affine3f incremental_odom_affine_back_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_key_pose3D_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloud_key_pose3D_;

  pcl::PointCloud<PointXYZRPYIRT>::Ptr cloud_key_pose6D_;
  pcl::PointCloud<PointXYZRPYIRT>::Ptr copy_cloud_key_pose6D_;

  std::mutex mutex_;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surround_key_poses_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_history_key_poses_;

  pcl::VoxelGrid<pcl::PointXYZI> downsample_filter_surround_key_poses_;

  double time_laser_info_current_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_from_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_from_map_DS_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surface_from_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surface_from_map_DS_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surface_last_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last_DS_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surface_last_DS_;
  int laser_cloud_corner_last_DS_num_ = 0;
  int laser_cloud_surface_last_DS_num_ = 0;

  std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZI>,
                          pcl::PointCloud<pcl::PointXYZI>>>
      laser_cloud_map_container_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> corner_cloud_key_frames_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> surface_cloud_key_frames_;

  pcl::VoxelGrid<pcl::PointXYZI> downsample_filter_corner_;
  pcl::VoxelGrid<pcl::PointXYZI> downsample_filter_surface_;

  int laser_cloud_corner_from_map_DS_num_;
  int laser_cloud_surface_from_map_DS_num_;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surface_from_map_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_ori_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr coeff_sel_;

  Eigen::Affine3f trans_point_accociate_to_map_;

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

  void ExtractNearby();

  void ExtractCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);

  pcl::PointCloud<pcl::PointXYZI>::Ptr TransfromPointCloud(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
      PointXYZRPYIRT &transfrom_in);

  // 角特征点优化
  void CornerOptimization();

  // 平面特征点优化
  void SurfaceOptimization();

  void CombineOptimizationCoeffs();

  bool LMOptimization(int num);

  void TransfromUpdate();

  void UpdatePointAssociateToMap();

 public:
  MapOptmization();
  ~MapOptmization();
};

MapOptmization::MapOptmization() {
  sub_point_cloud_ = nh.subscribe<lio_sam::cloud_info>(
      "lio_sam/feature/cloud_info", 1, &MapOptmization::PointcloudCallback,
      this, ros::TransportHints().tcpNoDelay());
  pointcloud_in_.reset(new lio_sam::cloud_info());
  // note: 数组初始化
  // 方法1：transform_to_be_mapped_[6]={0};
  // 方法2：
  std::memset(transform_to_be_mapped_, 0, sizeof(transform_to_be_mapped_) * 6);

  kdtree_surround_key_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_history_key_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  laser_cloud_corner_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_corner_from_map_DS_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surface_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surface_from_map_DS_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  downsample_filter_surround_key_poses_.setLeafSize(
      surrounding_keyframe_density, surrounding_keyframe_density,
      surrounding_keyframe_density);
  downsample_filter_corner_.setLeafSize(mapping_corner_leaf_size,
                                        mapping_corner_leaf_size,
                                        mapping_corner_leaf_size);
  downsample_filter_surface_.setLeafSize(
      mapping_surf_leaf_size, mapping_surf_leaf_size, mapping_surf_leaf_size);

  laser_cloud_corner_last_DS_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surface_last_DS_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_corner_last_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surface_last_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  kdtree_corner_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  kdtree_surface_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
  pcl::fromROSMsg(msg_in->cloud_corner, *laser_cloud_corner_last_);
  pcl::fromROSMsg(msg_in->cloud_surface, *laser_cloud_surface_last_);

  std::lock_guard<std::mutex> lock(mutex_);

  // ?:难道这就是传说中的 mapping执行频率控制？
  // 当两帧之间的delta_t > mapping周期时,说明该进行建图了
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

/**
 * 当前帧pose初始化
 * 1. 如果是第一帧，用原始imu数据初始化(0,0,0,r,p,y)
 * 2.后续帧，用imu预积分的值计算两帧之间的T，然后通过前一帧的transTobeMapped(优化量)，
 *   计算当前的lidarPose
 */
void MapOptmization::UpdateInitialGuess() {
  // 前一帧的位姿。主要指lidar
  incremental_odom_affine_front_ = [](float *transform_in) -> Eigen::Affine3f {
    return pcl::getTransformation(transform_in[3], transform_in[4],
                                  transform_in[5], transform_in[0],
                                  transform_in[1], transform_in[2]);
  }(transform_to_be_mapped_);
  // note: static 的局部变量，在函数结束后不会释放，程序结束后才释放
  static Eigen::Affine3f last_imu_trans;

  // 如果关键帧为空，则继续进行初始化
  if (cloud_key_pose3D_->points.empty()) {
    // 旋转部分用imu数据初始化
    transform_to_be_mapped_[0] = pointcloud_in_->imu_roll_init;
    transform_to_be_mapped_[1] = pointcloud_in_->imu_pitch_init;
    transform_to_be_mapped_[2] = pointcloud_in_->imu_yaw_init;

    if (!use_imu_heading_init) {
      transform_to_be_mapped_[2] = 0;
    }

    last_imu_trans = pcl::getTransformation(0, 0, 0, transform_to_be_mapped_[0],
                                            transform_to_be_mapped_[1],
                                            transform_to_be_mapped_[2]);
    return;
  }

  static bool last_imu_pre_trans_available = false;
  static Eigen::Affine3f last_imu_pre_trans;

  if (pointcloud_in_->odom_available == true) {
    // 当前帧初始pose用imu预积分的odom初始化
    Eigen::Affine3f trans_back = pcl::getTransformation(
        pointcloud_in_->init_guess_x, pointcloud_in_->init_guess_y,
        pointcloud_in_->init_guess_z, pointcloud_in_->init_guess_roll,
        pointcloud_in_->init_guess_pitch, pointcloud_in_->init_guess_yaw);
    // 给前一帧赋值
    if (last_imu_pre_trans_available == false) {
      last_imu_pre_trans_available = true;
      last_imu_pre_trans = trans_back;
    } else {
      Eigen::Affine3f trans_incre = last_imu_pre_trans.inverse() * trans_back;

      Eigen::Affine3f trans_tobe = [](float *transform_in) -> Eigen::Affine3f {
        return pcl::getTransformation(transform_in[3], transform_in[4],
                                      transform_in[5], transform_in[0],
                                      transform_in[1], transform_in[2]);
      }(transform_to_be_mapped_);

      Eigen::Affine3f trans_final = trans_tobe * trans_incre;
      // 更新当前帧的pose
      pcl::getTranslationAndEulerAngles(
          trans_final, transform_to_be_mapped_[3], transform_to_be_mapped_[4],
          transform_to_be_mapped_[5], transform_to_be_mapped_[0],
          transform_to_be_mapped_[1], transform_to_be_mapped_[2]);
      // 赋值给前一帧
      last_imu_pre_trans = trans_back;
      last_imu_trans = pcl::getTransformation(
          0, 0, 0, pointcloud_in_->imu_roll_init,
          pointcloud_in_->imu_pitch_init, pointcloud_in_->imu_yaw_init);

      return;
    }
  }

  if (pointcloud_in_->imu_available == true) {
    Eigen::Affine3f trans_back = pcl::getTransformation(
        0, 0, 0, pointcloud_in_->imu_roll_init, pointcloud_in_->imu_pitch_init,
        pointcloud_in_->imu_yaw_init);
    Eigen::Affine3f trans_incre = last_imu_pre_trans.inverse() * trans_back;
    Eigen::Affine3f trans_tobe = [](float *transform_in) -> Eigen::Affine3f {
      return pcl::getTransformation(transform_in[3], transform_in[4],
                                    transform_in[5], transform_in[0],
                                    transform_in[1], transform_in[2]);
    }(transform_to_be_mapped_);

    Eigen::Affine3f trans_final = trans_tobe * trans_incre;
    pcl::getTranslationAndEulerAngles(
        trans_final, transform_to_be_mapped_[3], transform_to_be_mapped_[4],
        transform_to_be_mapped_[5], transform_to_be_mapped_[0],
        transform_to_be_mapped_[1], transform_to_be_mapped_[2]);

    last_imu_trans = pcl::getTransformation(
        0, 0, 0, pointcloud_in_->imu_roll_init, pointcloud_in_->imu_pitch_init,
        pointcloud_in_->imu_yaw_init);

    return;
  }
}

void MapOptmization::ExtractSurroundingKeyFrames() {
  if (!cloud_key_pose3D_->points.empty()) {
    ExtractNearby();
  } else {
    return;
  }
}

void MapOptmization::ExtractNearby() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr surrounding_key_poses(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surrounding_key_poses_DS(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<int> point_search_index;
  std::vector<float> point_search_distance;
  // kdtree输入的是全局关键帧pose
  kdtree_surround_key_poses_->setInputCloud(cloud_key_pose3D_);
  // 在最新关键帧附近搜索
  kdtree_surround_key_poses_->radiusSearch(
      cloud_key_pose3D_->back(),
      static_cast<double>(surrounding_keyframe_search_radius),
      point_search_index, point_search_distance);
  // 将找到的index的点云提出来
  for (auto point_index : point_search_index) {
    surrounding_key_poses->emplace_back(
        cloud_key_pose3D_->points[static_cast<int>(point_index)]);
  }
  // 降采样
  downsample_filter_surround_key_poses_.setInputCloud(surrounding_key_poses);
  downsample_filter_surround_key_poses_.filter(*surrounding_key_poses_DS);

  // 加入时间上相近的关键帧，比如机器人原地不动，打转等情况
  // note: 关键帧集合应该是 时空上相近的keyFrame
  int num = cloud_key_pose3D_->points.size();

  for (int i = num - 1; i >= 0; i--) {
    if (time_laser_info_current_ - cloud_key_pose6D_->points[i].timestamp <
        10.0) {
      surrounding_key_poses_DS->emplace_back(cloud_key_pose3D_->points[i]);
    } else {
      break;
    }
  }

  ExtractCloud(surrounding_key_poses_DS);
}

/**
 * 将关键帧对应的角点和平面点，加入局部地图
 */
void MapOptmization::ExtractCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in) {
  laser_cloud_corner_from_map_->clear();
  laser_cloud_surface_from_map_->clear();

  for (auto cloud : cloud_in->points) {
    if (PointDistance(cloud, cloud_key_pose3D_->back()) >
        surrounding_keyframe_search_radius) {
      continue;
    }

    int this_key_index = static_cast<int>(cloud.intensity);
    // 从局部地图中搜索
    // 如果找到了，就加入
    if (laser_cloud_map_container_.find(this_key_index) !=
        laser_cloud_map_container_.end()) {
      // note: 点云拼接，直接加
      // Objects for storing the point clouds.
      // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA(new
      // pcl::PointCloud<pcl::PointXYZRGBA>);
      // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB(new
      // pcl::PointCloud<pcl::PointXYZRGBA>);
      // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudC(new
      // pcl::PointCloud<pcl::PointXYZRGBA>);

      //  Create cloud "C", with the points of both "A" and
      //  "B".点云A、B合并生成C
      // *cloudC = (*cloudA) + (*cloudB);

      *laser_cloud_corner_from_map_ +=
          laser_cloud_map_container_[this_key_index].first;
      *laser_cloud_surface_from_map_ +=
          laser_cloud_map_container_[this_key_index].second;
    }
    // 如果没找到， 就把这个点加入局部地图
    else {
      pcl::PointCloud<pcl::PointXYZI> laser_cloud_corner_tmp =
          *TransfromPointCloud(corner_cloud_key_frames_[this_key_index],
                               cloud_key_pose6D_->points[this_key_index]);
      pcl::PointCloud<pcl::PointXYZI> laser_cloud_surface_tmp =
          *TransfromPointCloud(surface_cloud_key_frames_[this_key_index],
                               cloud_key_pose6D_->points[this_key_index]);

      *laser_cloud_corner_from_map_ += laser_cloud_corner_tmp;
      *laser_cloud_surface_from_map_ += laser_cloud_surface_tmp;
      laser_cloud_map_container_[this_key_index] =
          std::make_pair(laser_cloud_corner_tmp, laser_cloud_surface_tmp);
    }
  }

  downsample_filter_corner_.setInputCloud(laser_cloud_corner_from_map_);
  downsample_filter_corner_.filter(*laser_cloud_corner_from_map_DS_);
  laser_cloud_corner_from_map_DS_num_ = laser_cloud_corner_from_map_DS_->size();

  downsample_filter_surface_.setInputCloud(laser_cloud_surface_from_map_);
  downsample_filter_surface_.filter(*laser_cloud_surface_from_map_DS_);
  laser_cloud_surface_from_map_DS_num_ =
      laser_cloud_surface_from_map_DS_->size();

  if (laser_cloud_map_container_.size() > 1000) {
    laser_cloud_map_container_.clear();
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MapOptmization::TransfromPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    PointXYZRPYIRT &transfrom_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>());

  int cloud_size = cloud_in->points.size();
  cloud_out->resize(cloud_size);

  Eigen::Affine3f trans_current = pcl::getTransformation(
      transfrom_in.x, transfrom_in.y, transfrom_in.z, transfrom_in.roll,
      transfrom_in.pitch, transfrom_in.yaw);

  pcl::PointXYZI cloud_out_tmp;
  for (auto cloud : cloud_in->points) {
    cloud_out_tmp.x = trans_current(0, 0) * cloud.x +
                      trans_current(0, 1) * cloud.y +
                      trans_current(0, 2) * cloud.z;
    cloud_out_tmp.y = trans_current(1, 0) * cloud.x +
                      trans_current(1, 1) * cloud.y +
                      trans_current(1, 2) * cloud.z;
    cloud_out_tmp.z = trans_current(2, 0) * cloud.x +
                      trans_current(2, 1) * cloud.y +
                      trans_current(2, 2) * cloud.z;
    cloud_out_tmp.intensity = cloud.intensity;

    cloud_out->emplace_back(cloud_out_tmp);
  }

  return cloud_out;
}

/**
 * 对当前帧点云的角点和面点进行下采样
 */
void MapOptmization::DownsampleCurrentScan() {
  laser_cloud_corner_last_DS_->clear();
  downsample_filter_corner_.setInputCloud(laser_cloud_corner_last_);
  downsample_filter_corner_.filter(*laser_cloud_corner_last_DS_);
  laser_cloud_corner_last_DS_num_ = laser_cloud_corner_last_DS_->size();

  laser_cloud_surface_last_DS_->clear();
  downsample_filter_surface_.setInputCloud(laser_cloud_surface_last_);
  downsample_filter_surface_.filter(*laser_cloud_surface_last_DS_);
  laser_cloud_surface_last_DS_num_ = laser_cloud_surface_last_DS_->size();
}

void MapOptmization::Scan2MapOptimization() {
  if (cloud_key_pose3D_->points.empty()) {
    return;
  }
  // 当前帧的点云特征要达到一定的量
  if (laser_cloud_corner_last_DS_num_ >= edge_feature_min_valid_num &&
      laser_cloud_surface_last_DS_num_ >= surf_feature_min_valid_num) {
    // local map加入kdtree
    kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_from_map_DS_);
    kdtree_surface_from_map_->setInputCloud(laser_cloud_surface_from_map_DS_);

    for (int iter = 0; iter < 30; iter++) {
      laser_cloud_ori_->clear();
      coeff_sel_->clear();

      // 角特征点优化
      CornerOptimization();

      // 平面特征点优化
      SurfaceOptimization();

      CombineOptimizationCoeffs();

      if (LMOptimization(iter)) {
        break;
      }
    }

    TransfromUpdate();

  } else {
    ROS_WARN(" ==> 特征点不足:仅仅有%d个角特征点, %d个面特征点",
             laser_cloud_corner_last_DS_num_, laser_cloud_surface_last_DS_num_);
  }
}

void MapOptmization::SaveKeyFrameAndFactor() {}

void MapOptmization::CorrectPoses() {}

void MapOptmization::PublishOdometry() {}

void MapOptmization::PublishFrames() {}

// 角特征点优化
void MapOptmization::CornerOptimization() { UpdatePointAssociateToMap(); }

// 平面特征点优化
void MapOptmization::SurfaceOptimization() {}

void MapOptmization::CombineOptimizationCoeffs() {}

bool MapOptmization::LMOptimization(int num) {}

void MapOptmization::TransfromUpdate() {}

void MapOptmization::UpdatePointAssociateToMap() {
  // ?: 这个激光帧的位姿变换咋回事？
  trans_point_accociate_to_map_ = [](float *transform_in) -> Eigen::Affine3f {
    return pcl::getTransformation(transform_in[3], transform_in[4],
                                  transform_in[5], transform_in[0],
                                  transform_in[1], transform_in[2]);
  }(transform_to_be_mapped_);

// note: 多线程编程 openmp
// note: openmp是由一系列#paragma指令组成，这些指令控制如何多线程的执行程序.
// note: 所有的omp指令都是以"#pragma omp“开头，换行符结束
// # pragma omp paraller for num_threads(XXX) 多线程以指定核数运行for循环
#pragma omp paraller for num_threads(core_num)
  for (int i = 0; i < laser_cloud_corner_last_DS_num_; i++) {
    ;
  }
}