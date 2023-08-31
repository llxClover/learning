/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/xx/xx/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     feature_extraction.cpp
 * @brief          :     一步一步实现，并验证LIO-SAM的featureExtraction.cpp
 * ===================================================================*/
#include "lio_sam/cloud_info.h"
#include "utility.h"

#include <glog/logging.h>

struct Smoothness {
  size_t index;
  float value;
};

struct by_value {
  bool operator()(const Smoothness const &left, const Smoothness const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction : public ParamServer {
private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_corner_points_;
  ros::Publisher pub_surface_points_;
  ros::Publisher pub_point_cloud_info_;

  lio_sam::cloud_info cloud_info_;
  std_msgs::Header cloud_header_;

  double *cloud_curvature_;
  int *cloud_neighbor_picked_;
  int *cloud_label_;
  std::vector<Smoothness> cloud_smoothness_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_point_cloud_;

  // pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_point_cloud_;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr corner_point_cloud_;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr surface_point_cloud_;

  // pcl::VoxelGrid<pcl::PointXYZI> downsampling_filter_;

  // lio_sam::cloud_info cloud_info_;
  // std_msgs::Header cloud_header_;

  // std::vector<smoothness> cloud_smoothness;
  // float *cloud_curvature_;
  // int *cloud_neighbor_picked;
  // int *cloud_label;

private:
  void InitializationValue();

  void PointCloudCallback(const lio_sam::cloud_infoConstPtr &msg_in);

  void CalculateSmoothness();

  void MarkOccludePoints();

  void ExtractFeatures();

  void FreeCloudInfoMemory();

  void PublishFeatureClould();

public:
  FeatureExtraction(/* args */);
  ~FeatureExtraction();
};

FeatureExtraction::FeatureExtraction() {
  sub_point_cloud_ = nh.subscribe<lio_sam::cloud_info>(
      "lio_sam/deskew/cloud_info", 1, &FeatureExtraction::PointCloudCallback,
      this);
  pub_corner_points_ =
      nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_corner", 1);
  pub_surface_points_ =
      nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_surface", 1);
}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::InitializationValue() {
  extracted_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  corner_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  surface_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  cloud_smoothness_.resize(N_SCAN * HORIZON_RESOLUTION);

  cloud_curvature_ = new double[N_SCAN * HORIZON_RESOLUTION];
  cloud_label_ = new int[N_SCAN * HORIZON_RESOLUTION];
  cloud_neighbor_picked_ = new int[N_SCAN * HORIZON_RESOLUTION];
  ;
}

void FeatureExtraction::PointCloudCallback(
    const lio_sam::cloud_infoConstPtr &msg_in) {
  cloud_info_ = *msg_in;
  cloud_header_ = msg_in->header;

  pcl::moveFromROSMsg(cloud_info_.cloud_deskewed, *extracted_point_cloud_);

  CalculateSmoothness();

  MarkOccludePoints();
}

/**
 * 计算激光帧点云中每个点的曲率
 */
void FeatureExtraction::CalculateSmoothness() {
  int size = extracted_point_cloud_->points.size();
  // ?: 不区分ring属性吗? 上下2个环交界处咋处理？
  for (int i = 5; i < size - 5; i++) {
    double diff_range =
        cloud_info_.point_range[i - 5] + cloud_info_.point_range[i - 4] +
        cloud_info_.point_range[i - 3] + cloud_info_.point_range[i - 2] +
        cloud_info_.point_range[i - 1] + cloud_info_.point_range[i + 1] +
        cloud_info_.point_range[i + 2] + cloud_info_.point_range[i + 3] +
        cloud_info_.point_range[i + 4] + cloud_info_.point_range[i + 5] -
        10 * cloud_info_.point_range[i];

    cloud_curvature_[i] = diff_range * diff_range; // 近似表达，加速计算
    cloud_neighbor_picked_[i] = 0;
    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].index = i;
    cloud_label_[i] = 0;
  }
}

void FeatureExtraction::MarkOccludePoints() {
  int size = extracted_point_cloud_->points.size();

  for (int i = 5; i < size - 6; i++) {
    double depth_1 = cloud_info_.point_range[i];
    double depth_2 = cloud_info_.point_range[i + 1];
    int diff_column =
        std::abs(static_cast<int>(cloud_info_.point_column_index[i] -
                                  cloud_info_.point_column_index[i + 1]));
    // ?:为什么是这个顺序？
    // note: case1 遮挡（断层）
    if (diff_column < 10) {
      if (depth_1 - depth_2 > 0.3) {
        cloud_neighbor_picked_[i] = 1;
        cloud_neighbor_picked_[i - 1] = 1;
        cloud_neighbor_picked_[i - 2] = 1;
        cloud_neighbor_picked_[i - 3] = 1;
        cloud_neighbor_picked_[i - 4] = 1;
        cloud_neighbor_picked_[i - 5] = 1;
      } else if (depth_2 - depth_1 > 0.3) {
        cloud_neighbor_picked_[i + 1] = 1;
        cloud_neighbor_picked_[i + 2] = 1;
        cloud_neighbor_picked_[i + 3] = 1;
        cloud_neighbor_picked_[i + 4] = 1;
        cloud_neighbor_picked_[i + 5] = 1;
        cloud_neighbor_picked_[i + 6] = 1;
      }
    }

    double diff_range_1 = std::abs(static_cast<double>(
        cloud_info_.point_range[i] - cloud_info_.point_range[i - 1]));
    double diff_range_2 = std::abs(static_cast<double>(
        cloud_info_.point_range[i] - cloud_info_.point_range[i + 1]));
    // ?: 为什么这样处理？
    // ?: 平行？
    if (diff_range_1 > 0.02 * cloud_info_.point_range[i] &&
        diff_range_2 > 0.02 * cloud_info_.point_range[i]) {
      cloud_neighbor_picked_[i] = 1;
    }
  }
}

void FeatureExtraction::ExtractFeatures() {}

void FeatureExtraction::FreeCloudInfoMemory() {}

void FeatureExtraction::PublishFeatureClould() {}

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = false; //设置日志消息是否转到标准输出而不是日志文件
  FLAGS_alsologtostderr = true; //设置日志消息除了日志文件之外是否去标准输出
  FLAGS_log_prefix = true; //设置日志前缀是否应该添加到每行输出
  FLAGS_log_dir = "../log"; //预创建好

  /* code */

  google::ShutdownGoogleLogging();
  return 0;
}
