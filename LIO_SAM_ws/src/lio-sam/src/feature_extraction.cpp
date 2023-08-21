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

struct smoothness {
  size_t index;
  float value;
};

struct by_value {
  bool operator()(const smoothness const &left, const smoothness const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction : public ParamServer {
private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_corner_points_;
  ros::Publisher pub_surface_points_;

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

void FeatureExtraction::InitializationValue() {}

void FeatureExtraction::PointCloudCallback(
    const lio_sam::cloud_infoConstPtr &msg_in) {
      lio_sam::cloud_info tmp_cloud_info;
      tmp_cloud_info = *msg_in;



    }

void FeatureExtraction::CalculateSmoothness() {}

void FeatureExtraction::MarkOccludePoints() {}

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
