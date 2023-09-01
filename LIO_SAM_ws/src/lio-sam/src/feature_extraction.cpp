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

  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;

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
  down_size_filter_.setLeafSize(odom_surf_leaf_size, odom_surf_leaf_size,
                                odom_surf_leaf_size);
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

/**
 * 提取 角点 和 平面点
 * 为了保证特征点分布均匀，将360°均分6份，针对每一份提取20个曲率最大的角点，其它为平面点，
 * 然后加入角点和平面点指针中。因为平面点太多了，所以体素滤波采样一下。
 */
void FeatureExtraction::ExtractFeatures() {
  corner_point_cloud_->clear();
  surface_point_cloud_->clear();

  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_scan(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_scan_downsampping(
      new pcl::PointCloud<pcl::PointXYZI>());
  // 依次遍历每条扫描线
  for (int i = 0; i < N_SCAN; i++) {
    surface_cloud_scan->clear();
    surface_cloud_scan_downsampping->clear();
    // 每条激光线 分成6段
    for (int j = 0; j < 6; j++) {
      // 每一段的起始index
      int start_point = (cloud_info_.start_ring_index[i] * (6 - j) +
                         cloud_info_.end_ring_index[i] * j) /
                        6.0;
      // 每一段的结束index
      int end_point = (cloud_info_.start_ring_index[i] * (5 - j) +
                       cloud_info_.end_ring_index[i] * (j + 1)) /
                          6.0 -
                      1;
      if (start_point >= end_point)
        continue;

      // std::sort(cloud_smoothness_.begin() + start_point,
      //           cloud_smoothness_.begin() + end_point, by_value());
      // note: 更改为lamda表达式
      // 1/6 份的点云进行曲率从小到大排序
      std::sort(cloud_smoothness_.begin() + start_point,
                cloud_smoothness_.begin() + end_point,
                [](Smoothness const &left, Smoothness const &right) -> bool {
                  return left.value < right.value;
                });

      int max_curvature_picked_num = 0;
      // 提取角特征点：从大到小遍历
      for (int k = end_point; k >= start_point; k--) {
        // 激光点的index
        int index = cloud_smoothness_[k].index;
        // 激光点未被处理 且是 角点
        if (cloud_neighbor_picked_[index] == 0 &&
            cloud_curvature_[index] > edge_threshold) {
          // 角点特征点数 + 1
          max_curvature_picked_num++;
          // 仅仅提取20个角特征点
          if (max_curvature_picked_num <= 20) {
            // 角点的点云label=1
            cloud_label_[index] = 1;
            // 存放角特征点
            corner_point_cloud_->emplace_back(
                extracted_point_cloud_->points[index]);
          } else {
            break;
          }
          // 将该点标注为处理过
          cloud_neighbor_picked_[index] = 1;

          // 为避免特征点太密集，将确定为角特征点的前后5个点全部做免处理表示
          // 1. 后（前）5个点
          for (int l = 1; l <= 5; l++) {
            int column_diff = std::abs(static_cast<int>(
                cloud_info_.point_column_index[index + l] -
                cloud_info_.point_column_index[index + l - 1]));
            // 有时候一条扫描线上激光点并不是连续的（中间有可能存在空洞啥的，激光不返回）
            // 此时就算相邻的2个点，也相差很远
            if (column_diff > 10)
              break;
            // 即 标记为 处理过(实际就没处理)
            cloud_neighbor_picked_[index + l] = 1;
          }
          // 1. 前（后）5个点 ， 道理同上
          for (int l = -1; l >= -5; l--) {
            int column_diff = std::abs(static_cast<int>(
                cloud_info_.point_column_index[index + l] -
                cloud_info_.point_column_index[index + l - 1]));

            if (column_diff > 10)
              break;

            cloud_neighbor_picked_[index + l] = 1;
          }
        }
      }
      // 提取平面特征点：从小到大遍历
      for (int k = start_point; k <= end_point; k++) {
        // 激光点的index
        int index = cloud_smoothness_[k].index;
        // 激光点未被处理 且是 平面特征点
        if (cloud_neighbor_picked_[index] == 0 &&
            cloud_curvature_[index] < surf_threshold) {
          // 平面特征点label = -1
          cloud_label_[index] = -1;
          // note: 与角特征点不同，这里没有直接存放
          // note: label=1 角特征点，label=-1 平面特征点, 
          // note: label=0 普通没有处理的点(除去那些特征点前后各5个)
          // 标记为已处理
          cloud_neighbor_picked_[index] = 1;

          // 下面2个for{}的意义，同上角特征点的处理原因
          for (int l = 1; l <= 5; l++) {
            int column_diff = std::abs(static_cast<int>(
                cloud_info_.point_column_index[index + l] -
                cloud_info_.point_column_index[index + l - 1]));

            if (column_diff > 10)
              break;

            cloud_neighbor_picked_[index + l] = 1;
          }

          for (int l = -1; l >= -5; l--) {
            int column_diff = std::abs(static_cast<int>(
                cloud_info_.point_column_index[index + l] -
                cloud_info_.point_column_index[index + l - 1]));

            if (column_diff > 10)
              break;

            cloud_neighbor_picked_[index + l] = 1;
          }
        }
      }

      for (int k = start_point; k <= end_point; k++) {
          // note: label=0 普通没有处理的点(除去那些特征点前后各5个) 
          // note: label=-1 平面特征点  全部划分为 非角特征点大类
        if (cloud_label_[k] <= 0) {
          surface_cloud_scan->emplace_back(extracted_point_cloud_->points[k]);
        }
      }
    }
    // 非角特征点大类 回收的特征点太多了，降采样一下
    surface_cloud_scan_downsampping->clear();
    down_size_filter_.setInputCloud(surface_cloud_scan);
    down_size_filter_.filter(*surface_cloud_scan_downsampping);
    // 角特征点是一个一个入栈的，平面特征点要 N_SCAN 相加
    *surface_point_cloud_ += *surface_cloud_scan_downsampping;

  }
}

void FeatureExtraction::FreeCloudInfoMemory() {
  cloud_info_.start_ring_index.clear();
  cloud_info_.end_ring_index.clear();
  cloud_info_.point_column_index.clear();
  cloud_info_.point_range.clear();
}

void FeatureExtraction::PublishFeatureClould() {

  FreeCloudInfoMemory();

  cloud_info_.cloud_corner =
      PublishCloud(pub_corner_points_, corner_point_cloud_, cloud_header_.stamp,
                   lidar_frame);

  cloud_info_.cloud_surface =
      PublishCloud(pub_surface_points_, surface_point_cloud_,
                   cloud_header_.stamp, lidar_frame);

  pub_point_cloud_info_.publish(cloud_info_);
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = false; //设置日志消息是否转到标准输出而不是日志文件
  FLAGS_alsologtostderr = true; //设置日志消息除了日志文件之外是否去标准输出
  FLAGS_log_prefix = true; //设置日志前缀是否应该添加到每行输出
  FLAGS_log_dir = "../log"; //预创建好

  ros::init(argc, argv, "lio_sam");

  FeatureExtraction feature_extraction;

  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  google::ShutdownGoogleLogging();

  return 0;
}
