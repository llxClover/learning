/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/xx/xx/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     feature_extraction.cpp
 * @brief          :     一步一步实现，并验证LIO-SAM的eatureExtraction.cpp
 * ===================================================================*/
#include "lio_sam/cloud_info.h"
#include "utility.h"

#include <glog/logging.h>

class FeatureExtraction : public ParamServer {
private:
  /* data */

private:
  void InitializationValue();

  void PointCloudeInfoHandler(const lio_sam::cloud_infoConstPtr &msg_in);

  void CalculateSmoothness();

  void MarkOccludePoints();

  void ExtractFeatures();

  void FreeCloudInfoMemory();

  void PublishFeatureClould();

public:
  FeatureExtraction(/* args */);
  ~FeatureExtraction();
};

FeatureExtraction::FeatureExtraction(/* args */) {}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::PointCloudeInfoHandler(
    const lio_sam::cloud_infoConstPtr &msg_in) {}

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
