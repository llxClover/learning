/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/07
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     trajectory_error.cpp
 * @brief          :     calculate the error between groundTruth and estimation.
 * ===================================================================*/
#include <assert.h>
#include <fstream>
#include <iostream>
#include <string>

#include "matplotlibcpp.h"
#include "sophus/se3.hpp"

namespace plt = matplotlibcpp;

std::string groundtruth_file = "../data/groundtruth.txt";
std::string estimated_file = "../data/estimated.txt";

// 定义一个轨迹的数据格式
typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
    TrajectoryType;

TrajectoryType ReadTrajectory(std::string &file) {
  TrajectoryType trajectory;

  std::fstream in_file;
  in_file.open(file, std::ios::in);
  if (!in_file.is_open()) {
    std::cout << "Error opening the target file ! " << std::endl;
    return trajectory;
  } else {
    while (!in_file.eof()) {
      double time, x, y, z, qx, qy, qz, qw;
      in_file >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
      Sophus::SE3d tmp_se3d(Eigen::Quaterniond(qw, qx, qy, qz),
                            Eigen::Vector3d(x, y, z));
      trajectory.emplace_back(tmp_se3d);
    }
  }
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {}

int main(int argc, char const *argv[]) {
  TrajectoryType traj_gt = ReadTrajectory(groundtruth_file);
  TrajectoryType traj_esti = ReadTrajectory(estimated_file);
  // 验证一下读到了数据没
  assert(!traj_gt.empty() && !traj_esti.empty());
  // 验证一下两个数据是不是同样的维度大小
  assert(traj_gt.size() == traj_esti.size());

  // 计算均方根误差RMSE
  double rmse = 0;
  double error = 0;
  for (int i = 0; i < traj_gt.size(); i++) {
    error = (traj_gt[i].inverse() * traj_esti[i]).log().norm();
    rmse += error * error;
  }

  rmse /= static_cast<double>(traj_gt.size());
  rmse = std::sqrt(rmse);
  std::cout << "RMSE = " << rmse << std::endl;

  DrawTrajectory(traj_gt, traj_esti);

  return 0;
}
