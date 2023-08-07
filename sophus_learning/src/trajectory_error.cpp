/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/xx/xx/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     trajectory_error.cpp
 * @brief          :     calculate the error between groundTruth and estimation.
 * ===================================================================*/
#include <fstream>
#include <iostream>
#include <string>

#include "sophus/se3.hpp"

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
    return trajectory;
  }
}
