/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/xx/xx/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     coordinate_transform.cpp
 * @brief          :
 * 2个相机观测一个物体，相机1的位姿q1[0.35,0.2,0.3,0.1],t1[0.3,0.1,0.1];
 *                       相机2位姿q2[-0.5,0.4,-0.1,0.2],t2[-0.1,0.5,0.3];
 * 相机1下，物体的坐标为[0.5,0,0.2] 求：相机2下物体的坐标
 * ===================================================================*/
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char const *argv[]) {
  Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  // note: 四元数在使用的时候必须归一化
  q1.normalize();
  Eigen::Vector3d t1(0.3, 0.1, 0.1);
  Eigen::Isometry3d T1w(q1);
  // camera-1在世界坐标系下的位姿，表示为T的时候，写为T_camera1_w,
  // 表示w旋转到camera下的量，并不是camera旋转到w下的量
  T1w.pretranslate(t1);

  Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
  q2.normalize();
  Eigen::Vector3d t2(-0.1, 0.5, 0.3);
  Eigen::Isometry3d T2w(q2);
  T2w.pretranslate(t2);

  Eigen::Vector3d target_point(0.5, 0, 0.2);

  Eigen::Vector3d reslut_point = T2w * T1w.inverse() * target_point;

  std::cout << "the point in camera_2 is:\n"
            << reslut_point.transpose() << std::endl;

  return 0;
}
