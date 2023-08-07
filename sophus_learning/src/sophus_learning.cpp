/* ==================================================================
* Copyright (c) 2023 lilinxiang. All rights reserved.
* @Author         :     lilinxiang
* @Created Date   :     2023/08/06
* @Email          :     lilinxiang@tsari.tsinghua.edu.cn
* 
* @file           :     sophus_learning.cpp
* @brief          :     learning sophus essential operation
* ===================================================================*/
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

int main(int argc, char const *argv[]) {
  // Eigen::AngleAxisd (旋转角度， 绕着旋转的旋转轴向量)
  // 绕z轴旋转pi/2
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  // Sophus::SO(3) 可以直接由旋转矩阵构造
  Sophus::SO3d SO3_R(R);

  Eigen::Quaterniond q(R);
  Sophus::SO3d SO3_q(q);

  std::cout << "SO(3) from matrix :\n" << SO3_R.matrix() << std::endl;
  std::cout << "SO(3) from quatation :\n" << SO3_q.matrix() << std::endl;

  Eigen::Vector3d so3 = SO3_R.log(); //使用对数映射：李群->李代数
  std::cout << "so3 = \n" << so3.transpose() << std::endl;

  // hat: 向量->反对称矩阵
  Eigen::Matrix3d so3_hat = Sophus::SO3d::hat(so3);
  std::cout << "vector to hat matrix by hat:\n" << so3_hat << std::endl;

  // vee: 反对称矩阵->向量
  Eigen::Vector3d so3_hat_vee = Sophus::SO3d::vee(so3_hat);
  std::cout << "hat matrix to vector by vee:\n"
            << so3_hat_vee.transpose() << std::endl;

  // 增量扰动模型(李群)
  std::cout << "-------增量扰动模型SO3-------" << std::endl;
  Eigen::Vector3d update_so3(1e-4, 0, 0); // 增量
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  std::cout << "SO3 updated= \n" << SO3_updated.matrix() << std::endl;

  std::cout << "**************************" << std::endl;
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d SE3_Rt(R, t);
  Sophus::SE3d SE3_qt(q, t);
  std::cout << "SE3 structed from (R,t) : \n" << SE3_Rt.matrix() << std::endl;
  std::cout << "SE3 structed from (q,t) : \n" << SE3_qt.matrix() << std::endl;
  // 李代数se(3)是6维向量
  Eigen::VectorXd se3 = SE3_Rt.log();
  std::cout << "se3= " << se3.transpose() << std::endl;

  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3_1 = SE3_qt.log();
  std::cout << "se3_1= " << se3_1.transpose() << std::endl;

  std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3_1).matrix() << std::endl;
  std::cout << "se3 vee = \n"
            << Sophus::SE3d::vee(Sophus::SE3d::hat(se3_1).matrix()).transpose()
            << std::endl;

  std::cout << "-------增量扰动模型SE3-------" << std::endl;
  Vector6d update_se3;
  // update_se3 << 1e-4, 0, 0, 0, 0, 0;
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  std::cout << "Se3 updated : \n" << SE3_updated.matrix() << std::endl;

  return 0;
}
