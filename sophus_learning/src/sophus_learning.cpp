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
  
  

  return 0;
}
