/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/06/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     eigen_geometry.cpp
 * @brief          :     learning libeigen geometry operation
 * ===================================================================*/
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char const *argv[]) {
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));

  std::cout.precision(6);
  std::cout << "rotation mattrix = \n" << rotation_vector.matrix() << std::endl;
  rotation_matrix = rotation_vector.toRotationMatrix();

  Eigen::Vector3d v(1, 0, 0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  std::cout << "(1, 0, 0) rotated by rotation vector :\n"
            << v_rotated.transpose() << std::endl;

  v_rotated = rotation_matrix * v;
  std::cout << "(1, 0, 0) rotated by rotation vmatrix :\n"
            << v_rotated.transpose() << std::endl;

  // 旋转矩阵可以直接转换成欧拉角
  // ZYX顺序，即为 RPY角
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  std::cout << "R P Y : " << euler_angles.transpose() << std::endl;

  // 欧式变换矩阵使用Eigen::Isometry
  // 仿射变换使用：Eigen::Affine
  // 影射变换使用：Eigen::Projective

  // 看似3d，实则4x4
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(rotation_vector);
  std::cout << "T rotated by rotation vector: \n" << T.matrix() << std::endl;
  T.rotate(rotation_matrix);
  std::cout << "T rotated by rotation matrix: \n" << T.matrix() << std::endl;

  // 相当于 R*v+t
  T.pretranslate(Eigen::Vector3d(1, 3, 4));
  std::cout << "T translate by (1,3,4): \n" << T.matrix() << std::endl;
  Eigen::Vector3d v_transformed = T * v;
  std::cout << "v_transformed : \n" << v_transformed.transpose() << std::endl;

  // 四元数
  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  q = Eigen::Quaterniond(rotation_matrix);
  // coeffs:顺序是 （x,y,z,w）
  std::cout << "quatation: \n " << q.coeffs() << std::endl;
  // 四元数旋转一个向量
  v_rotated = q*v; // 数学实现为 qvq^{-1}

  std::cout << "(1,0,0) rotated by quaterntion :\n" << v_rotated.transpose()<<std::endl;


  return 0;
}
