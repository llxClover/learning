/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/05/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     eigen_learning.cpp
 * @brief          :     learning libeigen matrix operation
 * ===================================================================*/
#include <ctime> // 计时
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#define MATRIX_SIZE 50

int main(int argc, char *argv[]) {

  Eigen::Matrix<float, 2, 3> matrix_23;
  Eigen::Vector3d v_3d; // Eigen::Vector<double, 3, 1>
  Eigen::Matrix<double, 3, 3> matrix_33 = Eigen::Matrix3d::Zero();

  // 动态矩阵
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
  // 简单的动态矩阵写法
  Eigen::MatrixXd matrix_x;

  matrix_23 << 1, 2, 3, 4, 5, 6;
  std::cout << matrix_23 << std::endl;

  for (int i = 0; i < 1; i++) {
    for (int j = 0; j < 2; j++) {
      std::cout << matrix_23(i, j) << std::endl;
    }
  }

  v_3d << 3, 2, 1;

  //矩阵相乘  //note: 强制数据转换  data.cast<double>() eigen内部不支持自动数据转换
  Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  std::cout << result << std::endl;

  // 随机生成矩阵
  matrix_33 = Eigen::Matrix3d::Random();

  std::cout << matrix_33 << std::endl;
  std::cout << matrix_33.transpose() << std::endl;
  std::cout << matrix_33.sum() << std::endl;
  std::cout << matrix_33.trace() << std::endl;
  std::cout << 10 * matrix_33 << std::endl;
  std::cout << matrix_33.determinant() << std::endl;
  std::cout << matrix_33.inverse() << std::endl;

  // 特征值 特征向量
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      matrix_33.transpose() * matrix_33);
  std::cout << eigen_solver.eigenvalues() << std::endl;
  std::cout << eigen_solver.eigenvectors() << std::endl;

  // 解方程
  Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN =
      Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE>::Random();
  Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd =
      Eigen::Matrix<double, MATRIX_SIZE, 1>::Random();
  clock_t time_start = clock();
  // 方法1：直接求逆运算
  //   note: 计算消耗时间： 1000 * (clock() - time_start) /
  //   note: (double)CLOCKS_PER_SEC 单位ms
  Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;

  std::cout << "time used in normal invers is :   "
            << 1000 * (clock() - time_start) / (double)CLOCKS_PER_SEC << "ms"
            << std::endl;
  // 矩阵分解，快速求解
  time_start = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);

  std::cout << "time used in QR compsition is :   "
            << 1000 * (clock() - time_start) /
                   static_cast<double> CLOCKS_PER_SEC
            << "ms" << std::endl;

  return 0;
}
