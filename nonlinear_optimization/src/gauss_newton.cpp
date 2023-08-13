/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/13/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     gauss_newton.cpp
 * @brief          :     learning gauss_newton
 * ===================================================================*/
#include <chrono>
#include <functional>
#include <iostream>
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class GaussNewton {
private:
  // 原函数
  std::function<double(double, double, double, double)> func =
      [](double a, double b, double c, double x) {
        return std::exp(a * x * x + b * x + c);
      };
  // 函数参数
  double a_r = 1.0, b_r = 2.0, c_r = 1.0; // 真实曲线参数a，b，c
  double a_e = 2.0, b_e = -1.0, c_e = 5.0; //估计曲线参数，a,b,c为初值，随意赋值
  double w_sigma = 1.0;                    // 噪声方差的sigma
  double inv_w_sigma = 1.0 / w_sigma; // sigma的倒数
  cv::RNG rng;                        // opencv的随机数生成器

  std::vector<double> x_data, y_data; // 数据存放容器
private:
  void GenerateData(const int &N);
  void GaussNewtonAlgorithm(const int &max_iteration);
  void Draw();

public:
  // 如果不加入main友元，main()无法调用private
  friend int main(int argc, char const *argv[]);

  GaussNewton();
  ~GaussNewton();
};

GaussNewton::GaussNewton() {}

GaussNewton::~GaussNewton() {}

void GaussNewton::GenerateData(const int &N) {
  for (int i = 0; i < N; i++) {
    double x = i / 100.0; // 随心所欲的设置x
    // double y =
    //     std::exp(a_r * x * x + b_r * x + c_r) + rng.gaussian(w_sigma *
    //     w_sigma);
    double y = func(a_r, b_r, c_r, x) + rng.gaussian(w_sigma * w_sigma);
    this->x_data.emplace_back(x);
    this->y_data.emplace_back(y);
  }
}

void GaussNewton::Draw() {
  // 带有噪声的真实数据
  std::map<std::string, std::string> keywords;
  keywords.insert(
      std::pair<std::string, std::string>("label", "sampling points"));
  plt::scatter(this->x_data, this->y_data, 1, keywords);

  // 真实模型
  std::vector<double> xt_data;
  std::vector<double> yt_data;
  for (int i = 0; i < 100; i++) {
    xt_data.emplace_back(i / 100.0);
    yt_data.emplace_back(this->func(a_r, b_r, c_r, i / 100.0));
  }
  plt::named_plot("groundtruth model", xt_data, yt_data, "k-");

  // 估计模型
  std::vector<double> xe_data;
  std::vector<double> ye_data;
  for (int i = 0; i < 100; i++) {
    xe_data.emplace_back(i / 100.0);
    ye_data.emplace_back(this->func(a_e, b_e, c_e, i / 100.0));
  }
  plt::named_plot("estimated model", xe_data, ye_data, "r-");

  plt::title("Gauss Newton Algorithm");
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::show();
}

void GaussNewton::GaussNewtonAlgorithm(const int &max_iteration) {
  double curr_cost = 0, last_cost = 0; // 当前cost，上一次cost
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < max_iteration; iter++) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();

    for (int i = 0; i < this->x_data.size(); i++) {
      double x_i = this->x_data[i];
      double y_i = this->y_data[i];
      double error = y_i - this->func(a_e, b_e, c_e, x_i);
      Eigen::Vector3d JacobianMatrix;
      JacobianMatrix[0] = -x_i * x_i * this->func(a_e, b_e, c_e, x_i);
      JacobianMatrix[1] = -x_i * this->func(a_e, b_e, c_e, x_i);
      JacobianMatrix[2] = -1 * this->func(a_e, b_e, c_e, x_i);

      H += inv_w_sigma * inv_w_sigma * JacobianMatrix *
           JacobianMatrix.transpose();
      g += -1 * inv_w_sigma * inv_w_sigma * error * JacobianMatrix;

      curr_cost += error * error;
    }

    // 解方程 Hx=g
    // note: 很重要，矩阵分解 解算矩阵
    Eigen::Vector3d dx = H.ldlt().solve(g);
    // 无解
    if (isnan(dx[0])) {
      std::cout << "reslut is nan!" << std::endl;
      break;
    }
    // 代价不再减小
    if (iter > 0 && curr_cost >= last_cost) {
      std::cout << "-------" << iter << "-------" << std::endl;
      std::cout << "current_cost = " << curr_cost
                << "\nlast_cost = " << last_cost << "\nstop iteration!"
                << std::endl;
    }

    a_e += dx[0];
    b_e += dx[1];
    c_e += dx[2];

    last_cost = curr_cost;
    std::cout << "-------" << iter << "-------" << std::endl;
    std::cout << "total cost =  " << curr_cost
              << " , \t\tupdate =  " << dx.transpose()
              << "\t\testimated params =   " << a_e << ", " << b_e << ", "
              << c_e << std::endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double time_used = std::chrono::duration<double>(t2 - t1).count();
    std::cout << "solve time cost : " << time_used << "s." << std::endl;
    std::cout << "estimated params a,b,c = " << a_e << ", " << b_e << ", "
              << c_e << std::endl;
  }
}

int main(int argc, char const *argv[]) {
  GaussNewton gn;
  int data_size = 100;     // 数据量
  int max_iteration = 100; // 最大迭代次数
  gn.GenerateData(data_size);
  gn.GaussNewtonAlgorithm(max_iteration);
  gn.Draw();
  return 0;
}
