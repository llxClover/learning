/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/13/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     ceres_learning.cpp
 * @brief          :     learning ceres lib
 * ===================================================================*/
#include <chrono>
#include <functional>
#include <iostream>

#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// 原函数
std::function<double(double, double, double, double)> func =
    [](double a, double b, double c, double x) -> double {
  return ceres::exp(a * x * x + b * x + c);
};

// 代价函数的计算模型
class CurveFittingCost {
public:
  friend int main(int argc, char const *argv[]);

  CurveFittingCost() {}
  ~CurveFittingCost() {}
  CurveFittingCost(double x, double y) : x_(x), y_(y) {}

private:
  double x_;
  double y_;

public:
  template <typename T> bool operator()(const T *const abc, T *residual) const;
};

template <typename T>
bool CurveFittingCost::operator()(const T *const abc, T *residual) const {
  // y-exp(ax^2+bx+c)
  residual[0] = T(this->y_) -
                ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);
  return true;
}

// ceres类（自动求导类型）
class CeresFitting : public CurveFittingCost {
public:
  friend int main(int argc, char const *argv[]);

  CeresFitting() {}
  ~CeresFitting() {}

private:
  // 函数参数
  double a_r = 1.0, b_r = 2.0, c_r = 1.0; // 真实曲线参数a，b，c
  double w_sigma = 1.0;                   // 噪声方差的sigma
  double inv_w_sigma = 1.0 / w_sigma;     // sigma的倒数
  cv::RNG rng;                            // opencv的随机数生成器
  // note: 这是ceres估计参数,gauss_newton代码中的a_e,b_e,c_e
  double abc[3] = {2.0, -1.0, 5.0};
  std::vector<double> x_data, y_data; // 数据存放容器

private:
  void GenerateData(const int &N);
  void Draw(const std::string figname);

private:
  void ConstructCeresSolver();
};

void CeresFitting::GenerateData(const int &N) {
  for (int i = 0; i < N; i++) {
    double x = i / 100.0; // 随心所欲的设置x
    double y = func(a_r, b_r, c_r, x) + rng.gaussian(w_sigma * w_sigma);
    this->x_data.emplace_back(x);
    this->y_data.emplace_back(y);
  }
}

void CeresFitting::ConstructCeresSolver() {
  //   构建最小二乘问题
  ceres::Problem problem;
  for (int i = 0; i < x_data.size(); i++) {
    problem.AddResidualBlock(
        // 使用自动求导，模版参数为： 误差类型 输出维度 输入维度
        // (维度要与误差类一致)
        new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 3>(
            new CurveFittingCost(x_data[i], y_data[i])),
        nullptr, //核函数，这里不使用
        abc      //待估计的参数
    );
  }

  // 配置相关的求解器参数
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;               // 输出到cout

  ceres::Solver::Summary summary; //优化信息
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  // 开始求解优化
  ceres::Solve(options, &problem, &summary);

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double time_used = std::chrono::duration<double>(t2 - t1).count();
  std::cout << "ceres sloved the problem time used : " << time_used << "s."
            << std::endl;

  // 查看结果
  std::cout << "=========  ceres summary  =========" << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "*********estimated params : *********" << std::endl;
  for (auto a : abc) {
    std::cout << a << ", ";
  };
  std::cout << std::endl;
}

void CeresFitting::Draw(const std::string figname) {
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
    yt_data.emplace_back(func(a_r, b_r, c_r, i / 100.0));
  }
  plt::named_plot("groundtruth model", xt_data, yt_data, "k-");

  // 估计模型
  std::vector<double> xe_data;
  std::vector<double> ye_data;
  for (int i = 0; i < 100; i++) {
    xe_data.emplace_back(i / 100.0);
    ye_data.emplace_back(
        func(this->abc[0], this->abc[1], this->abc[2], i / 100.0));
  }
  plt::named_plot("estimated model", xe_data, ye_data, "r-");

  plt::title(figname);
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::show();
}

/**
 * *********************指定jacobian求导******************************
 */

class CostSpecifiedJacobian : public ceres::SizedCostFunction<1, 3> {
public:
  CostSpecifiedJacobian(){};
  // note:如果我们定义了有参构造函数，但没有定义无参构造函数，那么创建对象时就必须传入参数，
  // note:否则编译器会报错。如果同时定义了无参构造函数和有参构造函数，可以根据需要选择调用哪个构造函数。
  CostSpecifiedJacobian(const double x, const double y) : x_(x), y_(y) {}
  // ~CostSpecifiedJacobian() {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

private:
  double x_; // const变量必须初始化
  double y_;
};

bool CostSpecifiedJacobian::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  const double a = parameters[0][0];
  const double b = parameters[0][1];
  const double c = parameters[0][2];

  // residuals[0] = y_ - std::exp(a * x_ * x_ + b * x_ + c);
  residuals[0] = y_ - func(a, b, c, x_);

  if (!jacobians) {
    return true;
  }

  double *jacobian = jacobians[0];
  if (!jacobian) {
    return true;
  }
  jacobian[0] = -1 * func(a, b, c, x_) * x_ * x_;
  jacobian[1] = -1 * func(a, b, c, x_) * x_;
  jacobian[2] = -1 * func(a, b, c, x_) * 1;
  return true;
};

// ceres类（给定jacobian求导类型）
class CeresSpecifiedJacobian : public CostSpecifiedJacobian
{
public:
  CeresSpecifiedJacobian() {}
  ~CeresSpecifiedJacobian() {}

private:
  // 函数参数
  double a_r = 1.0, b_r = 2.0, c_r = 1.0; // 真实曲线参数a，b，c
  double w_sigma = 1.0;                   // 噪声方差的sigma
  double inv_w_sigma = 1.0 / w_sigma;     // sigma的倒数
  cv::RNG rng;                            // opencv的随机数生成器
  // note: 这是ceres估计参数,gauss_newton代码中的a_e,b_e,c_e
  double abc[3] = {2.0, -1.0, 5.0};
  std::vector<double> x_data, y_data; // 数据存放容器

public:
  void GenerateData(const int &N);
  void Draw(const std::string figname);

public:
  void ConstructSpecifiedJacobianCeresSolver();
};

void CeresSpecifiedJacobian::ConstructSpecifiedJacobianCeresSolver() {
  //   构建最小二乘问题
  ceres::Problem problem;
  for (int i = 0; i < x_data.size(); i++) {
    ceres::CostFunction *cost_function =
        new CeresSpecifiedJacobian::CostSpecifiedJacobian(x_data[i], y_data[i]);
    problem.AddResidualBlock(cost_function,
                             nullptr, //核函数，这里不使用
                             abc      //待估计的参数
    );
  }

  // 配置相关的求解器参数
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;               // 输出到cout

  ceres::Solver::Summary summary; //优化信息
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  // 开始求解优化
  ceres::Solve(options, &problem, &summary);

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double time_used = std::chrono::duration<double>(t2 - t1).count();
  std::cout << "ceres of specified jacobian sloved the problem time used : "
            << time_used << "s." << std::endl;

  // 查看结果
  std::cout << "=========  ceres of specified jacobian summary  ========="
            << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "*********estimated params : *********" << std::endl;
  for (auto a : abc) {
    std::cout << a << ", ";
  };
  std::cout << std::endl;
}

void CeresSpecifiedJacobian::GenerateData(const int &N) {
  for (int i = 0; i < N; i++) {
    double x = i / 100.0; // 随心所欲的设置x
    double y = func(a_r, b_r, c_r, x) + rng.gaussian(w_sigma * w_sigma);
    this->x_data.emplace_back(x);
    this->y_data.emplace_back(y);
  }
}


void CeresSpecifiedJacobian::Draw(const std::string figname) {
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
    yt_data.emplace_back(func(a_r, b_r, c_r, i / 100.0));
  }
  plt::named_plot("groundtruth model", xt_data, yt_data, "k-");

  // 估计模型
  std::vector<double> xe_data;
  std::vector<double> ye_data;
  for (int i = 0; i < 100; i++) {
    xe_data.emplace_back(i / 100.0);
    ye_data.emplace_back(
        func(this->abc[0], this->abc[1], this->abc[2], i / 100.0));
  }
  plt::named_plot("estimated model", xe_data, ye_data, "r-");

  plt::title(figname);
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::show();
}

int main(int argc, char const *argv[]) {
  CeresFitting ceres_fitting;
  CeresSpecifiedJacobian ceres_jacobian_fitting;

  int data_size = 100;     // 数据量
  int max_iteration = 100; // 最大迭代次数

  ceres_fitting.GenerateData(data_size);
  ceres_fitting.ConstructCeresSolver();
  // note:打开画图，显示的时间会不准
  // ceres_fitting.Draw("Ceres of Auto Diff Fitting Curve");

  ceres_jacobian_fitting.GenerateData(data_size);
  ceres_jacobian_fitting.ConstructSpecifiedJacobianCeresSolver();
  // note:打开画图，显示的时间会不准
  // ceres_jacobian_fitting.Draw("Ceres of Specified Jacobian Fitting Curve");

  return 0;
}
