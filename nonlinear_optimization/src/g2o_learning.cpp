/* ==================================================================
 * Copyright (c) 2023 lilinxiang. All rights reserved.
 * @Author         :     lilinxiang
 * @Created Date   :     2023/08/17/
 * @Email          :     lilinxiang@tsari.tsinghua.edu.cn
 *
 * @file           :     g2o_learning.cpp
 * @brief          :     fitting curve by g2olib
 * ===================================================================*/
#include "matplotlibcpp.h"
#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/core/core.hpp>

namespace plt = matplotlibcpp;

// 原函数
std::function<double(double, double, double, double)> func =
    [](double a, double b, double c, double x) -> double {
  return std::exp(a * x * x + b * x + c);
};

// vertex模版重载：优化变量维度，数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // reset
  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }
  // update
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // 存盘和读盘：留空
  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}
};

// edge模版重载：观测数据维度，数据类型，连接的vertex类型
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), x_(x) {}
  // 计算曲线的误差模型
  virtual void computeError() override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = vertex->estimate();
    _error(0, 0) = _measurement - func(abc(0, 0), abc(1, 0), abc(2, 0), x_);
  }

  virtual void linearizeOplus() override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = vertex->estimate();
    double y = func(abc[0], abc[1], abc[2], x_);
    _jacobianOplusXi[0] = -x_ * x_ * y;
    _jacobianOplusXi[1] = -x_ * y;
    _jacobianOplusXi[0] = -y;
  }
  // 计算Jacobian
  // 存盘和读盘：留空
  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}

public:
  double x_; // x 值， y 值为 _measurement
};

int main(int argc, char const *argv[]) {
  DLOG(INFO) << "=====  g2o learning  =====";
  return 0;
}

class G2oCurveFitting {
private:
  // 函数参数
  double a_r = 1.0, b_r = 2.0, c_r = 1.0; // 真实曲线参数a，b，c
  double w_sigma = 1.0;                   // 噪声方差的sigma
  double inv_w_sigma = 1.0 / w_sigma;     // sigma的倒数
  cv::RNG rng;                            // opencv的随机数生成器
  // note: 这是ceres估计参数,gauss_newton代码中的a_e,b_e,c_e
  double abc[3] = {2.0, -1.0, 5.0};
  int N = 100;                        // 数据量
  std::vector<double> x_data, y_data; // 数据存放容器

  // 构建图优化，先设定g2o
  // 每个误差项的优化变量维度为3，误差值的维度为1
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
  // 线性求解器类型
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  g2o::SparseOptimizer optimizer; // 图模型

private:
  void GenerateData(const int &N);
  void Draw(const std::string figname);

public:
  G2oCurveFitting();
  ~G2oCurveFitting();
};

G2oCurveFitting::G2oCurveFitting() {}

G2oCurveFitting::G2oCurveFitting() {
  // 梯度下降方法：有GN，LM，DogLeg法可选
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

  this->optimizer.setAlgorithm(solver); // 设置求解器
  this->optimizer.setVerbose(true);     // 打开调试输出

  G2oCurveFitting::GenerateData(this->N);
}

G2oCurveFitting::~G2oCurveFitting() {}

void G2oCurveFitting::GenerateData(const int &N) {
  for (int i = 0; i < N; i++) {
    double x = i / 100.0; // 随心所欲的设置x
    double y = func(a_r, b_r, c_r, x) + rng.gaussian(w_sigma * w_sigma);
    this->x_data.emplace_back(x);
    this->y_data.emplace_back(y);
  }
}

void G2oCurveFitting::Draw(const std::string figname) {
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