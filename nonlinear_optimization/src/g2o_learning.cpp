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
#include <functional>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
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
  virtual void SetToOriginImpl() override { _estimate << 0, 0, 0; }
  // update
  virtual void OplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // 存盘和读盘：留空
  virtual bool Read(std::istream &in) {}

  virtual bool Write(std::ostream &out) {}
};

// edge模版重载：观测数据维度，数据类型，连接的vertex类型
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), x_(x) {}
  // 计算曲线的误差模型
  virtual void ComputeError override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement - func(abc(0, 0), abc(1, 0), abc(2, 0), x_);
  }

  virtual void LinearizeOplus() override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = func(abc[0], abc[1], abc[2], x_);
    _jacobianOplusXi[0] = -x_ * x_ * y;
    _jacobianOplusXi[1] = -x_ * y;
    _jacobianOplusXi[0] = -y;
  }
  // 计算Jacobian
  // 存盘和读盘：留空
  virtual bool Read(std::istream &in) {}

  virtual bool Write(std::ostream &out) {}

private:
  double x_; // x 值， y 值为 _measurement
};

int main(int argc, char const *argv[]) {
  DLOG(INFO) << "=====  g2o learning  =====";
  return 0;
}
