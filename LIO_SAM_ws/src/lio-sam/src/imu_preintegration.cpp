/* ==================================================================
* Copyright (c) 2023 lilinxiang. All rights reserved.
* @Author         :     lilinxiang
* @Created Date   :     2023/xx/xx/
* @Email          :     lilinxiang@tsari.tsinghua.edu.cn
* 
* @file           :     imu_preintergration.cpp
* @brief          :     一步一步实现，并验证LIO-SAM的imuPreintergration.cpp
* ===================================================================*/
#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
