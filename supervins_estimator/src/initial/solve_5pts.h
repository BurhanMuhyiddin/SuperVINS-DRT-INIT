/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>

#include "../estimator/feature_manager.h"

class MotionEstimator
{
  public:

    static bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);
    static bool relativePose(const FeatureManager &f_manager, Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
};


