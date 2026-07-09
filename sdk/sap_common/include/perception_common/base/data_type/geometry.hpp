/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * He Yuan <heyuan@sensetime.com>
 */

#pragma once

#include "Eigen/Core"

#include "perception_common/adapter/device_adapter.hpp"

namespace senseAD {
namespace perception {

typedef HPC::fastcv::Point2i Point2i;
typedef HPC::fastcv::Point2f Point2f;
typedef HPC::fastcv::Point3f Point3f;
typedef HPC::fastcv::Recti Rect;
typedef HPC::fastcv::Mat<float, 3, FASTCV_DATA_TYPE_DEFAULT> ImageMatf;

typedef Eigen::NumTraits<double> EigenDouble;
typedef Eigen::MatrixXd MatXd;
typedef Eigen::VectorXd VecXd;
typedef Eigen::Matrix<double, 3, 3> Mat3d;
typedef Eigen::Matrix<double, 4, 4> Mat4d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> MatX8d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;

typedef Eigen::NumTraits<float> EigenFloat;
typedef Eigen::MatrixXf MatXf;
typedef Eigen::VectorXf VecXf;
typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<float, 4, 4> Mat4f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 8> MatX8f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;

}  // namespace perception
}  // namespace senseAD
