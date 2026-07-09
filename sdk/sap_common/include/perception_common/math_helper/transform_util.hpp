/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * He Yuan <heyuan@sensetime.com>
 * Luo Quan <luoquan@sensetime.com>
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

#include "perception_common/base/data_type/geometry.hpp"
#include "perception_common/math_helper/geometry/perspective_tf.hpp"

namespace senseAD {
namespace perception {

// iniline util functions
inline apStatus_t RTTransformPoint(const Eigen::Matrix2f &rotate,
                                   const Eigen::Vector2f &transform,
                                   Point2f *point) {
    if (nullptr == point) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    Vec2f p = Eigen::Vector2f::Zero();
    p(0) = point->x;
    p(1) = point->y;
    auto out = rotate * p + transform;
    point->x = out(0);
    point->y = out(1);

    return AP_SUCCESS;
}

inline apStatus_t CompensateEgoMotion(const float dt_s,
                                      const Eigen::Vector3f &ego_v,
                                      const Eigen::Vector3f &ego_angular_v,
                                      Eigen::VectorXf *state) {
    if (state->rows() < 4) {
        AP_LERROR(CompensateEgoMotion) << "Least input format: x, y, vx, vy";
        AP_CHECK_ERROR(AP_INVALID_PARAM);
    }

    // Rotation
    auto yaw_rate = ego_angular_v(2);
    auto theta = yaw_rate * dt_s;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    R(0, 0) = static_cast<float>(cos(theta));
    R(0, 1) = static_cast<float>(-sin(theta));
    R(1, 0) = static_cast<float>(sin(theta));
    R(1, 1) = static_cast<float>(cos(theta));

    auto velocity = ego_v.segment<2>(0).norm();
    if (std::abs(yaw_rate) > 1e-5) {
        t(0) = velocity * R(1, 0) / yaw_rate;
        t(1) = velocity * (1 - R(1, 1)) / yaw_rate;
    } else {
        t(0) = ego_v(0) * dt_s;
        t(1) = ego_v(1) * dt_s;
    }

    Eigen::Vector3f state_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f state_v = Eigen::Vector3f::Zero();
    state_x.segment<2>(0) = state->segment<2>(0);
    state_v.segment<2>(0) = state->segment<2>(2);

    if (yaw_rate < 1e-5) {
        state_x(0) = state_x(0) - ego_v(0) * dt_s;
        state_x(1) = state_x(1) - ego_v(1) * dt_s;
    } else {
        state_x = R.inverse() * state_x - t;
        state_v = R.inverse() * state_v;
    }

    state->segment<2>(0) = state_x.segment<2>(0);
    state->segment<2>(2) = state_v.segment<2>(0);

    // Yaw theta Compensation
    if (state->rows() >= 7) {
        (*state)(6) += theta;
    }

    return AP_SUCCESS;
}

inline apStatus_t CheckValidHTransform(
    const std::vector<Point2f> &transformed_points, bool *is_valid) {
    if (nullptr == is_valid) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    if (transformed_points.size() != 4) {
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    // check clockwise
    *is_valid = true;
    if (transformed_points.at(0).x >= transformed_points.at(1).x ||
        transformed_points.at(1).y >= transformed_points.at(2).y ||
        transformed_points.at(2).x <= transformed_points.at(3).x ||
        transformed_points.at(3).y <= transformed_points.at(0).y) {
        *is_valid = false;
    }
    return AP_SUCCESS;
}

template <typename T>
Eigen::Matrix<T, 3, 1> R2ypr(const Eigen::Matrix<T, 3, 3> &R) {
    Eigen::Matrix<T, 3, 1> n = R.col(0);
    Eigen::Matrix<T, 3, 1> o = R.col(1);
    Eigen::Matrix<T, 3, 1> a = R.col(2);

    Eigen::Matrix<T, 3, 1> ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;
    return ypr;
}

template <typename T>
Eigen::Matrix<T, 3, 3> ypr2R(const Eigen::Matrix<T, 3, 1> &ypr) {
    T y = ypr(0);
    T p = ypr(1);
    T r = ypr(2);

    Eigen::Matrix<T, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<T, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<T, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> CalCuPseudoInverse(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat,
    const T &error) {
    Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>
        svd_holder(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> U = svd_holder.matrixU();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> V = svd_holder.matrixV();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> D =
        svd_holder.singularValues();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> S(V.cols(), U.cols());

    S.setZero();
    for (int i = 0; i < D.size(); ++i) {
        if (D(i, 0) > error) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result =
        V * S * U.transpose();
    return result;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> InverseSquareRoot(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat) {
    Eigen::SelfAdjointEigenSolver<
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>
        square_root(mat);
    return square_root.operatorInverseSqrt();
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> OmegaMatJPL(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 4, 4>() << 0, vec[2], -vec[1],
      vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2], -vec[0],
      -vec[1], -vec[2], 0)
      .finished();
}

inline Eigen::Vector3f TransformPointByHomography(const Eigen::Matrix3f &h,
                                           const Eigen::Vector3f &p) {
    Eigen::Vector3f trans_p = h * p;
    trans_p = trans_p / trans_p[2];
    return trans_p;
}
}  // namespace perception
}  // namespace senseAD
