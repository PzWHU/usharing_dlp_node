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
#include <set>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

#include "perception_common/base/data_type/geometry.hpp"
#include "perception_common/math_helper/geometry/perspective_tf.hpp"

namespace senseAD {
namespace perception {

// iniline util functions
template <int n, int m>
apStatus_t UpdateLinearMeasureModel(const Eigen::Matrix<float, n, n>& R,
                                    const Eigen::Matrix<float, n, 1>& err,
                                    const std::vector<int>& mdims,
                                    const std::vector<int>& sdims,
                                    Eigen::Matrix<float, m, m>* cov,
                                    Eigen::Matrix<float, m, 1>* state,
                                    float* likelihood) {
    if (nullptr == cov || nullptr == state) {
        AP_LERROR(UpdateLinearPositionModel) << "input is nullptr.";
        AP_CHECK_ERROR(AP_INVALID_PARAM);
    }
    Eigen::Matrix<float, m, m> cov_in = *cov;
    if (mdims.size() != sdims.size()) {
        AP_LERROR(UpdateLinearPositionModel)
            << "measure dims and state dims must be same.";
        AP_CHECK_ERROR(AP_INVALID_PARAM);
    }
    std::set<uint8_t> dim_set;
    for (const uint8_t& xdim : mdims) {
        if (dim_set.count(xdim)) {
            AP_LERROR(UpdateLinearPositionModel)
                << "repeated dim, please using common kalman filter.";
            AP_CHECK_ERROR(AP_INTERNAL_ERROR);
        }
        dim_set.insert(xdim);
    }
    dim_set.clear();
    for (const uint8_t& xdim : sdims) {
        if (dim_set.count(xdim)) {
            AP_LERROR(UpdateLinearPositionModel)
                << "repeated dim, please using common kalman filter.";
            AP_CHECK_ERROR(AP_INTERNAL_ERROR);
        }
        dim_set.insert(xdim);
    }
    Eigen::Matrix<float, STATE_DIMS, n> cov_ht;
    for (size_t i = 0; i < mdims.size(); i++) {
        cov_ht.col(mdims[i]) = cov_in.col(sdims[i]);
    }
    Eigen::Matrix<float, n, n> S = Eigen::Matrix<float, n, n>::Zero();
    for (size_t i = 0; i < mdims.size(); i++) {
        for (size_t j = 0; j < mdims.size(); j++) {
            S(mdims[i], mdims[j]) = cov_in(sdims[i], sdims[j]);
        }
    }
    S = S + R;
    Eigen::Matrix<float, n, n> S_inv = S.inverse();
    float det = S.determinant();
    float r_det = R.determinant();
    if (r_det < 1e-18) {
        AP_LERROR(UpdateLinearPositionModel)
            << "Noise matrix determinant is zero.";
        AP_CHECK_ERROR(AP_INVALID_PARAM);
    }
    Eigen::Matrix<float, 1, n> err_trans = err.transpose();
    Eigen::Matrix<float, 1, 1> sim_mat = err_trans * S_inv * err;
    *likelihood =
        1.0 / sqrt(2 * M_PI * sqrt(fabs(det))) * exp(-0.5 * sim_mat(0, 0)) +
        1e-6;
    Eigen::Matrix<float, STATE_DIMS, n> K = cov_ht * S_inv;
    Eigen::Matrix<float, m, m> K_H = Eigen::Matrix<float, m, m>::Zero();
    for (size_t i = 0; i < mdims.size(); i++) {
        K_H.col(sdims[i]) = K.col(mdims[i]);
    }
    Eigen::Matrix<float, m, m> I = Eigen::Matrix<float, m, m>::Identity();
    *state += K * err;
    *cov = (I - K_H) * cov_in;
    return AP_SUCCESS;
}

}  // namespace perception
}  // namespace senseAD
