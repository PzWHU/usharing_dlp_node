/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Cholesky"
#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"

namespace senseAD {
namespace perception {

class UnscentedKalmanFilter : public KalmanFilterBase {
 public:
    UnscentedKalmanFilter();
    explicit UnscentedKalmanFilter(const UnscentedKalmanFilter& filter);
    virtual ~UnscentedKalmanFilter() = default;

    apStatus_t Predict(const KFStatePtr& state_in,
                       const KFStatePtr& state_out,
                       const double delta_t,
                       const BaseProcessModelPtr& process_model);

    apStatus_t Update(const KFStatePtr& state_in,
                      const MeasureModelBasePtr& measure_model);

 private:
    apStatus_t GenerateSigmaPoints(const Eigen::MatrixXf& aug_cov_mat,
                                   const Eigen::VectorXf& aug_state_vec,
                                   Eigen::MatrixXf* sigma_points_mat);
    apStatus_t GenerateWeightVector(const int& noise_dim,
                                    Eigen::VectorXf* weight_vec);
    apStatus_t ComputePriorMatrixAndState(
        const Eigen::MatrixXf& sigma_points_mat,
        const Eigen::VectorXf& weight_vec,
        Eigen::MatrixXf* prior_P,
        Eigen::VectorXf* prior_state_vec);
    apStatus_t ComputeCovOfStateAndMeas(
        const Eigen::MatrixXf& state_sigma_points,
        const Eigen::MatrixXf& meas_sigma_points,
        const Eigen::VectorXf& weight_vec,
        Eigen::MatrixXf* P_yy,
        Eigen::MatrixXf* P_xy,
        Eigen::VectorXf* u_y);

 private:
    const float ukf_lamda_param = 7;
    const float gussian_factor_;
};
REGISTER_KALMANFILTER(UnscentedKalmanFilter);

}  // namespace perception
}  // namespace senseAD
