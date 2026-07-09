/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <cmath>
#include <vector>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

template <typename T>
inline apStatus_t PreProcessMeasYaw(const T optim_yaw, T* meas_yaw) {
    if (nullptr == meas_yaw) {
        return AP_NULL_PTR;
    }
    T angle_diff = *meas_yaw - optim_yaw;
    const auto times = std::round(std::fabs(angle_diff / M_PI));
    if (angle_diff > M_PI / 2) {
        *meas_yaw -= times * M_PI;
    }
    if (angle_diff < -M_PI / 2) {
        *meas_yaw += times * M_PI;
    }
    return AP_SUCCESS;
}

// inline apStatus_t PostProcessMeasYaw(const float optim_yaw,
//                                      float* meas_yaw,
//                                      const float max_diff = M_PI / 6) {
//     if (nullptr == meas_yaw) {
//         return AP_NULL_PTR;
//     }
//     float angle_diff = *meas_yaw - optim_yaw;
//     if (angle_diff > max_diff || angle_diff < -max_diff) {
//         *meas_yaw = optim_yaw;
//     }
//     return AP_SUCCESS;
// }

template <typename T>
inline apStatus_t NormalizeAngle(T* angle) {
    if (nullptr == angle) {
        return AP_NULL_PTR;
    }
    const auto times = std::round(std::fabs(*angle / (2. * M_PI)));
    if (*angle > M_PI) {
        *angle -= times * 2.0 * M_PI;
    }
    if (*angle < -M_PI) {
        *angle += times * 2.0 * M_PI;
    }
    return AP_SUCCESS;
}

template <typename T>
inline apStatus_t SubstractAngle(T angle_l, T angle_r, T* res) {
    *res = 0;
    if (std::abs(angle_l) > M_PI || std::abs(angle_r) > M_PI) {
        return AP_INVALID_PARAM;
    }
    auto AntiNormalize = [=](T* angle) {
        if (*angle < 0) {
            *angle += 2.0 * M_PI;
        }
    };
    AntiNormalize(&angle_l);
    AntiNormalize(&angle_r);
    *res = angle_r - angle_l;
    NormalizeAngle(res);
    return AP_SUCCESS;
}

template <typename T>
inline apStatus_t CalcMeanScaleAngle(const std::vector<T>& angles,
                                     const std::vector<T>& scales,
                                     T* mean_angle) {
    if (nullptr == mean_angle) {
        AP_LERROR(CalcMeanScaleAngle) << "Input mean_angle is null pointer";
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    if (angles.empty()) {
        return AP_SUCCESS;
    }
    T sin_sum = 0, cos_sum = 0;
    for (size_t i = 0; i < angles.size(); i++) {
        sin_sum += scales[i] * std::sin(angles[i]);
        cos_sum += scales[i] * std::cos(angles[i]);
    }
    T angle = std::atan2(sin_sum, cos_sum);
    *mean_angle = angle;
    return AP_SUCCESS;
}

template <typename T>
inline apStatus_t ProcessToAngleDiffSmallPI(const T optim_yaw, T* meas_yaw) {
    if (nullptr == meas_yaw) {
        return AP_NULL_PTR;
    }
    T angle_diff = *meas_yaw - optim_yaw;
    const auto times = std::round(std::fabs(angle_diff / (2 * M_PI)));
    if (angle_diff > M_PI) {
        *meas_yaw -= times * 2.0 * M_PI;
    }
    if (angle_diff < -M_PI) {
        *meas_yaw += times * 2.0 * M_PI;
    }
    return AP_SUCCESS;
}

template <typename T>
inline apStatus_t ProcessToAngleDiffSmallPI(std::vector<T>* angles) {
    if (angles == nullptr || angles->empty()) return AP_SUCCESS;
    T angle = (*angles)[0];
    for (size_t i = 1; i< angles->size(); i++) {
        T process_angle = (*angles)[i];
        ProcessToAngleDiffSmallPI(angle, &process_angle);
        (*angles)[i] = process_angle;
    }
    return AP_SUCCESS;
}

inline float Rad2Deg(const float rad) { return rad * 180.0f / M_PI; }
inline float Deg2Rad(const float deg) { return deg * M_PI / 180.0f; }

}  // namespace perception
}  // namespace senseAD
