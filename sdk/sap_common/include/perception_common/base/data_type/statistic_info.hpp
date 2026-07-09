/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <eigen3/Eigen/Core>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {
struct StatisticInfo {
    uint64_t timestamp_ns;
    float weight;
    float count;
    Eigen::VectorXf mean;
    Eigen::MatrixXf covariance;
};

static const int DIVISOR = 1000;
static const int FREEDOM = 3;
static const float MAX_PVAlUE = 0.009;
static const float MIN_PVALUE = 0.001;

static const double pValues_1[] = {3.29052673149, 3.09023230617, 2.96773792534,
                                   2.8781617391,  2.80703376834, 2.74778138544,
                                   2.69684426088, 2.6520698079,  2.61205414123};
static const double pValues_2[] = {3.71692218885, 3.52550935282, 3.40856069047,
                                   3.32308920069, 3.25524726144, 3.1987484458,
                                   3.15018892447, 3.10751146009, 3.06937475771};
static const double pValues_3[] = {4.03314222366, 3.84649412512, 3.73248210518,
                                   3.6491654737,  3.5830373242,  3.52796698337,
                                   3.4806353931,  3.43903676234, 3.40186345334};
static const double pValues_4[] = {4.29730461486, 4.11384955921, 4.00179038868,
                                   3.91989939226, 3.8549006473,  3.80076869888,
                                   3.75424159674, 3.71334812761, 3.67680323018};
static const double* Chi2Table[] = {pValues_1, pValues_2, pValues_3, pValues_4};

inline apStatus_t GetChi2StatisticValue(float p_value,
                                        int freedom,
                                        float* statistic_value) {
    if (nullptr == statistic_value) {
        AP_LERROR(GetChi2StatisticValue) << "Input Ptr is null";
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    if (freedom > FREEDOM) {
        AP_LERROR(GetChi2StatisticValue) << "Query freemdom is larger, Check "
                                            "Chi2Table in statistic_info.hpp";
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    if (p_value < MIN_PVALUE || p_value > MAX_PVAlUE) {
        AP_LERROR(GetChi2StatisticValue)
            << "Query probability is larger than MAX_PVAlUE or lower than "
               "MIN_PVALUE, Check Chi2Table in statistic_info.hpp";
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    int col = static_cast<int>(DIVISOR * p_value) - 1;
    int row = freedom - 1;
    *statistic_value = Chi2Table[row][col];
    return AP_SUCCESS;
}

}  // namespace perception
}  // namespace senseAD
