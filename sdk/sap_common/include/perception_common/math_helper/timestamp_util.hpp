/*
 * Copyright (C) 2020-2022 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@senseauto.com>
 */

#pragma once

#include <vector>
#include <utility>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

// @brief ignore tiny diff
inline uint64_t GetTimeStampUS(const uint64_t timestamp_ns) {
    return timestamp_ns / 1e3;
}

// @brief ignore small diff
inline uint64_t GetTimeStampMS(const uint64_t timestamp_ns) {
    return timestamp_ns / 1e6;
}

inline double GetTimeStampS(const uint64_t timestamp_ns) {
    return static_cast<double>(timestamp_ns) * 1e-6;
}

inline uint64_t GetTimeStampFromSToNS(const float timestamp_s) {
    return static_cast<uint64_t>(timestamp_s * 1e9);
}

inline apStatus_t CalculateTimeDurationS(const uint64_t start_time_ns,
                                         const uint64_t end_time_ns,
                                         double* time_duration_s) {
    if (nullptr == time_duration_s) {
        AP_LERROR(CalculateTimeDurationS) << "input time_duration_s nullptr";
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    if (end_time_ns < start_time_ns) {
        AP_LERROR(CalculateTimeDurationS)
            << "end time ns -> " << end_time_ns
            << " should be larger than start time -> " << start_time_ns;
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    *time_duration_s = static_cast<double>(end_time_ns - start_time_ns) / 1e9;
    return AP_SUCCESS;
}

inline uint64_t CalculateAbsGapTimestampNS(const uint64_t src_timestamp_ns,
                                           const uint64_t dst_timestamp_ns) {
    if (src_timestamp_ns < dst_timestamp_ns) {
        return dst_timestamp_ns - src_timestamp_ns;
    } else {
        return src_timestamp_ns - dst_timestamp_ns;
    }
}

}  // namespace perception
}  // namespace senseAD
