/*
 * Copyright (C) 2017~2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shu <zhangshu@sensetime.com>
 */

#pragma once

#include <vector>
#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

template <typename T>
apStatus_t polyfit(const std::vector<T>& oX,
                   const std::vector<T>& oY,
                   int nDegree,
                   std::vector<T>* result);

template <typename T>
apStatus_t polyfit_status(const std::vector<T>& oX,
                          const std::vector<T>& oY,
                          int nDegree,
                          std::vector<T>* result,
                          bool* result_flag);

template <typename T>
apStatus_t polyfit(const std::vector<T>& oX,
                   const std::vector<T>& oY,
                   const std::vector<T>& weight,
                   int nDegree,
                   std::vector<T>* result);

template <typename T>
std::vector<T> polyval(const std::vector<T>& oCoeff, const std::vector<T>& oX);

template <typename T>
T polyval(const std::vector<T>& oCoeff, const T oX);

}  // namespace perception
}  // namespace senseAD
