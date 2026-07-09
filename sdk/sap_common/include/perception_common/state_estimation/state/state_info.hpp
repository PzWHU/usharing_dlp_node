/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <string>
#include <map>

#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

namespace senseAD {
namespace perception {
template <typename E>
constexpr int E2I(E e) {
    return static_cast<int>(e);
}
#define CP_STATE_DIMS 2
#define CV_STATE_DIMS 4
#define CA_STATE_DIMS 6
#define CT_STATE_DIMS 6
#define STATE_DIMS 12

typedef Eigen::Matrix<float, 2, 2> CPMatrixf;
typedef Eigen::Matrix<float, 4, 4> CVMatrixf;
typedef Eigen::Matrix<float, 6, 6> CAMatrixf;
typedef Eigen::Matrix<float, 6, 6> CTMatrixf;

typedef Eigen::Matrix<float, 2, 1> CPVectorf;
typedef Eigen::Matrix<float, 4, 1> CVVectorf;
typedef Eigen::Matrix<float, 6, 1> CAVectorf;
typedef Eigen::Matrix<float, 6, 1> CTVectorf;

typedef Eigen::Matrix<float, STATE_DIMS, 1> StateVectorf;
typedef Eigen::Matrix<float, STATE_DIMS, STATE_DIMS> StateMatrixf;

// const int STATE_DIMS_3D = 21;
enum class CPStateDimIdx {
    P_X = 0,
    P_Y = 1
};

// enum class ProcessModelType {
//     C_P = 0,
//     C_V = 1,
//     C_A = 2,
//     C_T = 3,
//     C_J = 4
// }

enum class CVStateDimIdx {
    P_X = 0,
    P_Y = 1,
    V_X = 2,
    V_Y = 3
};

enum class CAStateDimIdx {
    P_X = 0,
    P_Y = 1,
    V_X = 2,
    V_Y = 3,
    A_X = 4,
    A_Y = 5
};

enum class CTStateDimIdx {
    P_X = 0,
    P_Y = 1,
    V_X = 2,
    V_Y = 3,
    YAW = 4,
    YAW_RATE = 5
};

enum StateDimIdx {
    P_X = 0,
    P_Y = 1,
    V_X = 2,
    V_Y = 3,
    A_X = 4,
    A_Y = 5,
    YAW = 6,
    YAW_RATE = 7,
    X_BIAS = 8,
    Y_BIAS = 9,
    J_X = 10,
    J_Y = 11
};

enum class MeasDimIdx {
    P_X = 0,
    P_Y = 1,
    V_X = 2,
    V_Y = 3,
    A_X = 4,
    A_Y = 5,
    YAW = 6,
    YAW_RATE = 7,
    X_BIAS = 8,
    Y_BIAS = 9,
    J_X = 10,
    J_Y = 11,
    R_V = 12
};
struct EnumHash {
    template <typename T>
    std::size_t operator()(T t) const {
        return static_cast<std::size_t>(t);
    }
};
template <typename Key>
using EnumHashType = typename std::
    conditional<std::is_enum<Key>::value, EnumHash, std::hash<Key>>::type;

template <typename Key, typename T>
using MapOfEnumIdx = std::unordered_map<Key, T, EnumHashType<Key>>;
}  // namespace perception
}  // namespace senseAD
