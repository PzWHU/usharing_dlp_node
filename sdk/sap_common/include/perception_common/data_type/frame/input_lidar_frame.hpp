/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"

#include <pcl/point_types.h>    // NOLINT
#include <pcl/point_cloud.h>    // NOLINT
#include "perception_common/base/data_type/point_cloud.hpp"     // NOLINT

namespace senseAD {
namespace perception {

typedef struct StructInputLidarFrame {
    uint64_t timestamp_ns = 0;

    using PointT = senseAD::perception::LidarPointXYZIR;
    using PointCloudT = pcl::PointCloud<PointT>;
    PointCloudT cloud;

    /**
     * TODO(someone):
     *  1. Add Pose info of cloud? also Pose CovMatrix
     *  2. Then dependency of localziation can be removed
     */

    void Reset() {
        cloud.clear();

        timestamp_ns = 0;
    }
} InputLidarFrame;  // struct InputLidarFrame

}  // namespace perception
}  // namespace senseAD
