/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * KONG Zhenqiang <kongzhenqiang@sensetime.com>
 * Cai Xinyuan <caixinyuan@sensetime.com>
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

/**
 * TODO(chenshengjie): common doesn't depend on pcl, but those where
 *   #include "common/data_type/point_cloud.hpp"
 */
#include <pcl/point_types.h>
#include <opencv2/core.hpp>

#include "perception_common/base/data_type/base.hpp"

namespace senseAD {
namespace perception {

struct LidarPacket {
    uint64_t time_ns;
    uint8_t *data;
    uint32_t size;
};

struct LidarPointXYZIR {
    PCL_ADD_POINT4D;
    float32_t intensity;
    union {
        uint16_t ring;
        struct {
            uint8_t ring_id;
            uint8_t lidar_id;
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// rows * cols = points number * 3(XYZ channels)
typedef cv::Mat_<float> PointCloud;
// rows * cols = points number * 1(RingID)
typedef cv::Mat_<int> PointCloudRingID;
// xyzi
typedef cv::Mat_<float> PointCloudXYZI;
}  // namespace perception
}  // namespace senseAD

POINT_CLOUD_REGISTER_POINT_STRUCT(
    senseAD::perception::LidarPointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float32_t,
                                            intensity,
                                            intensity)(uint16_t, ring, ring))
