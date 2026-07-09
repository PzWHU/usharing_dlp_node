/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhangshiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"

#include <opencv2/opencv.hpp>

namespace senseAD {
namespace perception {

typedef struct StructFreespaceInfo {
    uint64_t timestamp_ns = 0;
    cv::Mat rot;
    cv::Mat trans;
    std::vector<std::vector<cv::Point2f>> freespace_contour;
    std::vector<cv::Vec4i> freespace_hierarchy;
} FreespaceInfo;

}  // namespace perception
}  // namespace senseAD
