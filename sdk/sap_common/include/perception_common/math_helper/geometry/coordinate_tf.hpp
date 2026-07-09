/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * NI Tianwei <nitianwei@sensetime.com>
 * YUE Dayu <yuedayu@sensetime.com>
 * Dai Xing <daixing@sensetime.com>
 * Zhou Siyu <zhousiyu@sensetime.com>
 * Alex Kong <kongzhenqiang@sensetime.com>
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "perception_common/base/data_type/base.hpp"

namespace senseAD {
namespace perception {

inline void warp_r2a(float32_t x,
                     float32_t y,
                     float32_t base_x,
                     float32_t base_y,
                     float32_t heading,
                     float32_t *r_x,
                     float32_t *r_y) {
    float32_t r_heading = heading;
    float32_t cos_val = cos(r_heading);
    float32_t sin_val = sin(r_heading);

    *r_x = x * cos_val - y * sin_val + base_x;
    *r_y = y * cos_val + x * sin_val + base_y;
}

inline void warp_a2r(float32_t x,
                     float32_t y,
                     float32_t base_x,
                     float32_t base_y,
                     float32_t heading,
                     float32_t *r_x,
                     float32_t *r_y) {
    float32_t r_heading = heading;
    float32_t cos_val = cos(r_heading);
    float32_t sin_val = sin(r_heading);
    x -= base_x;
    y -= base_y;

    *r_x = x * cos_val + y * sin_val;
    *r_y = y * cos_val - x * sin_val;
}

inline cv::Point2f rotate(const cv::Point2f &center_pt,
                          const float32_t &radian,
                          const cv::Point2f ori_pt) {
    cv::Point2f result_pt;
    float32_t cos_val = cos(radian);
    float32_t sin_val = sin(radian);

    float32_t x = ori_pt.x - center_pt.x;
    float32_t y = ori_pt.y - center_pt.y;

    result_pt.x = x * cos_val - y * sin_val + center_pt.x;
    result_pt.y = y * cos_val + x * sin_val + center_pt.y;
    return result_pt;
}

inline cv::Point2f utm2car(const cv::Point2f &ori_pt,
                           const float32_t &heading_radian,
                           const float32_t &car_heading_radian) {
    float32_t rotate_radian = car_heading_radian - heading_radian;
    cv::Point2f center_pt(0, 0);
    return rotate(center_pt, rotate_radian, ori_pt);
}

inline cv::Point2f TransformPointCoordbyRTMatrix(const cv::Mat &rotate,
                                                 const cv::Mat &transform,
                                                 const cv::Point2f point) {
    cv::Mat p(2, 1, CV_32FC1);
    p.at<float>(0, 0) = point.x;
    p.at<float>(1, 0) = point.y;
    cv::Mat out = rotate * p + transform;

    cv::Point2f transform_point(0, 0);
    transform_point.x = out.at<float>(0, 0);
    transform_point.y = out.at<float>(1, 0);
    return transform_point;
}

}  // namespace perception
}  // namespace senseAD
