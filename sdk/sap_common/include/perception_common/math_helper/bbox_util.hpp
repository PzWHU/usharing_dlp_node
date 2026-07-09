/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

#include "perception_common/log/common_log.hpp"
#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/bbox_2d.hpp"
#include "perception_common/base/data_type/geometry.hpp"

namespace senseAD {
namespace perception {

apStatus_t CalculateIoU(const Bbox2D& source_bbox,
                        const Bbox2D& target_bbox,
                        float* iou);

apStatus_t CalculateIoUIoSIoT(const Bbox2D& source_bbox,
                              const Bbox2D& target_bbox,
                              float* iou,
                              float* ios,
                              float* iot);

apStatus_t ClipByImageHW(const int image_height,
                         const int image_width,
                         Bbox2D* bbox);

apStatus_t ClipByImageRange(const int image_start_x,
                            const int image_start_y,
                            const int image_width,
                            const int image_height,
                            Bbox2D* bbox);

apStatus_t FlipBbox(const uint16_t& image_width, Bbox2D* bbox2d);

apStatus_t Bbox2DToVerticals(const Bbox2D& bbox,
                             std::vector<Point2f>* verticals);

apStatus_t Verticals2Bbox2D(const std::vector<Point2f>& verticals,
                            Bbox2D* bbox);

apStatus_t CalculateCenterDistance(const Bbox2D& src_bbox,
                                   const Bbox2D& dst_bbox,
                                   float* distance);

apStatus_t CalculateCornerDistance(const Bbox2D& src_bbox,
                                   const Bbox2D& dst_bbox,
                                   float* distance);

}  // namespace perception
}  // namespace senseAD
