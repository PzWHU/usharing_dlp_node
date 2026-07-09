/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#include <algorithm>

#include "perception_common/log/error.hpp"
#ifdef WITH_SERIALIZATION_INTERFACE
#include "perception_common/base/serialization/serialization_helper.hpp"
#endif
namespace senseAD {
namespace perception {

class Bbox2D {
 public:
    Bbox2D()
        : initialized_(false),
          top_left_x_(0),
          top_left_y_(0),
          bottom_right_x_(0),
          bottom_right_y_(0),
          confidence_(-1),
          is_left_side_truncated_(false),
          is_right_side_truncated_(false),
          is_bottom_side_truncated_(false),
          is_top_side_truncated_(false) {}

    Bbox2D(const float x1, const float y1, const float x2, const float y2)
        : initialized_(true),
          top_left_x_(x1),
          top_left_y_(y1),
          bottom_right_x_(x2),
          bottom_right_y_(y2),
          confidence_(-1),
          is_left_side_truncated_(false),
          is_right_side_truncated_(false),
          is_bottom_side_truncated_(false),
          is_top_side_truncated_(false) {}

    Bbox2D(const float x1,
           const float y1,
           const float x2,
           const float y2,
           const float conf,
           const bool is_left_side_truncated,
           const bool is_right_side_truncated,
           const bool is_bottom_side_truncated,
           const bool is_top_side_truncated)
        : initialized_(true),
          top_left_x_(x1),
          top_left_y_(y1),
          bottom_right_x_(x2),
          bottom_right_y_(y2),
          confidence_(conf),
          is_left_side_truncated_(is_left_side_truncated),
          is_right_side_truncated_(is_right_side_truncated),
          is_bottom_side_truncated_(is_bottom_side_truncated),
          is_top_side_truncated_(is_top_side_truncated) {}

    Bbox2D(const float x1,
           const float y1,
           const float x2,
           const float y2,
           const float conf)
        : initialized_(true),
          top_left_x_(x1),
          top_left_y_(y1),
          bottom_right_x_(x2),
          bottom_right_y_(y2),
          confidence_(conf),
          is_left_side_truncated_(false),
          is_right_side_truncated_(false),
          is_bottom_side_truncated_(false),
          is_top_side_truncated_(false) {}

    Bbox2D(const Bbox2D& bbox)
        : initialized_(bbox.Initialized()),
          top_left_x_(bbox.TopLeftX()),
          top_left_y_(bbox.TopLeftY()),
          bottom_right_x_(bbox.BottomRightX()),
          bottom_right_y_(bbox.BottomRightY()),
          confidence_(bbox.Confidence()),
          is_left_side_truncated_(bbox.IsLeftSideTruncated()),
          is_right_side_truncated_(bbox.IsRightSideTruncated()),
          is_bottom_side_truncated_(bbox.IsBottomSideTruncated()),
          is_top_side_truncated_(bbox.IsTopSideTruncated()) {}

    bool Initialized() const { return initialized_; }
    float TopLeftX() const { return top_left_x_; }
    float TopLeftY() const { return top_left_y_; }
    float BottomRightX() const { return bottom_right_x_; }
    float BottomRightY() const { return bottom_right_y_; }
    float Confidence() const { return confidence_; }
    bool IsLeftSideTruncated() const { return is_left_side_truncated_; }
    bool IsRightSideTruncated() const { return is_right_side_truncated_; }
    bool IsBottomSideTruncated() const { return is_bottom_side_truncated_; }
    bool IsTopSideTruncated() const { return is_top_side_truncated_; }
    void SetTopLeftX(const float x1) {
        initialized_ = true;
        top_left_x_ = x1;
    }
    void SetTopLeftY(const float y1) {
        initialized_ = true;
        top_left_y_ = y1;
    }
    void SetBottomRightX(const float x2) {
        initialized_ = true;
        bottom_right_x_ = x2;
    }
    void SetBottomRightY(const float y2) {
        initialized_ = true;
        bottom_right_y_ = y2;
    }
    void SetConfidence(const float confidence) {
        initialized_ = true;
        confidence_ = confidence;
    }
    void SetLeftSideTruncated(const float is_left_side_truncated) {
        initialized_ = true;
        is_left_side_truncated_ = is_left_side_truncated;
    }
    void SetRightSideTruncated(const float is_right_side_truncated) {
        initialized_ = true;
        is_right_side_truncated_ = is_right_side_truncated;
    }
    void SetBottomSideTruncated(const float is_bottom_side_truncated) {
        initialized_ = true;
        is_bottom_side_truncated_ = is_bottom_side_truncated;
    }
    void SetTopSideTruncated(const bool is_top_side_truncated) {
        initialized_ = true;
        is_top_side_truncated_ = is_top_side_truncated;
    }
    float CenterX() const { return (top_left_x_ + bottom_right_x_) / 2.; }
    float CenterY() const { return (top_left_y_ + bottom_right_y_) / 2.; }
    float Width() const { return bottom_right_x_ - top_left_x_ + 1; }
    float Height() const { return bottom_right_y_ - top_left_y_ + 1; }
    float BottomCenterX() const { return (top_left_x_ + bottom_right_x_) / 2.; }
    float BottomCenterY() const { return bottom_right_y_; }
    float WidthHeightRatio() const {
        const float height = bottom_right_y_ - top_left_y_ + 1;
        if (fabs(height) < __FLT_EPSILON__) {
            return 0;
        }
        return (bottom_right_x_ - top_left_x_ + 1) / height;
    }

    float Scale() const {
        const float height = bottom_right_y_ - top_left_y_ + 1;
        if (fabs(height) < __FLT_EPSILON__) {
            return 0;
        }
        const float width = bottom_right_x_ - top_left_x_ + 1;
        if (fabs(width) < __FLT_EPSILON__) {
            return 0;
        }
        return height * width;
    }

    apStatus_t CheckValue(bool* is_any_nan) {
        if (nullptr == is_any_nan) {
            return AP_NULL_PTR;
        }
        *is_any_nan = false;
        if (std::isnan(top_left_x_) || std::isinf(top_left_x_)) {
            *is_any_nan = true;
            return AP_SUCCESS;
        }
        if (std::isnan(top_left_y_) || std::isinf(top_left_y_)) {
            *is_any_nan = true;
            return AP_SUCCESS;
        }
        if (std::isnan(bottom_right_x_) || std::isinf(bottom_right_x_)) {
            *is_any_nan = true;
            return AP_SUCCESS;
        }
        if (std::isnan(bottom_right_y_) || std::isinf(bottom_right_y_)) {
            *is_any_nan = true;
            return AP_SUCCESS;
        }
        return AP_SUCCESS;
    }

    bool initialized_;
    float top_left_x_;
    float top_left_y_;
    float bottom_right_x_;
    float bottom_right_y_;
    float confidence_;
    bool is_left_side_truncated_;
    bool is_right_side_truncated_;
    bool is_bottom_side_truncated_;
    bool is_top_side_truncated_;

#ifdef WITH_SERIALIZATION_INTERFACE

 private:
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {  // NOLINT
        ar& initialized_;
        ar& top_left_x_;
        ar& top_left_y_;
        ar& bottom_right_x_;
        ar& bottom_right_y_;
        ar& confidence_;
        ar& is_left_side_truncated_;
        ar& is_right_side_truncated_;
        ar& is_bottom_side_truncated_;
        ar& is_top_side_truncated_;
    }
#endif
};  // class Bbox2D

}  // namespace perception
}  // namespace senseAD
