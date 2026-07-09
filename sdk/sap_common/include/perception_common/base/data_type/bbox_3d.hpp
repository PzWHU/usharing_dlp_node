/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Li Xiao <lixiao@sensetime.com>
 */
#pragma once

#include <algorithm>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

class Bbox3D {
 public:
    Bbox3D()
        : initialized_(false),
          top_front_left_x_(0),
          top_front_left_y_(0),
          top_front_right_x_(0),
          top_front_right_y_(0),
          top_back_left_x_(0),
          top_back_left_y_(0),
          top_back_right_x_(0),
          top_back_right_y_(0),
          bottom_front_left_x_(0),
          bottom_front_left_y_(0),
          bottom_front_right_x_(0),
          bottom_front_right_y_(0),
          bottom_back_left_x_(0),
          bottom_back_left_y_(0),
          bottom_back_right_x_(0),
          bottom_back_right_y_(0),
          confidence_(-1) {}

    // explicit Bbox3D(const float conf) : initialized_(true), confidence_(conf)
    // {}

    Bbox3D(const Bbox3D& bbox)
        : initialized_(bbox.Initialized()),
          top_front_left_x_(bbox.TopFrontLeftX()),
          top_front_left_y_(bbox.TopFrontLeftY()),
          top_front_right_x_(bbox.TopFrontRightX()),
          top_front_right_y_(bbox.TopFrontRightY()),
          top_back_left_x_(bbox.TopBackLeftX()),
          top_back_left_y_(bbox.TopBackLeftY()),
          top_back_right_x_(bbox.TopBackRightX()),
          top_back_right_y_(bbox.TopBackRightY()),
          bottom_front_left_x_(bbox.BottomFrontLeftX()),
          bottom_front_left_y_(bbox.BottomFrontLeftY()),
          bottom_front_right_x_(bbox.BottomFrontRightX()),
          bottom_front_right_y_(bbox.BottomFrontRightY()),
          bottom_back_left_x_(bbox.BottomBackLeftX()),
          bottom_back_left_y_(bbox.BottomBackLeftY()),
          bottom_back_right_x_(bbox.BottomBackRightX()),
          bottom_back_right_y_(bbox.BottomBackRightY()),
          confidence_(bbox.Confidence()) {}

    bool Initialized() const { return initialized_; }
    float TopFrontLeftX() const { return top_front_left_x_; }
    float TopFrontLeftY() const { return top_front_left_y_; }
    float TopFrontRightX() const { return top_front_right_x_; }
    float TopFrontRightY() const { return top_front_right_y_; }
    float TopBackLeftX() const { return top_back_left_x_; }
    float TopBackLeftY() const { return top_back_left_y_; }
    float TopBackRightX() const { return top_back_right_x_; }
    float TopBackRightY() const { return top_back_right_y_; }
    float BottomFrontLeftX() const { return bottom_front_left_x_; }
    float BottomFrontLeftY() const { return bottom_front_left_y_; }
    float BottomFrontRightX() const { return bottom_front_right_x_; }
    float BottomFrontRightY() const { return bottom_front_right_y_; }
    float BottomBackLeftX() const { return bottom_back_left_x_; }
    float BottomBackLeftY() const { return bottom_back_left_y_; }
    float BottomBackRightX() const { return bottom_back_right_x_; }
    float BottomBackRightY() const { return bottom_back_right_y_; }
    float Confidence() const { return confidence_; }
    void SetTopFrontLeftX(const float x) {
        initialized_ = true;
        top_front_left_x_ = x;
    }
    void SetTopFrontLeftY(const float y) {
        initialized_ = true;
        top_front_left_y_ = y;
    }
    void SetTopFrontRightX(const float x) {
        initialized_ = true;
        top_front_right_x_ = x;
    }
    void SetTopFrontRightY(const float y) {
        initialized_ = true;
        top_front_right_y_ = y;
    }
    void SetTopBackLeftX(const float x) {
        initialized_ = true;
        top_back_left_x_ = x;
    }
    void SetTopBackLeftY(const float y) {
        initialized_ = true;
        top_back_left_y_ = y;
    }
    void SetTopBackRightX(const float x) {
        initialized_ = true;
        top_back_right_x_ = x;
    }
    void SetTopBackRightY(const float y) {
        initialized_ = true;
        top_back_right_y_ = y;
    }
    void SetBottomFrontLeftX(const float x) {
        initialized_ = true;
        bottom_front_left_x_ = x;
    }
    void SetBottomFrontLeftY(const float y) {
        initialized_ = true;
        bottom_front_left_y_ = y;
    }
    void SetBottomFrontRightX(const float x) {
        initialized_ = true;
        bottom_front_right_x_ = x;
    }
    void SetBottomFrontRightY(const float y) {
        initialized_ = true;
        bottom_front_right_y_ = y;
    }
    void SetBottomBackLeftX(const float x) {
        initialized_ = true;
        bottom_back_left_x_ = x;
    }
    void SetBottomBackLeftY(const float y) {
        initialized_ = true;
        bottom_back_left_y_ = y;
    }
    void SetBottomBackRightX(const float x) {
        initialized_ = true;
        bottom_back_right_x_ = x;
    }
    void SetBottomBackRightY(const float y) {
        initialized_ = true;
        bottom_back_right_y_ = y;
    }

    void SetConfidence(const float confidence) {
        initialized_ = true;
        confidence_ = confidence;
    }

 private:
    bool initialized_;
    float top_front_left_x_;
    float top_front_left_y_;
    float top_front_right_x_;
    float top_front_right_y_;
    float top_back_left_x_;
    float top_back_left_y_;
    float top_back_right_x_;
    float top_back_right_y_;
    float bottom_front_left_x_;
    float bottom_front_left_y_;
    float bottom_front_right_x_;
    float bottom_front_right_y_;
    float bottom_back_left_x_;
    float bottom_back_left_y_;
    float bottom_back_right_x_;
    float bottom_back_right_y_;
    float confidence_;
};

}  // namespace perception
}  // namespace senseAD
