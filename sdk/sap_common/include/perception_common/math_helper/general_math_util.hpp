/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * NI Tianwei <nitianwei@sensetime.com>
 * YUE Dayu <yuedayu@sensetime.com>
 * Dai Xing <daixing@sensetime.com>
 * Zhou Siyu <zhousiyu@sensetime.com>
 */

#pragma once

#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include "perception_common/base/data_type/base.hpp"

namespace senseAD {
namespace perception {

inline double RegularAngleRangeFromNegativePI2PositivePI(double yaw) {
    static constexpr double k2PI = 2 * M_PI;
    while (1) {
        if (yaw < M_PI && yaw >= -M_PI) {
            return yaw;
        } else if (yaw >= M_PI && yaw <= k2PI) {
            yaw = yaw - k2PI;
        } else if (yaw > k2PI) {
            yaw = yaw - k2PI;
        } else if (yaw < -M_PI && yaw >= -k2PI) {
            yaw = yaw + k2PI;
        } else if (yaw < -k2PI) {
            yaw = yaw + k2PI;
        }
    }
}

inline float32_t Cosine2dvect(const cv::Point2f& a, const cv::Point2f& b) {
    float32_t norm_a, norm_b;
    norm_a = std::sqrt(a.x * a.x + a.y * a.y);
    norm_b = std::sqrt(b.x * b.x + b.y * b.y);
    if ((norm_a * norm_b) == 0) {
        return NAN;
    }
    float32_t&& cosine = (a.x * b.x + a.y * b.y) / (norm_a * norm_b);
    return std::move(cosine);
}

inline float32_t Radian2dvect(const cv::Point2f& base_vect,
                              const cv::Point2f& new_vect) {
    float32_t cos_val = Cosine2dvect(base_vect, new_vect);
    float32_t radian = acos(cos_val);
    if (base_vect.x != 0) {
        float32_t tmp_y = base_vect.y / base_vect.x * new_vect.x;
        if (base_vect.x > 0 && new_vect.y < tmp_y) {
            radian *= -1;
        } else if (base_vect.x < 0 && new_vect.y > tmp_y) {
            radian *= -1;
        }
    } else {
        if (new_vect.x < 0) radian *= -1;
    }
    return radian;
}

inline cv::Point2f Normalize2dvect(const cv::Point2f& a) {
    float32_t norm_ = a.x * a.x + a.y * a.y;
    norm_ = std::sqrt(norm_);
    if (norm_ == 0) {
        return a;
    }
    cv::Point2f res;
    res.x = a.x / norm_;
    res.y = a.y / norm_;
    return res;
}

inline float32_t CalcNorm(const cv::Point2f& pts) {
    float32_t norm;
    norm = pts.x * pts.x + pts.y * pts.y;
    norm = sqrtf(norm);
    return norm;
}

inline float32_t CalcCrossValue(const cv::Point2f& from,
                                const cv::Point2f& to) {
    auto&& cross_val = from.x * to.y - from.y * to.x;
    return std::move(cross_val);
}

inline float32_t CalcDotValue(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    auto&& dot_val = pt1.x * pt2.x + pt1.y * pt2.y;
    return std::move(dot_val);
}

inline float32_t CalcAntiClockWiseRad(const cv::Point2f& from,
                                      const cv::Point2f& to) {
    auto&& cross_val = CalcCrossValue(from, to);
    auto&& dot_val = CalcDotValue(from, to);
    if (dot_val == 0.f) {
        return NAN;
    }
    auto&& theta = std::atan2(cross_val, dot_val);
    return std::move(theta);
}

inline float32_t SafeCast2dVect2CosineDeg(const cv::Point2f& from,
                                          const cv::Point2f& to) {
    auto&& rad = CalcAntiClockWiseRad(from, to);
    return std::move(rad / M_PI * 180.f);
}

inline cv::Point2f GetVerticalPoint(const cv::Point2f& pt0,
                                    const cv::Point2f& pt1,
                                    const cv::Point2f& pt2) {
    cv::Point2f target;
    float32_t&& A = pt1.y - pt0.y;
    float32_t&& B = pt0.x - pt1.x;
    float32_t&& C = pt1.x * pt0.y - pt0.x * pt1.y;
    if (A == 0 || B == 0) {
        return std::move(target);
    }
    target.x = (B * B * pt2.x - A * B * pt2.y - A * C) / (A * A + B * B);
    target.y = (-A * B * pt2.x + A * A * pt2.y - B * C) / (A * A + B * B);
    return std::move(target);
}

template <typename T>
static inline void Rect2Polar(T x, T y, T* r, T* theta) {
    if (r != nullptr) *r = std::sqrt(x * x + y * y);
    if (theta != nullptr) *theta = std::atan2(y, x);
}

// Used to check if a point is inside a polygon
template <typename PointType>
inline bool IsInsidePolygon(int32_t num_vertex,
                            const std::vector<PointType>& pts,
                            const PointType& pt) {
    int32_t i = 0;
    int32_t j = 0;
    bool c = false;
    for (i = 0, j = num_vertex - 1;
         i < num_vertex && i < static_cast<int32_t>(pts.size()); j = i++) {
        if (((pts[i].y > pt.y) != (pts[j].y > pt.y)) &&
            (pt.x <
             (pts[j].x - pts[i].x) * (pt.y - pts[i].y) / (pts[j].y - pts[i].y) +
                 pts[i].x))
            c = !c;
    }
    return c;
}

template <typename PointType>
inline bool IsInPolygon(const PointType& pt,
                        const std::vector<PointType>& polygon) {
    bool BResult = false;
    const float EPS = 1e-6;
    for (int i = 0; i < polygon.size(); ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[(i + 1) % polygon.size()];
        if (fabs(p1.y - p2.y) < EPS) {
            if (fabs(pt.y - p1.y) < EPS && (pt.x - p1.x) * (pt.x - p2.x) < 0.) {
                return false;
            }
            continue;
        }

        bool b1 = (p1.y <= pt.y);
        bool b2 = (pt.y < p2.y);
        bool b3 = b1 && b2;
        bool b4 = (!b1) && (!b2);
        if (b3 || b4) {
            float64_t x_k = (p1.x - p2.x) / (p1.y - p2.y);
            float64_t dTmp = x_k * (pt.y - p2.y) + p2.x;
            if (fabs(dTmp - pt.x) < EPS) {
                return false;
            }

            if (pt.x < dTmp) {
                BResult = !BResult;
            }
        }
    }
    return BResult;
}

}  // namespace perception
}  // namespace senseAD
