/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 * Xu Jingwei <xujingwei@senseauto.com>
 */

#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/bbox_2d.hpp"
#include "perception_common/base/data_type/geometry.hpp"

namespace senseAD {
namespace perception {

apStatus_t CalculateBboxCenterSimilarity(const Bbox2D& source_bbox,
                                         const Bbox2D& target_bbox,
                                         float* similarity);

apStatus_t CalculateShapeSimilarity(const Bbox2D& source_bbox,
                                    const Bbox2D& target_bbox,
                                    float* similarity);

apStatus_t CalculateMotionSimilarity(const Bbox2D& source_bbox,
                                     const Bbox2D& target_bbox,
                                     float* similarity);

apStatus_t CalculateXYMotionSimilarity(const Bbox2D& source_bbox,
                                       const Bbox2D& target_bbox,
                                       float* similarity);

apStatus_t CalculateBottomCenterMotionSimilarity(const Bbox2D& source_bbox,
                                                 const Bbox2D& target_bbox,
                                                 float* similarity);

apStatus_t CalculateLocationDistance(const Eigen::Vector3f& src_location,
                                     const Eigen::Vector3f& dst_location,
                                     float* distance);

apStatus_t CalculateWeightedLocationDistance(
    const Eigen::Vector3f& src_location,
    const Eigen::Vector3f& dst_location,
    const float x_weight,
    const float y_weight,
    float* distance);

apStatus_t CalculateLocationDistanceExpSimilarity(
    const Eigen::Vector3f& src_location,
    const Eigen::Vector3f& dst_location,
    float* distance_similarity,
    float scale_fac = 1);

apStatus_t CalculateWeightedLocationDistanceSimilarity(
    const Eigen::Vector3f& src_location,
    const Eigen::Vector3f& dst_location,
    const float x_weight,
    const float y_weight,
    const float scale,
    const float distance_threshold,
    float* distance_similarity);

apStatus_t CalculateWeightedVelocityDistanceSimilarity(
    const Eigen::Vector3f& src_velocity,
    const Eigen::Vector3f& dst_velocity,
    const float x_weight,
    const float y_weight,
    const float scale,
    const float velocity_threshold,
    float* distance_similarity);

apStatus_t CalculateDirectedLocationDistanceSimilarity(
    const Eigen::Vector3f& src_location,
    const Eigen::Vector3f& dst_location,
    const float x_weight,
    const float y_weight,
    const float scale,
    float* distance_similarity);

inline const Eigen::Vector3f& AngleToVector(const float angle) {
    Eigen::Vector3f vector;
    vector(0) = std::cos(angle);
    vector(1) = std::sin(angle);
    vector(2) = 0.0f;
    return std::move(vector);
}

// @brief cos similarity
apStatus_t CalculateDirectionSimilarity(const Eigen::Vector3f& src_yaw_vector,
                                        const Eigen::Vector3f& dst_yaw_vector,
                                        float* similarity);

apStatus_t CalculateDirectionSimilarity(const float src_yaw_angle,
                                        const float dst_yaw_angle,
                                        float* similarity);

// @brief cos(Azimuth) exp similarity
apStatus_t CalculateAzimuthExpSimilarity(const Eigen::Vector3f& src_location,
                                         const Eigen::Vector3f& dst_location,
                                         const float& scale,
                                         const float& direction_delta_threshold,
                                         float* similarity);

apStatus_t CalculateAppearanceSimilarity(const Eigen::VectorXf& object_feature,
                                         const Eigen::VectorXf& tracker_feature,
                                         float* similarity);

apStatus_t CalculateAppearanceSimilarity(
    const Eigen::VectorXf& object_feature,
    const std::vector<Eigen::VectorXf>& tracker_features,
    float* similarity);

apStatus_t CalculateLaneDistance(
    const std::vector<Point2f>& cur_lane_representations,
    const std::vector<Point2f>& trk_lane_representations,
    float* distance);

apStatus_t CalculateLaneWeightDistance(
    const std::vector<Point2f>& cur_lane_representations,
    const std::vector<Point2f>& trk_lane_representations,
    const float start_range,
    const float scale,
    float* distance);

apStatus_t CalculateMahalanobisDistance(const Eigen::VectorXf& src,
                                        const Eigen::VectorXf& dst,
                                        const Eigen::MatrixXf& covar,
                                        float* similarity);

apStatus_t CalculateBhattacharyyaDistance(const Eigen::VectorXf& src,
                                          const Eigen::VectorXf& dst,
                                          const Eigen::MatrixXf& src_cov,
                                          const Eigen::MatrixXf& dst_cov,
                                          float* distance);

apStatus_t Calculate3DLocDistance(const Eigen::VectorXf& src,
                                  const Eigen::VectorXf& dst,
                                  float* distance);

apStatus_t Calculate3DSizeDistance(const Eigen::VectorXf& src,
                                   const Eigen::VectorXf& dst,
                                   float* distance);

apStatus_t CalculateDirectionDistance(const Eigen::VectorXf& src,
                                      const Eigen::VectorXf& dst,
                                      float* distance);

apStatus_t CalculateLogGaussSimilarity(const Eigen::VectorXf& x,
                                       const Eigen::MatrixXf& cov,
                                       float* similarity);

Eigen::Vector3f TransPointByHomography(const Eigen::MatrixXf& homography,
                                       const Eigen::VectorXf& point);

// @brief: project src(a) to dst (b) vector, get the distance.
// @return: dot(a, b) / norm(b)
void CalulateProjectionDistance2D(const Eigen::Vector2f& src_vector,
                                  const Eigen::Vector2f& dst_vector,
                                  float* projection_distance);

apStatus_t CalculateVelDirGuidedDistance(const Eigen::Vector3f& src_pos,
                                         const Eigen::Vector3f& dst_pos,
                                         const Eigen::Vector3f& src_vel,
                                         float* vel_dir_distance,
                                         float* vertical_vel_dir_distance);

}  // namespace perception
}  // namespace senseAD
