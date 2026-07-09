/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "Eigen/Core"

#include "perception_common/base/data_type/base.hpp"
#include "perception_common/base/data_type/object_label.hpp"
#include "perception_common/data_type/sensor_info.hpp"

namespace senseAD {
namespace perception {

const Eigen::Matrix3f kDefaultUncertainty = Eigen::Matrix3f::Zero();

typedef struct StructPolygonBox : ValidInfo {
    std::vector<Eigen::Vector2f> polygon_contour;
    float height_lower_surface = -1.0;
    float height_upper_surface = -1.0;
} PolygonBox;

typedef struct StructBirdViewCorners : ValidInfo {
    std::vector<Eigen::Vector3f> bv_corners;
    std::vector<float> corner_conf;
    bool is_valid = false;
    StructBirdViewCorners() {
        bv_corners.clear();
        corner_conf.clear();
        is_valid = false;
    }
} BirdViewCorners;

enum class MotionStatus {
    INVALID_STATUS = -1,
    UNKNOWN = 0,
    STATIC,
    STOPPED,
    MOVING,
    CROSSING
};

enum class MaintenanceStatus {
    INVALID_STATUS = -1,
    STATE_NEW = 0,
    STATE_DELETED,
    STATE_MEASURED,
    STATE_MERGE_NEW,
    STATE_PREDICTED,
    STATE_MERGE_DELETED
};

enum class OcclusionStatus {
    INVALID_STATUS = -1,
    NO_OBJECT = 0,
    UNKNOWN,
    PARTIAL_OCCLUSIVE,
    COMPLETE_OCCLUSIVE
};

// @brief sensor source refers to HyperMode in fusion
enum class SensorSource {
    INVALID = -1,
    NONE = 0,
    CAMERA_ONLY = 1,
    LIDAR_ONLY = 2,
    RADAR_ONLY = 3,
    CAMERA_LIDAR = 4,
    CAMERA_RADAR = 5,
    LIDAR_RADAR = 6,
    CAMERA_LIDAR_RADAR = 7
};

enum class SplitMergeState {
    INVALID_STATE = -1,
    STATE_NONE = 0,
    STATE_MERGE,
    STATE_SPLIT,
    STATE_MAX_DIFF_TYPES
};

typedef struct StructSizeInfo : ValidInfo {
    // @brief size = [width, length, height] of boundingbox
    // length is the size of the main direction, meter unit
    Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
    // @brief size variance
    Eigen::Matrix3f size_uncertainty = Eigen::Matrix3f::Zero();
} SizeInfo;

typedef struct StructTimeCost {
    uint64_t timecost_ns = 0;
    uint64_t timecost_ns_raw_to_send = 0;
    bool is_valid = false;
} FrameTimeCostInfo;

typedef struct DirectionInfo : ValidInfo {
    // @brief main direction of the object, (yaw, pitch, roll), required
    Eigen::Vector3f direction_ypr = Eigen::Vector3f(1, 0, 0);
    Eigen::Matrix3f direction_ypr_uncertainty = kDefaultUncertainty;
    // @brief the yaw angle, theta = 0.0 <=> direction(1, 0, 0),
    //     currently roll and pitch are not considered,
    //     make sure direction and theta are consistent, required
    float yaw = 0.0f;
    // @brief theta variance, required
    float yaw_uncertainty = 0.0f;
    // @brief the yaw angle change rate, [rad/s], optional
    float yaw_rate = 0;
    // @brief variance of the yaw angle change rate, optional
    float yaw_rate_uncertainty = 0;
} DirectionInfo;

typedef std::unordered_map<std::string, std::vector<float>> ModelOutputFeatures;

typedef struct StructMotionInfo : ValidInfo {
    // @brief center of the boundingbox (cx, cy, cz), meter unit, required
    Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
    // @brief covariance matrix of the center uncertainty, required
    Eigen::Matrix3f center_uncertainty = kDefaultUncertainty;
    // @brief velocity of the object, required
    Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
    // @brief relative velocity, optional
    Eigen::Vector3f relative_velocity = Eigen::Vector3f(0, 0, 0);
    // @brief covariance matrix of the velocity uncertainty, required
    Eigen::Matrix3f velocity_uncertainty = kDefaultUncertainty;
    // @brief acceleration of the object, required
    Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
    // @brief relative acceleration, optional
    Eigen::Vector3f relative_acceleration = Eigen::Vector3f(0, 0, 0);
    // @brief covariance matrix of the acceleration uncertainty, required
    Eigen::Matrix3f acceleration_uncertainty = kDefaultUncertainty;
    // @brief jerk of the object, optional
    Eigen::Vector3f jerk = Eigen::Vector3f(0, 0, 0);
    // @brief covariance matrix of the acceleration uncertainty, optional
    Eigen::Matrix3f jerk_uncertainty = kDefaultUncertainty;
    // @brief direction of velocity, make sure heading and velocity vector are
    //     consistent, optional
    float velocity_heading = 0;
    // @brief variance of direction of velocity, make sure variance of heading
    //     and velocity vector are consistent, optional
    float velocity_heading_uncertainty = 0;
    // @brief changing rate of velocity direction, optional
    float velocity_heading_rate = 0;
    // @brief variance of changing rate of velocity direction, optional
    float velocity_heading_rate_uncertainty = 0;
    bool is_left_side_truncated = false;
    bool is_right_side_truncated = false;
    bool is_top_side_truncated = false;
    bool is_bottom_side_truncated = false;
} MotionInfo;

typedef struct StructMotionStatusInfo : ValidInfo {
    // @brief motion state of the tracked object, required
    MotionStatus motion_status = MotionStatus::UNKNOWN;
    // @brief unknown score, 0.0 ~1.0, 1.0 means more likely to be true
    float unknown_score;
    // @brief static score, 0.0 ~1.0, 1 is static and 0 is moving
    float static_score;
    // @brief stopped score, 0.0 ~1.0
    float stopped_score;
    // @brief moving score, 0.0 ~1.0
    float moving_score;
    // @brief crossing score, 0.0 ~1.0
    float crossing_score;
} MotionStatusInfo;

typedef struct StructGeneralMergeInfo : ValidInfo {
    // @brief object split merge state, required
    SplitMergeState merge_state = SplitMergeState::INVALID_STATE;
    // @brief object merge id, required
    int64_t merge_id = -1;
    // @brief object split id, required
    int64_t split_id = -1;
} GeneralMergeInfo;

typedef struct StructPerceptionLifeTimeHistory {
    // sensor-level node
    uint64_t raw_sensor_capture_timestamp_ns = 0;
    // single-sensor-perception-level node
    uint64_t sensor_perception_capture_timestamp_ns = 0;
    uint64_t sensor_perception_output_timestamp_ns = 0;
    // fusion-level node
    uint64_t fusion_capture_timestamp_ns = 0;
    uint64_t fusion_output_timestamp_ns = 0;
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    int64_t track_id = -1;
} PerceptionLifeTimeHistory;

struct BaseObject {
    BaseObject() = default;

    // @brief sensor id info, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief record the life time of a single object except from fusion.
    PerceptionLifeTimeHistory life_time_history;
    // @brief object type, required
    ObjectLabel label = ObjectLabel::UNKNOWN;
    // @brief type confidence from a detector, required
    float type_confidence = -1.0f;
    // @brief existence confidence, optional
    float existence_confidence = -1.0f;
    // @brief general object probilibty, the probilibty of influencing ego car
    // movement, required
    float general_object_probilibty = -1.0f;
    // @brief object sub-type, optional
    ObjectLabel sub_type = ObjectLabel::UNKNOWN;
    // @brief probability for each sub-type, optional
    std::vector<float> sub_type_probs;
    // @brief track id, default -1 is invalid value, required
    int64_t track_id = -1;
    // @brief age of the tracked object, second unit, required
    float track_age_time_second = 0;
    // @brief latest update timestamp, ns unit, optional
    uint64_t last_measure_update_timestamp_ns = 0;
    // @brief current prediction timestamp
    uint64_t current_prediction_timestamp_ns = 0;
    // @brief size, required
    SizeInfo size_info;
    // @brief polygon description, optional
    PolygonBox polygon_box;
    // @directrion info, required
    DirectionInfo direction_info;
    BirdViewCorners bv_corners;
    // @motion related info, required
    MotionInfo motion_info;
    // @brief motion status info of the tracked object, required
    MotionStatusInfo motion_status_info;
    // @brief maintenace state of the tracked object, required
    MaintenanceStatus maintenance_status = MaintenanceStatus::INVALID_STATUS;
    // @brief occlusion state of the object, required
    OcclusionStatus occlusion_status = OcclusionStatus::UNKNOWN;
    // @brief sensor source, detected by which sensors, required
    SensorSource sensor_source = SensorSource::INVALID;
    // @brief general split merge info for object, required
    GeneralMergeInfo general_merge_info;
    // @brief features that extracted from perception models
    std::shared_ptr<ModelOutputFeatures> feature;

    virtual ~BaseObject() {}
    virtual BaseObject *GetThisPtr() = 0;
};

typedef std::shared_ptr<BaseObject> BaseObjectPtr;
typedef std::shared_ptr<const BaseObject> BaseObjectConstPtr;

// attribute status
enum class AttrBool { DEFAULT = -2, UNKNOWN = -1, NEGATIVE, POSITIVE };

}  // namespace perception
}  // namespace senseAD
