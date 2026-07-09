/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"

// #include "perception_interface/data_type/object/radar/radar_object.hpp"

namespace senseAD {
namespace perception {

const int MAX_RADAR_IDX = 2147483647;
const int ORIGIN_CONTI_MAX_ID_NUM = 100;
const int ORIGIN_DELPHI_MAX_ID_NUM = 64;
const float MIN_PROBEXIST = 0.08;

// @brief input interface
typedef struct StructInputRadarRawObject {
    uint64_t timestamp_ns;
    std::string radar_type;
    // Object Tracking ID
    int track_id = -1;

    // Object Type
    // 0 - Vehicle, 1 - Bike, 2 - Pedestrian, 3 - Unknown
    int object_type;

    float theta = 0.0f;

    /* @brief (range, angle) in radar polar coordinate system
      x for forward and y for left
    */
    float range = 0.0f;
    float angle = 0.0f;

    float relative_radial_velocity = 0.0f;
    float relative_tangential_velocity = 0.0f;
    float radial_velocity = 0.0f;

    virtual ~StructInputRadarRawObject() {}
    virtual StructInputRadarRawObject* GetThisPtr() = 0;
} InputRadarRawObject;

enum class ARSObjectType {
    UNKNOWN = -1,
    POINT,
    CAR,
    TRUCK,
    PEDESTRIAN,
    MOTORCYCLE,
    BICYCLE,
    WIDE,
    RESERVED
};

typedef struct StructESRRadarConfig {
    float lateral_offset = 0.0f;
    float misalignment = 0.0f;
    bool positive_points_to_drivers_right = false;
    bool clockwise = false;
    bool is_deg = false;
    bool enable_raw_data = false;
} ESRRadarConfig;

struct ContinentalARSObject : public InputRadarRawObject {
    // @brief valid only for on_use = true
    bool on_use = false;
    // The Cartesian Coordinate is a right-hand car coordinate

    // Radar Timestamp
    uint64_t timestamp_ns;
    // Radar Checksum & Counter
    uint8_t check_sum;
    uint8_t counter;
    // Radar Detected Object Acceleartion

    // Radar Detected Object Velocity
    // Range From -51.1 - 51.2
    float velocity_x;
    float velocity_y;
    // Radar Detected Object Position
    // Range From -127 - 127
    float position_x;
    // Range From -102.3 - 102.3
    float position_y;
    // Coasted or not
    bool coasted;
    // Object Confidence
    float confidence;
    // Object dynamic property
    // Range From 0 - 7
    // 0 - moving, 1 - stationary, 2 - oncoming, 3 - stationary candidate,
    // 4 - unknown, 5 - crossing stationary, 6 - crossing moving, 7 - stopped
    int dynprop;
    // Object RCS
    float rcs;
    // Object Motion Pattern
    // 0 - Unknown, 1 - Moving, 2 - Reserved
    int motion_pattern;
    // Object Tracking ID
    int track_id;

    // Object Alive Time
    float alive_time;
    // Object Heading Direction
    // Range From -180 - 180
    float heading;

    // Object quality information
    // Detail information refers to ARS408_Technical_Documentation
    // Obj_DistLong_rm
    // Range From 0 - 31
    float dist_long_rms;
    // Obj_VrelLong_rms
    // Range From 0 - 31
    float vrel_long_rms;
    // Obj_DistLat_rms
    float dist_lat_rms;
    // Obj_VrelLat_rms
    float vrel_lat_rms;
    // Obj_ArelLat_rms
    float arel_lat_rms;
    // Obj_ArelLong_rms
    float arel_long_rms;
    // Obj_Orientation_rms
    float orientation_rms;
    // Obj_MeasState
    // Range From 0 - 7
    int meas_state;
    // Obj_ProbOfExist
    // Range From 0 - 7
    float prob_of_exist;

    // Object extended information
    // Detail information refers to ARS408_Technical_Documentation
    // Object_ArelLong
    // Range From -10.00 - 10.47
    float acceleration_x;
    // Object_Class
    // Range From 0 - 7
    // 0 - point, 1 - car, 2 - truck, 3 -  Pedestrian,
    // 4 - motorcycle, 5 - bicycle, 6 - wide, 7 - reserved
    int object_type;
    // Object_ArelLat
    // Range From -2.50 - 2.61
    float acceleration_y;
    // Object_OrientationAngel
    // Object Heading Direction
    // Range From -180 - 180 deg
    float orientation_angel;
    // Object_Length
    // Range From 0.0 - 51.0
    float length;
    // Object_Width
    // Range From 0.0 - 51.0
    float width;  // uint8_t

    // Cluster quality information
    // Cluster_Pdh0
    int false_alarm;
    // AmbigState
    int ambig_state;
    // Cluster_InvalidState
    int invalid_state;
    // @brief object type, required
    ARSObjectType type = ARSObjectType::UNKNOWN;
    float underpass_probability;
    ContinentalARSObject* GetThisPtr() override { return this; }
};

struct DelphiESRObject : public InputRadarRawObject {
    // @brief valid only for on_use = true
    bool on_use = false;
    // CanFrame timestamp
    uint64_t timestamp_ns;
    // Radar object ID from 0 - 63 given by can_frame id 0x500-0x53F
    uint8_t trackID;
    // Radar scan index
    uint16_t scan_index;
    // Lateral Rate (+) = counter clockwise, resolution 0.25m/s
    // set at 7.75 if > 7.75, set at -8 if < -8
    float track_lat_rate_m_per_s;
    // Boolean indicator that number of detections
    // associated with the track have changed
    // 0 = no change
    // 1 = change
    bool track_group_changed;
    // Oncoming Flag
    // 0 = not oncoming
    // 1 = oncoming
    bool track_oncoming;
    // Moving Flag
    // 0 = static
    // 1 = movable
    bool track_moving;
    bool track_moving_valid = false;
    // Measurement Status
    // 0 = no target
    // 1 = new target
    // 2 = new updated target
    // 3 = updated target
    // 4 = coasted target
    // 5 = merged target
    // 6 = invalid coasted target
    // 7 = new coasted target
    uint8_t track_status;
    // Azimuth 0 = toward front of vehicle parallel to
    // vehicle centerline (+) = counter clockwise
    // set at 51.1 * M_PI / 180 if > 51.1 * M_PI / 180
    // set at -51.2 * M_PI / 180 if < -51.2 * M_PI /180
    float track_angle_rad;
    // Range in meter, away from the ESR radar sensor, resolution 0.1m.
    // set at 204.7 if > 204.7
    float track_range_m;
    // Indicates if target is a bridge or not
    bool track_bridge_object;
    // Rolling Count must = rolling count of all messages in this burst
    bool track_rolling_count;
    // Width, ranging from 0 to 7.5, resolution 0.5. set at 7.5 if > 7.5
    float track_width_m;
    // Range Acceleration, (+) = away from sensor, resolution 0.05 m/s^2
    // set at 25.55 if > 25.55
    // set at -25.6 if < -25.6
    float track_range_accel_m_per_s2;
    // Indicates which mode updated the fused track
    // 0 = No MR / LR update
    // 1 = MR update only
    // 2 = LR update only
    // 3 = Both MR / LR update
    uint8_t track_med_range_mode;
    // Range Rate (+) = away from sensor, resolution 0.01 m/s
    // set at 81.91 if > 81.91
    // set at -81.92 if < -81.92
    float track_range_rate_m_per_s;

    DelphiESRObject* GetThisPtr() override { return this; }
};

struct DelphiSRRObject : public InputRadarRawObject {
    // @brief valid only for on_use = true
    bool on_use = false;

    // Currently types are not sure so I just make a speculation. And they
    // should be made fixed bits.
    // All the following properties are changed into Cartesian Coordinate by
    // radar itself
    // The Cartesian Coordinate is a right-hand car coordinate
    // Radar Timestamp
    uint64_t timestamp_ns;
    // Radar Checksum & Counter
    uint8_t check_sum;
    uint8_t counter;
    // Radar Detected Object Acceleartion
    // Range From -25.6 - 25.6
    float acceleration_x;
    // Range From -6.4 - 6.4
    float acceleration_y;
    // Radar Detected Object Velocity
    // Range From -51.1 - 51.2
    float velocity_x;
    float velocity_y;
    // Radar Detected Object Position
    // Range From -127 - 127
    float position_x;
    // Range From -102.3 - 102.3
    float position_y;
    // Coasted or not
    bool coasted;
    // Object Confidence
    float confidence;
    // Object Motion Pattern
    // 0 - Unknown, 1 - Moving, 2 - Reserved
    int motion_pattern;
    // Object Tracking ID
    int track_id;
    // Object Type
    // 0 - Vehicle, 1 - Bike, 2 - Pedestrian, 3 - Unknown
    int object_type;
    // Object Width
    // Range From 0 - 5
    float width;
    // Object Alive Time
    float alive_time;
    // Object Heading Direction
    // Range From -180 - 180
    float heading;

    DelphiSRRObject* GetThisPtr() override { return this; }
};

enum ContiObjectType {
    CONTI_POINT = 0,
    CONTI_CAR = 1,
    CONTI_TRUCK = 2,
    CONTI_PEDESTRIAN = 3,
    CONTI_MOTOCYCLE = 4,
    CONTI_BICYCLE = 5,
    CONTI_WIDE = 6,
    CONTI_TYPE_UNKNOWN = 7,
    CONTI_MAX_OBJECT_TYPE = 8
};

enum ContiMeasState {
    CONTI_DELETED = 0,
    CONTI_NEW = 1,
    CONTI_MEASURED = 2,
    CONTI_PREDICTED = 3,
    CONTI_DELETED_FOR = 4,
    CONTI_NEW_FROM_MERGE = 5
};

enum ContiDynProp {
    CONTI_MOVING = 0,
    CONTI_STATIONARY = 1,
    CONTI_ONCOMING = 2,
    CONTI_STATIONARY_CANDIDATE = 3,
    CONTI_DYNAMIC_UNKNOWN = 4,
    CONTI_CROSSING_STATIONARY = 5,
    CONTI_CROSSING_MOVING = 6,
    CONTI_STOPPED = 7
};

const float CONTI_PROBOFEXIST[8] = {0.00f, 0.25f, 0.5f,   0.75f,
                                    0.90f, 0.99f, 0.999f, 1.0f};

// quality info
const float CONTI_LINEAR_RMS[32] = {
    0.005, 0.006, 0.008, 0.011, 0.014, 0.018, 0.023,  0.029,
    0.038, 0.049, 0.063, 0.081, 0.105, 0.135, 0.174,  0.224,
    0.288, 0.371, 0.478, 0.616, 0.794, 1.023, 1.317,  1.697,
    2.187, 2.817, 3.630, 4.676, 6.025, 7.762, 10.000, 100.00};

const float CONTI_ANGLE_RMS[32] = {
    0.005,  0.007,  0.010,  0.014,  0.020,  0.029,   0.041,   0.058,
    0.082,  0.116,  0.165,  0.234,  0.332,  0.471,   0.669,   0.949,
    1.346,  1.909,  2.709,  3.843,  5.451,  7.734,   10.971,  15.565,
    22.061, 31.325, 44.439, 63.044, 69.437, 126.881, 180.000, 360.00};

}  // namespace perception
}  // namespace senseAD
