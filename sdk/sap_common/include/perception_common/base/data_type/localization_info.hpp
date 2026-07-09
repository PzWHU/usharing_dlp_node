/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

#include <map>
#include <memory>
#include <vector>
#include <deque>
#include <string>

#include "Eigen/Core"

#include "perception_common/base/data_type/base.hpp"

namespace senseAD {
namespace perception {

// @brief A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most modules, including localization and routing, generate results based on
// the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection.
// The z field of PointENU_t can be omitted. If so, it is a 2D location
// and we do not care its height.
struct PointENU_t {
    float64_t x = 0;  // East from the origin, in meters.
    float64_t y = 0;  // North from the origin, in meters.
    float64_t z = 0;  // Up from the WGS-84 ellipsoid, in meters.
    PointENU_t() = default;
    PointENU_t(float64_t _x, float64_t _y, float64_t _z = 0)
        : x(_x), y(_y), z(_z) {}
    friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                    const PointENU_t& point) {
        out << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return out;
    }
    friend PointENU_t operator+(const PointENU_t& a, const PointENU_t& b) {
        PointENU_t c;
        c.x = a.x + b.x;
        c.y = a.y + b.y;
        c.z = a.z + b.z;
        return c;
    }
    friend PointENU_t operator-(const PointENU_t& a, const PointENU_t& b) {
        PointENU_t c;
        c.x = a.x - b.x;
        c.y = a.y - b.y;
        c.z = a.z - b.z;
        return c;
    }
    friend PointENU_t operator*(const PointENU_t& a, const float& ratio) {
        PointENU_t c;
        c.x = a.x * ratio;
        c.y = a.y * ratio;
        c.z = a.z * ratio;
        return c;
    }
};

// @brief A general 3D point in double. Its meaning and units depend on context,
// and must be explained in comments.
typedef PointENU_t Point3D_t;

// @brief A point in the global reference frame. Similar to PointENU_t,
// PointLLH_t allows omitting the height field for representing a 2D location.
struct PointLLH_t {
    // Longitude in degrees, ranging from -180 to 180.
    float64_t lon = 0;
    // Latitude in degrees, ranging from -90 to 90.
    float64_t lat = 0;
    // WGS-84 ellipsoid height in meters.
    float64_t height = 0;
    PointLLH_t() = default;
    PointLLH_t(float64_t _lon, float64_t _lat, float64_t _height)
        : lon(_lon), lat(_lat), height(_height) {}
};

// @brief A EulerAngles_t that represents a spatial rotation.
typedef struct StructEulerAngles_t {
    // Right-handed rotation from local level around y‑axis, in radius.
    float64_t roll;
    // Right-handed rotation from local level around x‑axis, in radius.
    float64_t pitch;
    // Left-handed rotation around z-axis clockwise from North, in radius.
    float64_t yaw;
} EulerAngles_t;

// @brief A unit Quaternion_t also represents a spatial rotation.
// Most of the time, qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
typedef struct StructQuaternion_t {
    float64_t qx;
    float64_t qy;
    float64_t qz;
    float64_t qw;
} Quaternion_t;

enum class InsStatus {
    /**
     * Do NOT use.
     * Invalid solution due to insufficient observations, no initial GNSS, ...
     */
    INVALID = 0,

    /**
     * Use with caution. The covariance matrix may be unavailable or incorrect.
     * High-variance result due to aligning, insufficient vehicle dynamics, ...
     */
    CONVERGING = 100,

    // Safe to use. The INS has fully converged.
    GOOD = 200
};

enum class GnssStatus {
    /**
     * Not using the GNSS solution if status is INVALID or PROPAGATED.
     * INVALID solution due to insufficient observations, integrity warning...
     */
    INVALID = 0,
    // PROPAGATED by a Kalman filter without new observations.
    PROPAGATED,

    /**
     * Recommended using the following types of solution.
     * Standard GNSS solution without any corrections.
     */
    SINGLE = 100,
    // Pseudorange differential solution, including WAAS/SBAS solution.
    PSRDIFF,
    // Precise Point Positioning (PPP) solution.
    PPP,
    // Real Time Kinematic (RTK) float solution.
    RTK_FLOAT,
    // SINGLE, PSRDIFF, PPP, RTK_FLOAT or ... are OK, for compatibility.
    OK = 199,

    // RTK integer solution.
    RTK_INTEGER = 200,
    // RTK_INTEGER or ... are GOOD, for compatibility.
    GOOD = 299
};

enum class LMSFStatus {
    INVALID = 0,
    CONVERGING = 1,
    UNSTABLE = 2,
    OK = 3,
    GOOD = 4,
};

static inline std::string GetInsStatusStr(const InsStatus& status) {
    std::string status_str = "";
    static const std::map<InsStatus, std::string> InsStatusTable{
        {InsStatus::INVALID, "INVALID"},
        {InsStatus::CONVERGING, "CONVERGING"},
        {InsStatus::GOOD, "GOOD"}};
    const auto& it = InsStatusTable.find(status);
    if (it != InsStatusTable.end()) {
        status_str = it->second;
    }

    return status_str;
}

static inline std::string GetGnssStatusStr(const GnssStatus& status) {
    std::string status_str = "";
    static const std::map<GnssStatus, std::string> GnssStatusTable{
        {GnssStatus::INVALID, "INVALID"},
        {GnssStatus::PROPAGATED, "PROPAGATED"},
        {GnssStatus::SINGLE, "SINGLE"},
        {GnssStatus::PSRDIFF, "PSRDIFF"},
        {GnssStatus::PPP, "PPP"},
        {GnssStatus::RTK_FLOAT, "RTK_FLOAT"},
        {GnssStatus::OK, "OK"},
        {GnssStatus::RTK_INTEGER, "RTK_INTEGER"},
        {GnssStatus::GOOD, "GOOD"}};
    const auto& it = GnssStatusTable.find(status);
    if (it != GnssStatusTable.end()) {
        status_str = it->second;
    }

    return status_str;
}

static inline std::string GetLMSFStatusStr(const LMSFStatus& status) {
    std::string status_str = "";
    static const std::map<LMSFStatus, std::string> LMSFStatusTable{
        {LMSFStatus::INVALID, "INVALID"},
        {LMSFStatus::CONVERGING, "CONVERGING"},
        {LMSFStatus::UNSTABLE, "UNSTABLE"},
        {LMSFStatus::OK, "OK"},
        {LMSFStatus::GOOD, "GOOD"}};
    const auto& it = LMSFStatusTable.find(status);
    if (it != LMSFStatusTable.end()) {
        status_str = it->second;
    }
    return status_str;
}

static inline uint8_t GetLMSFStatusNum(const LMSFStatus& status) {
    switch (status) {
        case LMSFStatus::INVALID:
            return 0;
        case LMSFStatus::CONVERGING:
            return 1;
        case LMSFStatus::UNSTABLE:
            return 2;
        case LMSFStatus::OK:
            return 3;
        case LMSFStatus::GOOD:
            return 4;
        default:
            return 0;
    }
    return 0;
}

static inline LMSFStatus GetMSFStatusEnum(uint8_t num) {
    switch (num) {
        case 0:
            return LMSFStatus::INVALID;
        case 1:
            return LMSFStatus::CONVERGING;
        case 2:
            return LMSFStatus::UNSTABLE;
        case 3:
            return LMSFStatus::OK;
        case 4:
            return LMSFStatus::GOOD;
        default:
            return LMSFStatus::INVALID;
    }
    return LMSFStatus::INVALID;
}

typedef struct StructLocalizationStatus {
    // INS Status
    InsStatus ins_status;
    // GNSS Status
    GnssStatus gnss_status;
    // Localization MSF Status
    LMSFStatus lmsf_status;
} LocalizationStatus;

typedef struct StructCarPose {
    // time stamp of current car position, in nano second
    uint64_t timestamp_ns;

    /* 1. if this class is used for global position
     *    ENU protocol, x for East, y for North, z for Up Height, in meters
     * 2. if this class is used for car frame which is relative
     *    to the car start point
     *    x for forward(car head), y for left (car left), z for up height,
     *    in meters
     * 3. else self-definition
     */
    Point3D_t position;

    /* euler angles representation of car attitude
     * 1. if this class is used for global position
     *    roll around x axis, pitch around y axis, yaw around z aixs
     * 2. if this class is used for car frame which is relative
     *    to the car start point
     *    roll around x axis, pitch around y axis, yaw around z axis
     * 3. else self-definition
     */
    EulerAngles_t attitude;

    /**
     * 1. if this class is used for global position
     *    Heading of the Vehicle Reference Point(VRP) in the Map reference frame
     *    East = 0, North = pi/2, anti-clockwise in radius
     * 2. if this class is used for car frame which is relative
     *    to the car start point
     *    forward = 0, left = pi/2, anti-clockwise in radius
     * 3. esle self-definition
     */
    double heading;
} CarPose;

typedef struct StructSE3Pose {
    // time stamp of current car pose, in nano second
    uint64_t timestamp_ns;

    // car position in meters
    Point3D_t position;

    // quaternion representation of car attitude in Hamilton convention
    Quaternion_t quat;
} SE3Pose;

typedef struct StructOdometryInfo {
    /**
     * Position of the Vehicle Reference Point(VRP) in the Map reference frame.
     * The VRP is the center of rear axle.
     * x for East, y for North, z for Up Height, in meters
     */
    PointENU_t position;

    /** Position standard deviation in meters **/
    Point3D_t position_std;

    /**
     * Position of the Vehicle Reference Point(VRP) in the WGS84 Coordinate.
     */
    PointLLH_t position_lla;

    /**
     * Attitude of the VRP in the Map reference frame(East/North/Up) in radians.
     * The roll, in (-pi/2, pi/2), corresponds to a rotation around the x-axis.
     * The pitch, in [-pi, pi), corresponds to a rotation around the y-axis.
     * The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
     * The direction of rotation follows the right-hand rule.
     */
    EulerAngles_t euler_angle;

    /** Attitude standard deviation in radians **/
    Point3D_t attitude_std;

    /**
     * Linear velocity of the VRP in the Vehicle reference frame
     * x for Forward, y for Left, z for Up, in meters per second
     */
    Point3D_t linear_velocity;

    /**
     * Linear velocity of the VRP in the ENU reference frame
     * x for East, y for North, z for Up, in meters per second
     */
    Point3D_t global_linear_velocity;

    /**
     * Linear acceleration of the VRP in the Vehicle reference frame
     * x for Forward, y for Left, z for Up, in meters per power second
     */
    Point3D_t linear_acceleration;

    /**
     * Linear acceleration of the VRP in the ENU reference frame
     * x for East, y for North, z for Up, in meters per power second
     */
    Point3D_t global_linear_acceleration;

    /**
     * Angular velocity of the VRP in the Vehicle reference frame
     * x across Forward axes, y across Left axes,
     * z across Up axes, in radians per second
     */
    Point3D_t angular_velocity;
    double heading;
} OdometryInfo;

static constexpr uint64_t kWindowSize = 1000;
typedef struct StructLocalizationInfo {
    // latest localization information
    // The time of pose measurement, in nano second.
    uint64_t measurement_time_ns;
    // Localization Fusion Status
    LMSFStatus localization_status;
    // Integral Vehicle Localization Information
    OdometryInfo odometry_info;
    // array of history localization information
    uint64_t valid_len = 0;
    // history of Global car pose
    SE3Pose global_pose_window[kWindowSize];
    // history of ego-car velocity
    Point3D_t velocity_window[kWindowSize];
    // history of ego-car angular speed
    Point3D_t angular_speed_window[kWindowSize];
    // history of ego-car acceleration
    Point3D_t acceleration_window[kWindowSize];

    StructLocalizationInfo& operator=(const StructLocalizationInfo& other) {
        if (&other == this) {
            return *this;
        }
        memcpy(this, &other, sizeof(StructLocalizationInfo));
        return *this;
    }
} LocalizationInfo;

typedef struct StructLocalLocalizationInfo {
    uint64_t measurement_time_ns;
    uint32_t accuracy_level =
        2;  // 0-invald, 1-low precision, 2-medium/high precision
    SE3Pose local_pose;
    Point3D_t ego_car_angular_velocity;
    Point3D_t ego_car_linear_velocity;
    Point3D_t ego_car_linear_acceleration;
    Quaternion_t ego_car_to_ground_rotation;
    Point3D_t local_translation_covariance;
    Point3D_t local_rotation_covariance;
    /** Position standard deviation in meters **/
    Point3D_t position_std;
    /**
     * Attitude of the VRP in the Map reference frame(East/North/Up) in radians.
     * The roll, in (-pi/2, pi/2), corresponds to a rotation around the x-axis.
     * The pitch, in [-pi, pi), corresponds to a rotation around the y-axis.
     * The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
     * The direction of rotation follows the right-hand rule.
     */
    EulerAngles_t euler_angle;
    /** Attitude standard deviation in radians **/
    Point3D_t attitude_std;
} LocalLocalizationInfo;

typedef struct StructDequeLocalizationInfo {
    // latest localization information
    // The time of pose measurement, in nano second.
    uint64_t measurement_time_ns;
    // Localization Fusion Status
    LMSFStatus localization_status;
    // Integral Vehicle Localization Information
    OdometryInfo odometry_info;
    // array of history localization information
    uint64_t valid_len = 0;
    // history of Global car pose
    std::deque<SE3Pose> global_pose_window;
    // history of ego-car velocity
    std::deque<Point3D_t> velocity_window;
    // history of ego-car angular speed
    std::deque<Point3D_t> angular_speed_window;
    // history of ego-car acceleration
    std::deque<Point3D_t> acceleration_window;
} DequeLocalizationInfo;

}  // namespace perception
}  // namespace senseAD
