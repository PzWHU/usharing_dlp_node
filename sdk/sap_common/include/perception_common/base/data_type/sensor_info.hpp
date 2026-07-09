/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include "Eigen/Core"

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

enum class SensorType {
    UNKNOWN_SENSOR = -1,
    CAMERA = 0,
    LIDAR = 1,
    RADAR = 2,
    ULTRASONIC = 3,
    AGGREGATOR = 4
};
enum class MotionDirection { UNKNOWN = -1, FRONT, BACK };

static const std::unordered_map<std::string, SensorType> kLabelToSensorType{
    {"UNKNOWN_SENSOR", SensorType::UNKNOWN_SENSOR},
    {"CAMERA", SensorType::CAMERA},
    {"LIDAR", SensorType::LIDAR},
    {"RADAR", SensorType::RADAR},
    {"ULTRASONIC", SensorType::ULTRASONIC}};

static const std::map<SensorType, std::string> kSensorTypeToString{
    {SensorType::UNKNOWN_SENSOR, "UNKNOWN_SENSOR"},
    {SensorType::CAMERA, "CAMERA"},
    {SensorType::LIDAR, "LIDAR"},
    {SensorType::RADAR, "RADAR"},
    {SensorType::ULTRASONIC, "ULTRASONIC"}};

inline std::string GetSensorTypeDescription(const SensorType& status) {
    std::string status_description = "";
    auto iter = kSensorTypeToString.find(status);
    if (iter != kSensorTypeToString.end()) {
        status_description = iter->second;
    }
    return status_description;
}

// sensor name type description
// TODO(zhangshiquan): remove definition into calibration manager op and init by
// prototxt
static const std::unordered_map<std::string, SensorType>
    kSensorNameToSensorType{
        // unknow
        {"unknown_sensor", SensorType::UNKNOWN_SENSOR},
        // cameras
        {"camera_sensor", SensorType::CAMERA},
        {"center_camera_fs", SensorType::CAMERA},
        {"center_camera_fov120", SensorType::CAMERA},
        {"center_camera_fov60", SensorType::CAMERA},
        {"center_camera_fov30", SensorType::CAMERA},
        {"center_camera", SensorType::CAMERA},
        {"front_left_camera", SensorType::CAMERA},
        {"front_right_camera", SensorType::CAMERA},
        {"left_front_camera", SensorType::CAMERA},
        {"left_rear_camera", SensorType::CAMERA},
        {"right_front_camera", SensorType::CAMERA},
        {"right_rear_camera", SensorType::CAMERA},
        {"rear_camera", SensorType::CAMERA},
        {"center_camera_fov30_gop", SensorType::CAMERA},
	    {"center_camera_fov120_gop", SensorType::CAMERA},
        {"nv_cameras_center_camera_fov120", SensorType::CAMERA},
        {"nv_cameras_center_camera_fov60", SensorType::CAMERA},
        {"nv_cameras_center_camera_fov30", SensorType::CAMERA},
        {"nv_cameras_left_front_camera", SensorType::CAMERA},
        {"nv_cameras_left_rear_camera", SensorType::CAMERA},
        {"nv_cameras_right_front_camera", SensorType::CAMERA},
        {"nv_cameras_right_rear_camera", SensorType::CAMERA},
        {"nv_cameras_rear_camera", SensorType::CAMERA},
        {"nv_cameras", SensorType::CAMERA},
        {"camera_1", SensorType::CAMERA},
        {"camera_2", SensorType::CAMERA},
        {"camera_3", SensorType::CAMERA},
        {"camera_4", SensorType::CAMERA},
        {"camera_5", SensorType::CAMERA},
        {"camera_6", SensorType::CAMERA},
        // lidars
        {"lidar_sensor", SensorType::LIDAR},
        {"top_center_lidar", SensorType::LIDAR},
        {"front_lidar", SensorType::LIDAR},
        {"left_lidar", SensorType::LIDAR},
        {"right_lidar", SensorType::LIDAR},
        // radars
        {"radar_sensor", SensorType::RADAR},
        {"front_radar", SensorType::RADAR},
        {"rear_left_radar", SensorType::RADAR},
        {"rear_right_radar", SensorType::RADAR},
        {"front_left_radar", SensorType::RADAR},
        {"front_right_radar", SensorType::RADAR},
        {"rear_radar", SensorType::RADAR},
        // aggregator
        {"aggregator", SensorType::AGGREGATOR}};

inline apStatus_t CheckSensorNameExist(const std::string& sensor_name) {
    if (kSensorNameToSensorType.find(sensor_name) ==
        kSensorNameToSensorType.end()) {
        AP_LERROR(CheckSensorNameExist)
            << "Sensor name -> " << sensor_name
            << " not found in kSensorNameToSensorType, please check "
               "base/data_type/sensor_info.hpp";
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    return AP_SUCCESS;
}

inline bool IsCamera(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckSensorNameExist(sensor_name)) {
        return false;
    }
    if (SensorType::CAMERA == kSensorNameToSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsLidar(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckSensorNameExist(sensor_name)) {
        return false;
    }
    if (SensorType::LIDAR == kSensorNameToSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsRadar(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckSensorNameExist(sensor_name)) {
        return false;
    }
    if (SensorType::RADAR == kSensorNameToSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsSameSensorType(const std::string& src_sensor_name,
                             const std::string& dst_sensor_name) {
    if (AP_SUCCESS != CheckSensorNameExist(src_sensor_name)) {
        return false;
    }
    if (AP_SUCCESS != CheckSensorNameExist(dst_sensor_name)) {
        return false;
    }
    if (kSensorNameToSensorType.at(src_sensor_name) ==
        kSensorNameToSensorType.at(dst_sensor_name)) {
        return true;
    }
    return false;
}
enum class RadarNearestPointType {
    CENTER = 0,
    FRONT_LEFT,
    FRONT_CENTER,
    FRONT_RIGHT,
    CENTER_LEFT,
    CENTER_RIGHT,
    BACK_LEFT,
    BACK_CENTER,
    BACK_RIGHT
};

static const std::map<RadarNearestPointType, std::string>
    kRadarNearestPointTypeToString{
        {RadarNearestPointType::CENTER, "CENTER"},
        {RadarNearestPointType::FRONT_LEFT, "FRONT_LEFT"},
        {RadarNearestPointType::FRONT_CENTER, "FRONT_CENTER"},
        {RadarNearestPointType::FRONT_RIGHT, "FRONT_RIGHT"},
        {RadarNearestPointType::CENTER_LEFT, "CENTER_LEFT"},
        {RadarNearestPointType::CENTER_RIGHT, "CENTER_RIGHT"},
        {RadarNearestPointType::BACK_LEFT, "BACK_LEFT"},
        {RadarNearestPointType::BACK_CENTER, "BACK_CENTER"},
        {RadarNearestPointType::BACK_RIGHT, "BACK_RIGHT"}};

typedef std::map<RadarNearestPointType, Eigen::Vector2f>
    RadarNearestPointTypeVectorMap;
typedef std::shared_ptr<RadarNearestPointTypeVectorMap>
    RadarNearestPointTypeVectorMapPtr;
static const RadarNearestPointTypeVectorMap kRadarNearestPointTypeToObjectPoint{
    {RadarNearestPointType::CENTER, Eigen::Vector2f(0, 0)},
    {RadarNearestPointType::FRONT_LEFT, Eigen::Vector2f(1, 1)},
    {RadarNearestPointType::FRONT_CENTER, Eigen::Vector2f(1, 0)},
    {RadarNearestPointType::FRONT_RIGHT, Eigen::Vector2f(1, -1)},
    {RadarNearestPointType::CENTER_LEFT, Eigen::Vector2f(0, 1)},
    {RadarNearestPointType::CENTER_RIGHT, Eigen::Vector2f(0, -1)},
    {RadarNearestPointType::BACK_LEFT, Eigen::Vector2f(-1, 1)},
    {RadarNearestPointType::BACK_CENTER, Eigen::Vector2f(-1, 0)},
    {RadarNearestPointType::BACK_RIGHT, Eigen::Vector2f(-1, -1)}};

enum class ObjectFaceType {
    FRONT = 0,
    BACK,
    LEFT,
    RIGHT,
};

static const std::map<ObjectFaceType, std::string> kObjectFaceTypeToString{
    {ObjectFaceType::FRONT, "FRONT"},
    {ObjectFaceType::BACK, "BACK"},
    {ObjectFaceType::LEFT, "LEFT"},
    {ObjectFaceType::RIGHT, "RIGHT"}};

static const std::map<ObjectFaceType, std::vector<RadarNearestPointType>>
    kObjectFaceTypeToRadarNearestPointType{
        {ObjectFaceType::FRONT,
         {RadarNearestPointType::FRONT_LEFT,
          RadarNearestPointType::FRONT_CENTER,
          RadarNearestPointType::FRONT_RIGHT}},
        {ObjectFaceType::BACK,
         {RadarNearestPointType::BACK_LEFT, RadarNearestPointType::BACK_CENTER,
          RadarNearestPointType::BACK_RIGHT}},
        {ObjectFaceType::LEFT,
         {RadarNearestPointType::FRONT_LEFT,
		  RadarNearestPointType::CENTER_LEFT,
          RadarNearestPointType::BACK_LEFT}},
        {ObjectFaceType::RIGHT,
         {RadarNearestPointType::FRONT_RIGHT,
          RadarNearestPointType::CENTER_RIGHT,
          RadarNearestPointType::BACK_RIGHT}}};

}  // namespace perception
}  // namespace senseAD
