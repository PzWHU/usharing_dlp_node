/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <unordered_map>

#include "Eigen/Core"

namespace senseAD {
namespace perception {

enum class SensorID {
    // camera
    CAMERA_SENSOR_ID_START = 0,
    CAMERA_SENSOR = 1,
    CENTER_CAMERA,
    FRONT_LEFT_CAMERA,
    FRONT_RIGHT_CAMERA,
    REAR_LEFT_CAMERA,
    REAR_RIGHT_CAMERA,
    REAR_CAMERA,
    CENTER_CAMERA_FOV120,
    CENTER_CAMERA_FOV30,
    NV_CAMERAS,
    NV_FRONT_LEFT_CAMERA,
    NV_FRONT_RIGHT_CAMERA,
    NV_REAR_LEFT_CAMERA,
    NV_REAR_RIGHT_CAMERA,
    NV_REAR_CAMERA,
    NV_CENTER_CAMERA_FOV120,
    NV_CENTER_CAMERA_FOV30,
    CENTER_CAMERA_FOV120_GOP,
    CENTER_CAMERA_FOV30_GOP,
    FRONT_CAMERA_FOV195,
    LEFT_CAMERA_FOV195,
    RIGHT_CAMERA_FOV195,
    REAR_CAMERA_FOV195,
    CAMERA_SENSOR_ID_END,
    // lidar
    LIDAR_SENSOR_ID_START,
    LIDAR_SENSOR,
    TOP_CENTER_LIDAR,
    LEFT_LIDAR,
    RIGHT_LIDAR,
    STITCHING_LIDAR,
    LIDAR_SENSOR_ID_END,
    FRONT_LIDAR,
    // radar
    RADAR_SENSOR_ID_START,
    RADAR_SENSOR,
    FRONT_RADAR,
    REAR_RADAR,
    FRONT_LEFT_RADAR,
    FRONT_RIGHT_RADAR,
    REAR_LEFT_RADAR,
    REAR_RIGHT_RADAR,
    RADAR_SENSOR_ID_END,
    // aggregator
    AGGREGATOR_SENSOR_ID_START,
    AGGREGATOR,
    // unknown
    UNKNOWN_SENSOR,

    CENTER_CAMERA_FOV60,
    NV_CENTER_CAMERA_FOV60,
    CENTER_CAMERA_FOV30_MONO,
    CENTER_CAMERA_FOV20,
    NV_CENTER_CAMERA_FOV20
};

static const std::unordered_map<std::string, SensorID> kSensorNameToSensorID{
    {"unknown_sensor", SensorID::UNKNOWN_SENSOR},
    // camera
    {"camera_sensor", SensorID::CAMERA_SENSOR},
    {"center_camera", SensorID::CENTER_CAMERA},
    {"center_camera_fs", SensorID::CENTER_CAMERA},
    {"front_left_camera", SensorID::FRONT_LEFT_CAMERA},
    {"front_right_camera", SensorID::FRONT_RIGHT_CAMERA},
    {"left_front_camera", SensorID::FRONT_LEFT_CAMERA},
    {"right_front_camera", SensorID::FRONT_RIGHT_CAMERA},
    {"left_rear_camera", SensorID::REAR_LEFT_CAMERA},
    {"right_rear_camera", SensorID::REAR_RIGHT_CAMERA},
    {"rear_camera", SensorID::REAR_CAMERA},
    {"center_camera_fov120", SensorID::CENTER_CAMERA_FOV120},
    {"center_camera_fov60", SensorID::CENTER_CAMERA_FOV60},
    {"center_camera_fov30", SensorID::CENTER_CAMERA_FOV30},
    {"center_camera_fov20", SensorID::CENTER_CAMERA_FOV20},
    {"nv_cameras", SensorID::NV_CAMERAS},
    {"nv_cameras_gop", SensorID::NV_CAMERAS},
    {"nv_cameras_left_front_camera", SensorID::NV_FRONT_LEFT_CAMERA},
    {"nv_cameras_right_front_camera", SensorID::NV_FRONT_RIGHT_CAMERA},
    {"nv_cameras_left_rear_camera", SensorID::NV_REAR_LEFT_CAMERA},
    {"nv_cameras_right_rear_camera", SensorID::NV_REAR_RIGHT_CAMERA},
    {"nv_cameras_rear_camera", SensorID::NV_REAR_CAMERA},
    {"nv_cameras_center_camera_fov120", SensorID::NV_CENTER_CAMERA_FOV120},
    {"nv_cameras_center_camera_fov60", SensorID::NV_CENTER_CAMERA_FOV60},
    {"nv_cameras_center_camera_fov30", SensorID::NV_CENTER_CAMERA_FOV30},
    {"nv_cameras_center_camera_fov20", SensorID::NV_CENTER_CAMERA_FOV20},
    {"center_camera_fov120_gop", SensorID::CENTER_CAMERA_FOV120_GOP},
    {"center_camera_fov120_semantic", SensorID::CENTER_CAMERA_FOV120},
    {"center_camera_fov30_gop", SensorID::CENTER_CAMERA_FOV30_GOP},
    {"center_camera_fov30_mono", SensorID::CENTER_CAMERA_FOV30},
    {"center_camera_fov30_semantic", SensorID::CENTER_CAMERA_FOV30},
    {"front_camera_fov195", SensorID::FRONT_CAMERA_FOV195},
    {"left_camera_fov195", SensorID::LEFT_CAMERA_FOV195},
    {"rear_camera_fov195", SensorID::REAR_CAMERA_FOV195},
    {"right_camera_fov195", SensorID::RIGHT_CAMERA_FOV195},
    // lidar
    {"lidar_sensor", SensorID::LIDAR_SENSOR},
    {"top_center_lidar", SensorID::TOP_CENTER_LIDAR},
    {"left_lidar", SensorID::LEFT_LIDAR},
    {"right_lidar", SensorID::RIGHT_LIDAR},
    {"front_lidar", SensorID::FRONT_LIDAR},
    {"stitching_lidar", SensorID::STITCHING_LIDAR},
    // radar
    {"radar_sensor", SensorID::RADAR_SENSOR},
    {"front_left_radar", SensorID::FRONT_LEFT_RADAR},
    {"front_right_radar", SensorID::FRONT_RIGHT_RADAR},
    {"rear_left_radar", SensorID::REAR_LEFT_RADAR},
    {"rear_right_radar", SensorID::REAR_RIGHT_RADAR},
    {"front_radar", SensorID::FRONT_RADAR},
    {"rear_radar", SensorID::REAR_RADAR},
    // aggregator
    {"aggregator", SensorID::AGGREGATOR}};

static const std::map<SensorID, std::string> kSensorIDToString{
    {SensorID::UNKNOWN_SENSOR, "unknown_sensor"},
    /* camera
     */
    {SensorID::CAMERA_SENSOR, "camera_sensor"},
    {SensorID::CENTER_CAMERA, "center_camera"},
    {SensorID::FRONT_LEFT_CAMERA, "left_front_camera"},
    {SensorID::FRONT_RIGHT_CAMERA, "right_front_camera"},
    {SensorID::REAR_LEFT_CAMERA, "left_rear_camera"},
    {SensorID::REAR_RIGHT_CAMERA, "right_rear_camera"},
    {SensorID::REAR_CAMERA, "rear_camera"},
    {SensorID::CENTER_CAMERA_FOV120, "center_camera_fov120"},
    {SensorID::CENTER_CAMERA_FOV60, "center_camera_fov60"},
    {SensorID::CENTER_CAMERA_FOV30, "center_camera_fov30"},
    {SensorID::CENTER_CAMERA_FOV20, "center_camera_fov20"},
    {SensorID::NV_CAMERAS, "nv_cameras"},
    {SensorID::NV_FRONT_LEFT_CAMERA, "nv_cameras_left_front_camera"},
    {SensorID::NV_FRONT_RIGHT_CAMERA, "nv_cameras_right_front_camera"},
    {SensorID::NV_REAR_LEFT_CAMERA, "nv_cameras_left_rear_camera"},
    {SensorID::NV_REAR_RIGHT_CAMERA, "nv_cameras_right_rear_camera"},
    {SensorID::NV_REAR_CAMERA, "nv_cameras_rear_camera"},
    {SensorID::NV_CENTER_CAMERA_FOV120, "nv_cameras_center_camera_fov120"},
    {SensorID::NV_CENTER_CAMERA_FOV60, "nv_cameras_center_camera_fov60"},
    {SensorID::NV_CENTER_CAMERA_FOV30, "nv_cameras_center_camera_fov30"},
    {SensorID::NV_CENTER_CAMERA_FOV20, "nv_cameras_center_camera_fov20"},
    {SensorID::CENTER_CAMERA_FOV120_GOP, "center_camera_fov120_gop"},
    {SensorID::CENTER_CAMERA_FOV30_GOP, "center_camera_fov30_gop"},
    {SensorID::CENTER_CAMERA_FOV30_MONO, "center_camera_fov30_mono"},
    {SensorID::FRONT_CAMERA_FOV195, "front_camera_fov195"},
    {SensorID::LEFT_CAMERA_FOV195, "left_camera_fov195"},
    {SensorID::RIGHT_CAMERA_FOV195, "right_camera_fov195"},
    {SensorID::REAR_CAMERA_FOV195, "rear_camera_fov195"},
    /* lidar
     */
    {SensorID::LIDAR_SENSOR, "lidar_sensor"},
    {SensorID::TOP_CENTER_LIDAR, "top_center_lidar"},
    {SensorID::STITCHING_LIDAR, "stitching_lidar"},
    // deprecated
    {SensorID::LEFT_LIDAR, "left_lidar"},
    {SensorID::RIGHT_LIDAR, "right_lidar"},
    {SensorID::FRONT_LIDAR, "front_lidar"},
    /* radar
     */
    {SensorID::RADAR_SENSOR, "radar_sensor"},
    {SensorID::FRONT_LEFT_RADAR, "front_left_radar"},
    {SensorID::FRONT_RIGHT_RADAR, "front_right_radar"},
    {SensorID::REAR_LEFT_RADAR, "rear_left_radar"},
    {SensorID::REAR_RIGHT_RADAR, "rear_right_radar"},
    {SensorID::FRONT_RADAR, "front_radar"},
    {SensorID::REAR_RADAR, "rear_radar"},
    /* aggregator
     */
    {SensorID::AGGREGATOR, "aggregator"}};

inline std::string GetSensorIDDescription(const SensorID& id) {
    std::string sensor_id_description = "";
    auto iter = kSensorIDToString.find(id);
    if (iter != kSensorIDToString.end()) {
        sensor_id_description = iter->second;
    }
    return sensor_id_description;
}

inline SensorID GetSensorID(const std::string& sensor_id_description) {
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    auto iter = kSensorNameToSensorID.find(sensor_id_description);
    if (iter != kSensorNameToSensorID.end()) {
        sensor_id = iter->second;
    }
    return sensor_id;
}

enum class PerceptionSensorType {
    UNKNOWN_SENSOR = -1,
    CAMERA = 0,
    LIDAR = 1,
    RADAR = 2,
    ULTRASONIC = 3
};

// sensor name type description
// TODO(zhangshiquan): remove definition into calibration manager op and init by
// prototxt
static const std::unordered_map<std::string, PerceptionSensorType>
    kSensorNameToPerceptionSensorType{
        // unknow
        {"unknown_sensor", PerceptionSensorType::UNKNOWN_SENSOR},
        // cameras
        {"center_camera_fov120", PerceptionSensorType::CAMERA},
        {"center_camera_fov60", PerceptionSensorType::CAMERA},
        {"center_camera_fov30", PerceptionSensorType::CAMERA},
        {"center_camera_fov20", PerceptionSensorType::CAMERA},
        {"center_camera", PerceptionSensorType::CAMERA},
        {"center_camera_fs", PerceptionSensorType::CAMERA},
        {"left_front_camera", PerceptionSensorType::CAMERA},
        {"left_rear_camera", PerceptionSensorType::CAMERA},
        {"right_front_camera", PerceptionSensorType::CAMERA},
        {"right_rear_camera", PerceptionSensorType::CAMERA},
        {"rear_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras_center_camera_fov120", PerceptionSensorType::CAMERA},
        {"nv_cameras_center_camera_fov60", PerceptionSensorType::CAMERA},
        {"nv_cameras_center_camera_fov30", PerceptionSensorType::CAMERA},
        {"nv_cameras_center_camera_fov20", PerceptionSensorType::CAMERA},
        {"nv_cameras_left_front_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras_left_rear_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras_right_front_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras_right_rear_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras_rear_camera", PerceptionSensorType::CAMERA},
        {"nv_cameras", PerceptionSensorType::CAMERA},
        {"center_camera_fov120_gop", PerceptionSensorType::CAMERA},
        {"center_camera_fov30_gop", PerceptionSensorType::CAMERA},
        {"center_camera_fov30_mono", PerceptionSensorType::CAMERA},
        {"front_camera_fov195", PerceptionSensorType::CAMERA},
        {"left_camera_fov195", PerceptionSensorType::CAMERA},
        {"right_camera_fov195", PerceptionSensorType::CAMERA},
        {"rear_camera_fov195", PerceptionSensorType::CAMERA},
        // lidars
        {"lidar_sensor", PerceptionSensorType::LIDAR},
        {"top_center_lidar", PerceptionSensorType::LIDAR},
        {"front_lidar", PerceptionSensorType::LIDAR},
        // radars
        {"front_radar", PerceptionSensorType::RADAR},
        {"rear_left_radar", PerceptionSensorType::RADAR},
        {"rear_right_radar", PerceptionSensorType::RADAR},
        {"front_left_radar", PerceptionSensorType::RADAR},
        {"front_right_radar", PerceptionSensorType::RADAR},
        {"rear_radar", PerceptionSensorType::RADAR}};

inline bool CheckPerceptionSensorName(const std::string& sensor_name) {
    if (kSensorNameToPerceptionSensorType.find(sensor_name) ==
        kSensorNameToPerceptionSensorType.end()) {
        return false;
    }
    return true;
}

inline bool CheckIsCamera(const std::string& sensor_name) {
    if (!CheckPerceptionSensorName(sensor_name)) {
        return false;
    }
    if (PerceptionSensorType::CAMERA ==
        kSensorNameToPerceptionSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool CheckIsNVCameras(const std::string& sensor_name) {
    if (!CheckPerceptionSensorName(sensor_name)) {
        return false;
    }
    if (PerceptionSensorType::CAMERA ==
            kSensorNameToPerceptionSensorType.at(sensor_name) &&
        SensorID::NV_CAMERAS == kSensorNameToSensorID.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool CheckIsLidar(const std::string& sensor_name) {
    if (!CheckPerceptionSensorName(sensor_name)) {
        return false;
    }
    if (PerceptionSensorType::LIDAR ==
        kSensorNameToPerceptionSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool CheckIsRadar(const std::string& sensor_name) {
    if (!CheckPerceptionSensorName(sensor_name)) {
        return false;
    }
    if (PerceptionSensorType::RADAR ==
        kSensorNameToPerceptionSensorType.at(sensor_name)) {
        return true;
    }
    return false;
}

}  // namespace perception
}  // namespace senseAD
