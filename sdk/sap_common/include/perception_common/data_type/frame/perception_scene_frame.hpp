/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <set>
#include <memory>
#include <vector>
#include <string>
#include <map>

#include "perception_common/data_type/sensor_info.hpp"

namespace senseAD {
namespace perception {

/* enum class SceneWeatherInfo {
    UNKNOWN = 0,
    SUNNY,
    CLOUDY,
    RAINY,
    SNOWY,
    FOGGY,
    SANDY,
    OVERCAST,
    OTHERS
}; */
enum class SceneWeatherInfo {
    UNKNOWN = 0,
    OVERCAST,
    SUNNY,
    RAINY,
    FOGGY,
    SNOWY,
    SANDY,
    OTHERS
};

enum class RoadIntersection {
    UNKNOWN = -1,
    T_SHAPE,
    Y_SHAPE,
    CROSS_SHAPE,
    X_SHAPE,
    ROUND_SHAPE
};

enum class SceneBrightnessInfo {
    UNKNOWN = 0,
    DAYTIME,
    NIGHT_BRIGHT,
    NIGHT_DARK
};

enum class SceneEnvironmentInfo {
    UNKNOWN = 0,
    OPEN_ROAD,
    CLOSED_ROAD,
    TUNNEL
};

static const std::map<SceneWeatherInfo, std::string>
    sceneweathertostring{
    {SceneWeatherInfo::UNKNOWN, "unknown"},
    {SceneWeatherInfo::SUNNY, "sunny"},
    // {SceneWeatherInfo::CLOUDY, "cloudy"},
    {SceneWeatherInfo::RAINY, "rainy"},
    {SceneWeatherInfo::SNOWY, "snowy"},
    {SceneWeatherInfo::FOGGY, "foggy"},
    {SceneWeatherInfo::SANDY, "sandy"},
    {SceneWeatherInfo::OVERCAST, "overcast"},
    {SceneWeatherInfo::OTHERS, "others"}
};
static const std::map<SceneBrightnessInfo, std::string>
    scenebrighttostring{
    {SceneBrightnessInfo::UNKNOWN, "unknown"},
    {SceneBrightnessInfo::DAYTIME, "daytime"},
    {SceneBrightnessInfo::NIGHT_BRIGHT, "night_bright"},
    {SceneBrightnessInfo::NIGHT_DARK, "night_dark"}
};
static const std::map<SceneEnvironmentInfo, std::string>
    sceneenvironmenttostring{
    {SceneEnvironmentInfo::UNKNOWN, "unknown"},
    {SceneEnvironmentInfo::OPEN_ROAD, "open_road"},
    {SceneEnvironmentInfo::CLOSED_ROAD, "closed_road"},
    {SceneEnvironmentInfo::TUNNEL, "tunnel"}
};

enum class RoadSlop { UNKNOWN = -1, SMOOTH, BUMPS, GENTLE };
enum class RoadQuality { UNKNOWN = -1, LEVEL_A, LEVEL_B, LEVEL_C };

struct RoadSceneInfo {
    RoadIntersection road_intersection = RoadIntersection::UNKNOWN;
    RoadSlop road_slop = RoadSlop::UNKNOWN;
    RoadQuality road_qulity = RoadQuality::UNKNOWN;
};

typedef struct SceneInfoFrame {
    uint64_t timestamp_ns = 0;
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    SceneWeatherInfo scene_weather;
    SceneBrightnessInfo scene_brightness;
    SceneEnvironmentInfo scene_environment;
    std::vector<RoadSceneInfo> road_scenes;
} PerceptionSceneFrame;

}  // namespace perception
}  // namespace senseAD
