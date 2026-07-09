/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhangshiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#include <mutex>
#include <memory>
#include <map>
#include <vector>
#include <string>

#include "Eigen/Core"
#include "Eigen/Dense"

#include <opencv2/opencv.hpp>
#include "perception_common/base/data_type/object_label.hpp"
#include "perception_common/base/data_type/geometry.hpp"
#include "perception_common/log/error.hpp"
// #include "perception_common/base/class_helper.hpp"

namespace senseAD {
namespace perception {

enum class TrafficSignal {
    NONE = 0x00,
    GREEN = 0x02,
    YELLOW = 0x04,
    RED = 0x08,
    GREEN_FLASH = 0x10,
    YELLOW_FLASH = 0x20,
    BLACK = 0x40,
    GREEN_NUMBER = 0x80
};

enum class TrafficLightStatus {
    BLACK = 0,
    GREEN = 1,
    YELLOW = 2,
    RED = 3,
    NONE = 255
};

using TrafficSignalTable_t = std::map<TrafficSignal, std::string>;
static const TrafficSignalTable_t TrafficSignalTable{
    {TrafficSignal::NONE, "NONE"},
    {TrafficSignal::GREEN, "GREEN"},
    {TrafficSignal::YELLOW, "YELLOW"},
    {TrafficSignal::RED, "RED"},
    {TrafficSignal::GREEN_FLASH, "GREEN_FLASH"},
    {TrafficSignal::YELLOW_FLASH, "YELLOW_FLASH"},
    {TrafficSignal::BLACK, "BLACK"},
    {TrafficSignal::GREEN_NUMBER, "GREEN_NUMBER"}};

using TrafficLightStatusTable_t = std::map<TrafficLightStatus, std::string>;
static const TrafficLightStatusTable_t TrafficLightStatusTable{
    {TrafficLightStatus::NONE, "NONE"},
    {TrafficLightStatus::GREEN, "GREEN"},
    {TrafficLightStatus::YELLOW, "YELLOW"},
    {TrafficLightStatus::RED, "RED"},
    {TrafficLightStatus::BLACK, "BLACK"}};

enum class TurnType {
    FREE = 0xffff,
    TURN_LEFT = 0x0001,     // 0000 0001 determine turn left
    TURN_RIGHT = 0x0002,    // 0000 0010  determine turn right
    LEFT_AROUND = 0x0004,   // 0000 0100 determine turn left around
    RIGHT_AROUND = 0x0008,  // 0000 1000 determine turn right around
    STRAIGHT = 0x0010       // 0001 0000 determine go straight
};

enum class GeneralTurnType {
    UNDEFINE = 0,
    NONE = 1,
    CIRCLE = 2,
    PEDESTRIAN = 3,
    TURN_LEFT = 4,
    STRAIGHT = 5,
    TURN_RIGHT = 6,
    TURN_AROUND = 7,
    TURN_LEFT_STRAIGHT = 8,
    TURN_LEFT_AROUND = 9,
    TURN_RIGHT_STRAIGHT = 10,
    BIKE = 11,
    X_ENTER = 12,
    NUMBER = 13,
    ENTER = 14
};

using TurnTypeTable_t = std::map<TurnType, std::string>;
static const TurnTypeTable_t TurnTypeTable{
    {TurnType::FREE, "NONE"},
    {TurnType::TURN_LEFT, "TURN_LEFT"},
    {TurnType::TURN_RIGHT, "TURN_RIGHT"},
    {TurnType::LEFT_AROUND, "LEFT_AROUND"},
    {TurnType::RIGHT_AROUND, "RIGHT_AROUND"},
    {TurnType::STRAIGHT, "STRAIGHT"}};

using GeneralTurnTypeTable_t = std::map<GeneralTurnType, std::string>;
static const GeneralTurnTypeTable_t GeneralTurnTypeTable{
    {GeneralTurnType::NONE, "NONE"},
    {GeneralTurnType::CIRCLE, "CIRCLE"},
    {GeneralTurnType::PEDESTRIAN, "PEDESTRIAN"},
    {GeneralTurnType::TURN_LEFT, "TURN_LEFT"},
    {GeneralTurnType::STRAIGHT, "STRAIGHT"},
    {GeneralTurnType::TURN_RIGHT, "TURN_RIGHT"},
    {GeneralTurnType::TURN_AROUND, "TURN_AROUND"},
    {GeneralTurnType::TURN_LEFT_STRAIGHT, "TURN_LEFT_STRAIGHT"},
    {GeneralTurnType::TURN_LEFT_AROUND, "TURN_LEFT_AROUND"},
    {GeneralTurnType::TURN_RIGHT_STRAIGHT, "TURN_RIGHT_STRAIGHT"},
    {GeneralTurnType::BIKE, "BIKE"},
    {GeneralTurnType::X_ENTER, "X_ENTER"},
    {GeneralTurnType::NUMBER, "NUMBER"},
    {GeneralTurnType::ENTER, "ENTER"}};

// @brief for hdmap query, as the input for camera perception
typedef struct StructMapTrafficLightInfo {
    std::string tl_id;
    std::vector<senseAD::perception::Vec3f> geometry;
} MapTrafficLightInfo;

// @brief for hdmap query, as the input for camera perception
typedef struct StructMapGroupTrafficLightInfo {
    std::string group_id;  // group_traffic_light_xx
    std::vector<std::string> traffic_light_ids;
    std::vector<MapTrafficLightInfo> traffic_lights;
    uint16_t turn_type;
} MapGroupTrafficLightInfo;

// @brief for hdmap query, as the input for camera perception
typedef struct StructMapTrafficLightFrame {
    uint64_t timestamp_ns;
    std::vector<MapGroupTrafficLightInfo> traffic_light_groups;
} MapTrafficLightFrame;

}  // namespace perception
}  // namespace senseAD
