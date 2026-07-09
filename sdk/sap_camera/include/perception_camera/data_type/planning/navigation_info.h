/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <unordered_map>
#include <functional>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t nanoSec;
} sapHeaderTime;

typedef struct {
    uint64_t seq;
    sapHeaderTime time;
    char frameId[30];
} sapStdHeader;

//--- 电子限速信息 ---
typedef struct {
    uint64_t electronicSpeedLimitType;  // 电子限速类型
    uint64_t speedLimitValue;           // 限速值
    uint64_t electronicSpeedLimitDist;  // 距离[m]
} sapAmapElectronicSpeedLimitInfo;

//--- 天气信息 ---
typedef struct {
    char weatherName[30];     // 天气名称
    char alertLevelName[30];  // 预警名称
    uint64_t weatherId;           // 天气ID
    uint64_t alertLevelId;        // 警告等级
    uint64_t distanceToCar;       // 自车到路段距离
} sapAmapWeatherInfo;

//--- 交通事件信息 ---
typedef struct {
    uint64_t trafficEventType;      // 事件类型
    uint64_t occupiedLaneType;      // 占用车道
    uint64_t trafficEventDistance;  // 距离[m]
} sapAmapTrafficEventInfo;

//--- 收费站信息 ---
typedef struct {
    uint64_t* laneTypes;  // 收费站车道类型列表
    uint64_t laneNums;      // 收费站车道数量
    uint64_t distance;                // 距离[m]
} sapAmapTollGateInfo;

//--- 摄像头限速信息 ---
typedef struct {
    uint64_t cameraDistance;  // 距离[m]
    uint64_t cameraSpeed;     // 限速[km/h]
    uint64_t cameraType;       // 摄像头种类
} sapAmapCameraInfo;

typedef struct {
    uint64_t startTime;          // 状态开始时间
    uint64_t endTime;            // 状态结束时间
    uint64_t trafficLightState;  // 红绿灯灯态 enum TrafficLightState
} sapAmapTrafficLight;

typedef struct {
    sapAmapTrafficLight* trafficLightInfo;  // 红绿灯信息列表
    uint64_t trafficlightNums;  // 红绿灯数量
    uint64_t length;     // 红绿灯位置，从link起点偏移量
    uint64_t linkIndex;  // 导航端index
    uint64_t linkId;     // link id
} sapAmapTrafficLightInfo;

//--- 车道信息 ---
typedef struct {
    uint64_t* foregroundLane;  // 前景车道
    uint64_t foregroundLaneNums;  // 前景车道数量
    uint64_t* backgroundLane;  // 背景车道
    uint64_t backgroundLaneNums;  // 背景车道数量
} sapAmapLaneInfo;

//--- 道路设施信息 ---
typedef struct {
    uint64_t trafficFacilityDistance;  // 距离[m]
    uint64_t trafficFacilitySpeed;     // 限速[km/h]
    uint64_t trafficFacilityType;       // 设施类型
} sapAmapTrafficFacilityInfo;

//--- 交通状态（拥堵） ---
typedef struct {
    double length;                  // 拥堵路段长度
    uint64_t startLinkIdx;           // 起始 Link ID
    uint64_t endLinkIndex;           // 结束 Link ID
    uint64_t startSegmentIdx;        // 起始 Seg 索引
    uint64_t endSegmentIdx;          // 结束 Seg 索引
    uint64_t trafficState;  // 拥堵等级
} sapAmapTrafficStatus;

//--- 主结构：导航信息 ---
typedef struct {
    sapStdHeader header;          // 消息头
    bool is_valid;               // 是否有效，planning sdk内部计算
    bool isOutOfRoute;            // 是否偏航
    uint64_t naviType;             // 导航模式
    uint64_t iconType;             // 推荐动作
    uint64_t curRetainDistance;    // 当前动作距离[m]
    uint64_t allLength;            // 全程长度[m]
    uint64_t pathRetainDistance;   // 剩余距离[m]
    sapAmapLaneInfo laneInfo;     // 车道信息
    uint64_t curSpeedLimit;        // 当前限速[km/h]
    uint64_t curLink;              // 当前 Link ID
    uint64_t curStep;              // 当前 Step ID
    uint64_t distanceToNextLink;   // 离下个 Link 距离[m]
    char curRoadNameNavi[30];     // 当前道路名称
    uint64_t curRoadLevelForNavi;  // 道路等级
    sapAmapCameraInfo* cameraInfo;  // 摄像头信息列表
    uint64_t cameraInfoNums;  // 摄像头数量
    sapAmapTrafficFacilityInfo*
        trafficFacilityInfo;  // 道路设施列表
    uint64_t trafficFacilityInfoNums;  // 道路设施数量
    sapAmapTrafficLightInfo* trafficLightInfo;  // 红绿灯列表
    uint64_t trafficLightInfoNums;  // 红绿灯数量
    uint64_t entranceExit;                                   // 匝道类型
    char naviText[30];                                      // TTS 播报文本
    sapAmapTrafficStatus trafficInfo;                            // 交通拥堵状态
    uint64_t elevatedRoadStatus;                     // 是否高架
    uint64_t parallelRoadStatus;                     // 主辅路状态
    sapAmapTollGateInfo* tollGateInfo;  // 收费站列表
    uint64_t tollGateInfoNums;  // 收费站数量
    sapAmapElectronicSpeedLimitInfo* electronicSpeedLimitInfo;   // 电子限速列表
    uint64_t electronicSpeedLimitInfoNums;  // 电子限速数量
    sapAmapWeatherInfo* weatherInfo;  // 天气信息列表
    uint64_t weatherInfoNums;  // 天气信息数量
    sapAmapTrafficEventInfo* trafficEventInfo;  // 交通事件列表
    uint64_t trafficEventInfoNums;  // 交通事件数量
} sapAmapNaviInfo;

#ifdef __cplusplus
}
#endif