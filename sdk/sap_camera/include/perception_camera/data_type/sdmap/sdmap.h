/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Wang Shaochang <wangshaochang@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "route_loc.h"

/** 车道引导信息 */
struct sapLaneTrafficInfo {
    /** isBus表示是否是公交专用道 */
    int8_t is_bus = false;

    /** 表示车道类型，由LaneExtensionType中的多个类型组合而成，
     * 每一个bit代表是否是某种车道类型，
     * 参考iface_guidance.h中的LaneExtensionType；*/
    uint32_t lane_type = 0;

    /** arrow代表道路箭头信息，由LaneArrorType中的多个类型组合而成，组合方式同上，
     * 参考iface_guidance.h中的LaneArrowType*/
    uint32_t arrow = 0;
};

/** 道路引导信息 */
struct sapLaneInfoMeta {
    /**  laneNum表示有几条车道包含了引导信息 */
    uint8_t lane_num = 0;

    /** laneInfos对应了这些车道引导信息。 */
    sapLaneTrafficInfo* lane_infos;
    uint64_t lane_infos_num;
};

/** 使用LinkIDType结构来存储，
 * 主要包括tile id，link id，link raw id */
struct sapLinkIDType {
    uint32_t tile_id = 0;
    uint32_t link_id = 0;
    uint64_t raw_id = 0;
};

/** 道路的曲率信息，
 * 其中offset表示距离link起点的偏移量，单位米；
 * curvature表示曲率分级值，不同的数字代表不同曲率范围，符号代表方向，
 * 具体分级参考linkposition.h中的LinkCurvature
 * */
struct sapLinkCurvature {
    /** offset表示距离link起点的偏移量，单位米 */
    uint32_t offset = 0;

    /** curvature表示曲率分级值，不同的数字代表不同曲率范围，符号代表方向，
     * 具体分级参考linkposition.h中的LinkCurvature */
    int32_t curvature = 0;
};

/** Link：sdmap中的一项很重要的元素，是道路抽象出来的对象 */
struct sapLink {
    /** 使用LinkIDType结构来存储，主要包括tile id，link id，link raw id */
    sapLinkIDType link_id;

    /** 当前道路的车道总数量 */
    uint8_t lane_num_sum = 0;

    /** 当前道路的通行方向，例如2表示道路的通行方向是从link坐标的起点到终点，
     * 0表示未知，1表示顺向和逆向，2表示顺向，3表示逆向 */
    uint8_t link_direction = 0;

    /** link起点到终点的速度限制，单位km/h。
     * 0: 没有数据，
     * 1: >130 km/h，
     * 2: (100 km/h, 130 km/h]，
     * 3: (90 km/h, 100 km/h]，
     * 4: (70 km/h, 90 km/h]，
     * 5: (50 km/h, 70 km/h]，
     * 6: (30 km/h, 50 km/h]，
     * 7: [11 km/h, 30 km/h]，
     * 8: <11 km/h。
     * */
    uint8_t speed_limit_s2e = 0;

    /** link终点到起点的速度限制，单位km/h。
     * 0: 没有数据，
     * 1: >130 km/h，
     * 2: (100 km/h, 130 km/h]，
     * 3: (90 km/h, 100 km/h]，
     * 4: (70 km/h, 90 km/h]，
     * 5: (50 km/h, 70 km/h]，
     * 6: (30 km/h, 50 km/h]，
     * 7: [11 km/h, 30 km/h]，
     * 8: <11 km/h。*/
    uint8_t speed_limit_e2s = 0;

    /** 速度等级。 目前没有用处。 */
    uint8_t speed_class_type = 0;

    /** link的起点角度，取值为[0,360)，以正北方向顺时针计算 */
    int16_t start_angle = 0;

    /** link的终点角度，取值为[0,360)，以正北方向顺时针计算 */
    int16_t end_angle = 0;

    /** 道路的类别，未知。 */
    int8_t road_class_type = -1;

    /** 是个数组表示道路可能有多重属性，
     * 具体属性参考iface_lanegroup.h中的RoadKindType */
    uint64_t* road_kind_types;
    /** road_kind_types个数 */
    uint64_t road_kind_types_num;

    /** 道路几何形状，使用一串坐标点表示 */
    sapPointLLH* geometry;
    uint64_t geometry_num;
    sapPointENU* geometry_lcal;
    uint64_t geometry_lcal_num;

    /** 道路的曲率信息，其中offset表示距离link起点的偏移量，单位米；
     * curvature表示曲率分级值，不同的数字代表不同曲率范围，符号代表方向，
     * 具体分级参考linkposition.h中的LinkCurvature
     * */
    sapLinkCurvature* link_curvature;
    uint64_t link_curvature_num;

    /** 道路引导信息，
     * laneNum表示有几条车道包含了引导信息，
     * laneInfos对应了这些车道引导信息。
     * 引导信息中的isBus表示是否是公交专用道；
     * laneType表示车道类型，由LaneExtensionType中的多个类型组合而成，每一个bit代表是否是某种车道类型，
     * 参考iface_guidance.h中的LaneExtensionType；
     * arrow代表道路箭头信息，由LaneArrorType中的多个类型组合而成，组合方式同上，
     * 参考iface_guidance.h中的LaneArrowType
     * */
    sapLaneInfoMeta* guidance_infos;
    uint64_t guidance_infos_num;

    /** 后继link id列表 */
    sapLinkIDType* next_link_ids;
    uint64_t next_link_ids_num;

    /** 前驱link id列表 */
    sapLinkIDType* previous_link_ids;
    uint64_t previous_link_ids_num;
};

struct sapNodeIDType {
    uint32_t tile_id = 0;
    uint32_t node_id = 0;
};

/** sdmap中的另外一项重要元素，是link的起点或终点 */
struct sapLinkNode {
    /** 使用NodeIDType结构来存储，
     * 主要包括tile id，node id */
    sapNodeIDType node_id;

    /** node的坐标*/
    sapPointLLH position;
    sapPointENU position_local;

    /** node的类型，
     * 0表示非路口节点，
     * 1表示复合路口子节点，
     * 2表示单一路口节点，
     * 3表示复合路口主节点
     * */
    uint8_t crossFlag = 0;

    /** 是否有红绿灯，
     * true表示有，
     * false表示没有
     * */
    int8_t traffic_light_flag = false;

    /** 进入该节点的所有link id */
    sapLinkIDType* enter_link_ids;
    uint64_t enter_link_ids_num;

    /** 离开该节点的所有link id */
    sapLinkIDType* out_link_ids;
    uint64_t out_link_ids_num;
};

typedef struct _sapSDMap {
    /** 时间戳 */
    uint64_t timestamp_ns;
    /** sdmap中所包含的跟导航路径相关的所有link */
    sapLink* links;
    uint64_t links_num;
    /** sdmap中所包含的跟导航路径相关的所有起始link node */
    sapLinkNode* start_link_nodes;
    uint64_t start_link_nodes_num;
    /** sdmap中所包含的跟导航路径相关的所有终止link node */
    sapLinkNode* end_link_nodes;
    uint64_t end_link_nodes_num;
    /** 上游发送的导航路径，主要包括link raw id和坐标 */
    sapNavRouteInfo nav_route_info;
    sapLocalizationInfo loc_info;
} sapSDMap;