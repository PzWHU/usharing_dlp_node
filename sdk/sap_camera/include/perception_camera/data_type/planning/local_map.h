/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CrossInfo
 */
typedef struct {
    uint64_t line_type;
    uint64_t line_style;
    uint64_t color;
    float width;
    sapPoint3f* points;
    uint64_t points_num;
} sapLineSegmentData;

typedef struct {
    uint64_t id;
    uint64_t left_lane_id;
    uint64_t right_lane_id;
    uint64_t predecessor_laneline_id;
    uint64_t successor_laneline_id;

    // for saving online_map curbline info
    uint64_t curb_father_line_id;

    sapLineSegmentData* line_segments;
    uint64_t line_segments_num;

} sapLineData;

typedef struct {
    sapPoint3f* points;
    uint64_t points_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapLineSegObjData;

typedef sapLineSegObjData sapStopLineData;

typedef struct {
    uint64_t area_type;
    sapPoint3f* boundary;
    uint64_t boundary_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapAreaData;

typedef struct {
    uint64_t id;
    uint64_t roadmarker_type;
    uint64_t color;
    char semantic[30];
    sapPoint3f centroid;
    uint64_t lane_id;
} sapRoadMarkerData;

typedef struct {
    sapLineData* lines;
    uint64_t lines_num;
    sapStopLineData* stop_lines;
    uint64_t stop_lines_num;
    sapAreaData* areas;
    uint64_t areas_num;
    sapRoadMarkerData* road_markers;
    uint64_t road_markers_num;
} sapSemanticMapData;

typedef struct {
    uint64_t id;
    uint64_t section_class;
    uint64_t section_type;
    uint64_t intersection_id;
    float length;
    // 从左至右排序，存储的lane id
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
    // road edge and laneline
    // 左右侧边界，有可能是多个，例如由车道线和路边沿共同组成
    uint64_t* left_boundary_ids;
    uint64_t left_boundary_ids_num;
    uint64_t* right_boundary_ids;
    uint64_t right_boundary_ids_num;
    uint64_t* successor_link_ids;
    uint64_t successor_link_ids_num;
    uint64_t* predecessor_link_ids;
    uint64_t predecessor_link_ids_num;
    bool is_routing_section;
} sapSectionData;

typedef struct {
    // 区间限速值，广汽接口单位千米/时，SM内部单位m/s
    double max_speed;
    double min_speed;

    // 0: Unknown;
    // 1: Persent;
    // 2: Index;
    // default = 0
    uint8_t offset_style;
    double start_offset;
    double end_offset;
} sapSpeedIntervalInfo;

typedef struct {
    uint64_t id;
    uint64_t section_id;
    uint64_t sequence_id;
    uint64_t intersection_id;
    uint64_t lane_type;

    // 16进制表示 按位组合
    uint64_t turn_type;
    uint64_t trans_type;

    uint64_t left_line_id;
    uint64_t right_line_id;
    uint64_t left_lane_id;
    uint64_t right_lane_id;
    float length;

    sapPoint3f* center_points;
    uint64_t center_points_num;

    // all these vector's size = center_points.size();
    float* lane_widths;
    uint64_t lane_widths_num;
    float* headings;
    uint64_t headings_num;
    float* longitudinal_slopes;  // 纵向坡度
    uint64_t longitudinal_slopes_num;
    float* lateral_slopes;  // 横向坡度
    uint64_t lateral_slopes_num;
    float* curvature_radius;
    uint64_t curvature_radius_num;

    // 车道区间限速，每个区间段具有最大和最小限速，广汽接口只有最大限速
    sapSpeedIntervalInfo* speed_limit_vector;
    uint64_t speed_limit_vector_num;

    // 旧版限速接口 后期弃用
    double max_speed;  // m/s
    double min_speed;

    uint64_t* successor_link_ids;
    uint64_t successor_link_ids_num;
    uint64_t* predecessor_link_ids;
    uint64_t predecessor_link_ids_num;

    // objects
    uint64_t* traffic_light_ids;
    uint64_t traffic_light_ids_num;
    uint64_t* traffic_sign_ids;
    uint64_t traffic_sign_ids_num;
    uint64_t* road_marker_ids;
    uint64_t road_marker_ids_num;
    uint64_t* stop_line_ids;
    uint64_t stop_line_ids_num;
    uint64_t* cross_walk_ids;
    uint64_t cross_walk_ids_num;
    uint64_t* speed_bump_ids;
    uint64_t speed_bump_ids_num;
    uint64_t* area_ids;
    uint64_t area_ids_num;
} sapLaneData;

typedef struct {
    uint64_t id;
    uint64_t from_lane_id;
    uint64_t to_lane_id;
    uint64_t link_type;
    sapPoint3f* ref_points;
    uint64_t ref_points_num;
} sapLaneLinkData;

typedef struct {
    uint64_t id;
    uint64_t from_section_id;
    uint64_t to_section_id;
    uint64_t link_type;
} sapSectionLinkData;

typedef struct {
    sapLaneData* lanes;
    uint64_t lanes_num;
    sapSectionData* sections;
    uint64_t sections_num;
    sapLaneLinkData* lane_links;
    uint64_t lane_links_num;
    sapSectionLinkData* section_links;
    uint64_t section_links_num;
} sapRouteMapData;

typedef struct {
    uint64_t timestamp_ns;
    uint64_t cur_status;
    sapSemanticMapData semantic_map_data;
    sapRouteMapData route_map_data;
} sapRoadStructure;

#ifdef __cplusplus
}
#endif