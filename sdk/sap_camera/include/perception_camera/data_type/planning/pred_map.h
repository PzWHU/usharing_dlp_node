/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * pan wenbo <panwenbo@sensetime.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t line_type;
    uint64_t line_style;
    uint64_t color;
    float width;
    sapPoint3f* points;
    uint64_t points_num;
} sapFusionLineSegmentData;

typedef struct {
    uint64_t id;
    uint64_t left_lane_id;
    uint64_t right_lane_id;
    uint64_t predecessor_laneline_id;
    uint64_t successor_laneline_id;

    // for saving online_map curbline info
    uint64_t curb_father_line_id;

    sapFusionLineSegmentData* line_segments;
    uint64_t line_segments_num;

} sapFusionLineData;

typedef struct {
    sapPoint3f* points;
    uint64_t points_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapFusionLineSegObjData;

typedef sapFusionLineSegObjData sapFusionStopLineData;
typedef struct {
    int32_t polyline_start_index = 0;
    int32_t polyline_end_index = 0;
    uint64_t boundary_type;

    uint64_t line_style;
    uint64_t color;
    // ID of BoundarySegment
    int64_t boundary_feature_id = -1;
} sapFusionBoundarySegmentData;

typedef struct {
    int64_t id;
    sapPoint3f* polyline_points;
    uint64_t polyline_points_num;
    sapFusionBoundarySegmentData* segments;
    uint64_t segments_num;
} sapFusionBoundaryData;

typedef struct {
    int64_t id = -1;
    // uint64_t section_class;
    uint64_t section_type;
    uint64_t intersection_id = 0;
    uint64_t junction_id = 0;
    float length;
    // 从左至右排序，存储的lane id
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
    // road edge and laneline
    // 左右侧边界，有可能是多个，例如由车道线和路边沿共同组成
    sapFusionBoundaryData* left_boundarys;
    uint64_t left_boundarys_num;
    sapFusionBoundaryData* right_boundarys;
    uint64_t right_boundarys_num;

    uint64_t* entry_section_ids;
    uint64_t entry_section_ids_num;
    uint64_t* exit_section_ids;
    uint64_t exit_section_ids_num;
    uint64_t* successor_link_ids;
    uint64_t successor_link_ids_num;
    uint64_t* predecessor_link_ids;
    uint64_t predecessor_link_ids_num;
    bool is_routing_section;
} sapFusionSectionData;

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
} sapFusionSpeedIntervalInfo;

typedef struct {
    double qx;
    double qy;
    double qz;
    double qw;
} sapFusionQuaternion_t;

typedef struct {
    int64_t lane_id = -1;
    int32_t overlap_line_start_index = 0;
    int32_t overlap_line_end_index = 0;
    sapFusionBoundarySegmentData* segments;
    uint64_t segments_num;
} sapFusionLaneNeighborData;
typedef struct {
    float32_t max_kph = 0.0f;
    float32_t min_kph = 0.0f;
} sapFusionSpeedLimitData;
typedef struct {
    int64_t id = -1;
    int64_t section_id = -1;
    // uint64_t sequence_id;
    // uint64_t intersection_id;
    uint64_t lane_type;

    // 16进制表示 按位组合
    uint64_t road_type;
    uint64_t turn_type;
    uint64_t trans_type;

    sapPoint3f* center_line_points;
    uint64_t center_line_points_num;

    // uint64_t left_line_id;
    // uint64_t right_line_id;
    // uint64_t left_lane_id;
    // uint64_t right_lane_id;
    float32_t length = 0.0f;
    sapFusionSpeedLimitData speed_limit;
    int64_t* entry_lane_ids;
    uint64_t entry_lane_ids_num;
    int64_t* exit_lane_ids;
    uint64_t exit_lane_ids_num;

    sapFusionBoundaryData left_boundary;
    sapFusionBoundaryData right_boundary;

    sapFusionLaneNeighborData* left_neighbors;
    uint64_t left_neighbors_num;
    sapFusionLaneNeighborData* right_neighbors;
    uint64_t right_neighbors_num;

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
    sapFusionSpeedIntervalInfo* speed_limit_vector;
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
} sapFusionLaneData;

typedef struct {
    int64_t id = 0;
    int64_t from_lane_id = 0;
    int64_t to_lane_id = 0;
    uint64_t link_type;
    sapPoint3f* ref_points;
    uint64_t ref_points_num;
} sapFusionLaneLinkData;

typedef struct {
    uint64_t id;
    uint64_t road_line_type;
    sapPoint3f* road_line_points;
    uint64_t road_line_points_num;
} sapFusionRoadLineData;

typedef struct {
    uint64_t id;
    uint64_t line_id;
    sapPoint3f* road_edge_line_points;
    uint64_t road_edge_line_points_num;
    uint64_t road_edge_line_type;
} sapFusionRoadEdgeData;

typedef struct {
    uint64_t id;
    sapPoint3f* cross_walk_line_points;
    uint64_t cross_walk_line_points_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
    uint64_t junction_id;
} sapFusionCrossWalkData;

typedef struct {
    uint64_t id;
    uint64_t traffic_signal_state_type;
    sapPoint3f* stop_line_points;
    uint64_t stop_line_points_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapFusionTrafficSignalData;

typedef struct {
    uint64_t id;
    sapPoint3f position;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapFusionStopSignData;

typedef struct {
    uint64_t id;
    sapPoint3f* polygon_points;
    uint64_t polygon_points_num;
} sapFusionSafeIslandData;

typedef struct {
    uint64_t shape;
    uint64_t status;
} sapFusionTrafficLightSignalData;

typedef struct {
    uint64_t id;
    sapPoint3f* polygon_points;
    uint64_t polygon_points_num;
} sapFusionSpeedBumpData;

typedef struct {
    uint64_t id;
    sapPoint3f centroid;
    sapFusionQuaternion_t quaternion;
    float length;
    float width;
    uint64_t detection_status;
    float confidence;  // 置信度 范围[0, 1]
    uint64_t lane_id;  // 这个Lane id 代表的含义是，需确认lane id的作用
    uint64_t countdown;  // 倒计时读秒 [-1, 99]
    sapFusionTrafficLightSignalData* signals;
    uint64_t signals_num;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
} sapFusionTrafficLightData;

typedef struct {
    uint64_t id;
    uint64_t* lane_ids;
    uint64_t lane_ids_num;
    sapPoint3f* points;
    uint64_t points_num;
    uint64_t junction_id;
} sapFusionStopLineLDData;

typedef struct {
    uint64_t* connected_ids;  //  // 路口一个lane/link连接序列的所有车道id
    uint64_t connected_ids_num;
    uint64_t level;
    uint64_t contract_point_type;
    uint64_t connect_type;
    uint64_t incoming_id;      // 进路口的lane/section id
    uint64_t outgoing_id;      // 出路口的lane/section id
    sapPoint3f incoming_node;  // 进路口lane/section的末尾点
    sapPoint3f outgoing_node;  // 出路口的lane/section的起始点
} sapFusionConnectionData;

typedef struct {
    uint64_t id;
    uint64_t junction_type;
    sapFusionConnectionData* lane_connections;
    uint64_t lane_connections_num;
    sapFusionConnectionData* section_connections;
    uint64_t section_connections_num;
    sapPoint3f* polygon_points;
    uint64_t polygon_points_num;
} sapFusionJunctionData;

typedef struct {
    uint64_t id;
    uint64_t from_section_id;
    uint64_t to_section_id;
    uint64_t link_type;
} sapFusionSectionLinkData;

typedef struct {
    uint64_t timestamp_ns;
    sapFusionLaneData* lanes;
    uint64_t lanes_num;
    sapFusionSectionData* sections;
    uint64_t sections_num;
    sapFusionLaneLinkData* lane_links;
    uint64_t lane_links_num;
    sapFusionSectionLinkData* section_links;
    uint64_t section_links_num;
    sapFusionRoadLineData* road_lines;
    uint64_t road_lines_num;
    sapFusionRoadEdgeData* road_edges;
    uint64_t road_edges_num;
    sapFusionCrossWalkData* cross_walks;
    uint64_t cross_walks_num;
    sapFusionTrafficSignalData* traffic_signals;
    uint64_t traffic_signals_num;
    sapFusionStopSignData* stop_signs;
    uint64_t stop_signs_num;
    sapFusionSafeIslandData* safe_islands;
    uint64_t safe_islands_num;
    sapFusionSpeedBumpData* speed_bumps;
    uint64_t speed_bumps_num;
    sapFusionTrafficLightData* traffic_lights;
    uint64_t traffic_lights_num;
    sapFusionStopLineLDData* stop_lines;
    uint64_t stop_lines_num;
    sapFusionJunctionData* junctions;
    uint64_t junctions_num;
} sapPredMap;

typedef std::shared_ptr<sapPredMap> sapPredMapPtr;

#ifdef __cplusplus
}
#endif