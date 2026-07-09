/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdlib.h>
#include <string.h>

#include "perception_camera/data_type/object/base_data_struct.h"
#include "perception_camera/data_type/object/base_object.h"
#include "perception_camera/data_type/base_type.h"
#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_data_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAME_STR_MAX_LEN 30
/**
 * @brief 单帧路面道路拓扑SDMap数据
 */
typedef struct _sapPerceptionRoadSDMapFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 传感器名称 */
    char sensor_name[NAME_STR_MAX_LEN];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 路面信息 */
    sapSDMap sdmap;
    /** 是否为新数据 */
    bool new_data = true;
} sapSDMapOriginFrame;

/**
 * @brief 车道线分类类别
 *
 */
typedef enum {
    /** 未知类别 */
    NO_CATEGORY = -1,
    /** 白色虚线 */
    WHITE_DASHED_LANE,
    /** 白色实线 */
    WHITE_SOLID_LANE,
    /** 双白色虚线 */
    DOUBLE_WHITE_DASHED_LANE,
    /** 双白色实线 */
    DOUBLE_WHITE_SOLID_LANE,
    /** 白色虚线+实线 */
    WHITE_DASHED_SOLID_LANE,
    /** 白色实线+虚线 */
    WHITE_SOLID_DASHED_LANE,
    /** 黄色虚线 */
    YELLOW_DASHED_LANE,
    /** 黄色实线 */
    YELLOW_SOLID_LANE,
    /** 双黄色虚线 */
    DOUBLE_YELLOW_DASHED_LANE,
    /** 双黄色实线 */
    DOUBLE_YELLOW_SOLID_LANE,
    /** 黄色虚线+实线 */
    YELLOW_DASHED_SOLID_LANE,
    /** 黄色实线+虚线 */
    YELLOW_SOLID_DASHED_LANE,
    /** 锯齿实心车道 */
    SERRATED_SOLID_LANE,
    /** 左路沿车道线 */
    LEFT_ROADSIDE_LANE,
    /** 右路沿车道线 */
    RIGHT_ROADSIDE_LANE,
    /** 车道线类型总数 */
    NUM_LANES_CATEGORY_MAX
} sapLaneCategory;
/**
 * @brief 车道线颜色
 **/
typedef enum {
    /** 未知颜色 */
    NO_COLOR_LINE = 0,
    /** 其它颜色 */
    OTHER_COLOR,
    /** 白色 */
    WHITE_COLOR,
    /** 黄色 */
    YELLOW_COLOR,
    /** 橙色 */
    ORANGE_COLOR,
    /** 蓝色 */
    BLUE_COLOR,
    /** 颜色数量总数 */
    NUM_COLORTYPES_MAX
} sapColorType;

/**
 * @brief 车道线的线条类型
 */
typedef enum {
    /** 未知类型 */
    NO_LINE_LINE = 0,
    /** 其他类型 */
    OTHER_LINE,
    /** 虚线 */
    DASHED_LINE,
    /** 实线 */
    SOLID_LINE,
    /** 双虚线 */
    DOUBLE_DASHED_LINE,
    /** 双实线 */
    DOUBLE_SOLID_LINE,
    /** 虚线+实线 */
    DASHED_SOLID_LINE,
    /** 实线+虚线 */
    SOLID_DASHED_LINE,
    /** 三虚线 */
    TRIPLE_DASHED_LINE,
    /** 虚线+实线+虚线 */
    DASHED_SOLID_DASHED_LINE,
    /** 实线过渡到虚线 */
    SOLID_TO_DASHED,
    /** 虚线过度到实线 */
    DASHED_TO_SOLID,
    /** 线型数量 */
    NUM_LINETYPES_MAX
} sapLineType;

/**
 * @brief 路沿类型
 */
typedef enum {
    /** 无路沿线 */
    NO_EDGE_LINE = 0,
    /** 路边沿 */
    ROAD_CURB_EDGE,
    /** 栅栏边缘 */
    FENCE_EDGE,
    /** 墙体或绿植边缘 */
    WALL_EDGE_OR_PLANT_EDGE,
    /** 可移动边缘 */
    MOVABLE_EDGE,
    /** 墙体边缘 */
    WALL_EDGE,
    /** 沟渠边缘 */
    DITCH_OR_PLANE,
    /** 其它路沿 */
    OTHER_EDGE,
    /** 路沿类型数量 */
    NUM_EDGETYPES_MAX
} sapEdgeType;

/**
 * @brief 车道线类型
 */
typedef enum {
    /** 未知类型 */
    LANE_LINE_UNKNOWN = 0,
    /** 单虚线 */
    LANE_LINE_SINGLE_DASHED,
    /** 单实线 */
    LANE_LINE_SINGLE_SOLID,
    /** 双虚线 */
    LANE_LINE_DOUBLE_DASHED,
    /** 双实线 */
    LANE_LINE_DOUBLE_SOLID,
    /** 左虚右实线 */
    LANE_LINE_LEFT_DASHED_RIGHT_SOLID,
    /** 左实右虚线 */
    LANE_LINE_LEFT_SOLID_RIGHT_DASHED,
    /** 路沿线 */
    LANE_LINE_ROAD_EDGE,
    /** 短粗虚线 */
    LANE_LINE_SHORT_THICK_DASHED
} sapLaneLineType;

/**
 * @brief 车道线宽度
 *
 */
typedef struct _sapLineWidth {
    /** 车道线宽度，单位cm */
    uint8_t width;
    /** 左车道线宽度，单位cm */
    uint8_t width_left;
    /** 右车道线宽度，单位cm */
    uint8_t width_right;
} sapLineWidth;

/**
 * @brief 合并或分离点信息
 */
typedef struct _sapMergeSplitPoint {
    /** 点x坐标 */
    float x;
    /** 点y坐标 */
    float y;
    /** 图像中的x坐标 */
    float image_x;
    /** 图像中的y坐标 */
    float image_y;
    /** 点类型 */
    uint8_t point_type;
    /** 合并或分离后的车道线类型 */
    uint16_t next_segment_laneline_type;
    /** 合并或分离后的车道线颜色 */
    int16_t next_segment_laneline_color;  // NO_COLOR_LINE
    /** 合并或分离后的车道线宽度 */
    sapLineWidth next_segment_laneline_width;
    /** 合并或分离后车道线颜色置信度 */
    float next_segment_laneline_color_confidence;
    /** 合并或分离后的车道线置信度 */
    float next_segment_laneline_type_confidence;
    /** 合并或分离后的车道线宽度置信度 */
    float next_segment_laneline_width_confidence;
} sapMergeSplitPoint;

/**
 * @brief 车道线拟合方法，算法内部使用
 */
typedef enum {
    /** 未知 */
    UNKNOWN = -1,
    /** 多项式拟合 */
    POLYNOMIAL_FITTING,
    /** 拟合方法枚举最大值 */
    NUM_LANEFITTINGTYPES_MAX
} sapLaneFittingType;

/**
 * @brief 多项式拟合结果
 *
 */
typedef struct _sapPolynomialFittingResult {
    /** 多项式拟合度 */
    int fitting_degree;
    /** 多项式拟合系数 */
    float* polyfit_coefficients;
    /** 多项式拟合系数个数，一般为4 */
    uint32_t polyfit_coef_num = 0;
} sapPolynomialFittingResult;

/**
 * @brief 车道线拟合结果
 */
typedef struct _sapLaneFittingResult {
    /** 拟合类型 */
    sapLaneFittingType fitting_type;
    /** 多项式拟合结果 */
    sapPolynomialFittingResult polyfit_result;
} sapLaneFittingResult;

/**
 * @brief 车道线交汇信息
 *
 */
typedef struct _sapLineJunction {
    /** 是否有效 */
    int8_t is_valid;
    // std::vector<Eigen::sapVector2f> junction_points;
} sapLineJunction;

/**
 * @brief 车道线信息
 */
typedef struct _sapLaneLine {
    /** 车道分类类别 */
    sapLaneCategory lane_category = NO_CATEGORY;
    /** 车道颜色 */
    int16_t color = 0;  // NO_COLOR_LINE
    /** 线类型 */
    sapLineType line_type = NO_LINE_LINE;
    /** 车道线类别 */
    uint8_t laneline_type = 0;
    /** 路沿类型 */
    int8_t edge_type = 0;  // NO_EDGE_LINE
    /** 车道线置信度 */
    float lane_confidence = -1.f;
    /** 车道线颜色置信度 */
    float color_confidence = -1.f;
    /** 线类型属性line_type的置信度 */
    float line_type_confidence = -1.f;
    /** 车道线类型laneline_type的置信度 */
    float laneline_type_confidence = -1.f;
    /** 路沿类型置信度 */
    float edge_type_confidence = -1.f;
    /** 车道线宽度置信度 */
    float laneline_width_confidence = -1.f;

    /** BV视角下的车道线起始点，单位米，自车坐标系 */
    sapVector2f start_point;
    /** BV视角下的车道线终止点，单位米，自车坐标系 */
    sapVector2f end_point;

    sapVector2f start_point_mid;
    sapVector2f end_point_mid;

    sapVector2f start_point_front;
    sapVector2f end_point_front;

    sapVector2f start_point_rear;
    sapVector2f end_point_rear;
    int8_t mid_pos;
    int8_t front_pos;
    int8_t rear_pos;

    /** 分离或合并点数目 */
    uint64_t merge_split_point_num = 0;
    /** 分离或合并点数组 */
    sapMergeSplitPoint* merge_split_points;
    /** 图像像素坐标系下的中间线点个数 */
    uint64_t image_point_num = 0;
    /** 图像像素坐标系下的中间线点集合 */
    sapVector2f* image_points;
    /** BV视角下的反投影采样点个数，HMI使用 */
    uint64_t world_point_num = 0;
    /** BV视角下的反投影采样点,单位米，自车坐标系 */
    sapPoint2fExtended* world_points_reproj;
    /** BV视角下拟合结果 */
    sapLaneFittingResult bv_fitting_results;
    sapLaneFittingResult bv_fitting_results_mid;
    sapLaneFittingResult bv_fitting_results_front;
    sapLaneFittingResult bv_fitting_results_rear;
    /** 图像视角下的拟合结果 */
    sapLaneFittingResult img_fitting_results;
    /** 不同线型交汇点信息 */
    sapLineJunction line_junction;
} sapLaneLine;

/**
 * @brief 初始化车道线结构体信息
 * @param[in,out] lane 车道线结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapLaneLine(sapLaneLine* lane);

/**
 * @brief 车道线结果信息
 */
typedef struct _sapLaneLineResult {
    /** BV视角车道线描述 */
    sapLaneLine data;
    /** 图像视角车道线描述 */
    sapLaneLine data_img;
    /** 车道类别ID，可选值NO_LANE=-1, LEFT_LEFT_LANE=0,
     * LEFT_LANE=1,RIGHT_LANE=2, RIGHT_RIGHT_LANE=3, NUM_LANES_MAX=4 */
    int32_t id;
    /** 车道姿态索引号，可选值：HOST_LEFT=0, HOST_RIGHT=1,
     * LEFT_NEIGHBOR_LANE_LEFT=2,LEFT_NEIGHBOR_LANE_RIGHT=3,
     * RIGHT_NEIGHBOR_LANE_LEFT=4,RIGHT_NEIGHBOR_LANE_RIGHT=5,
     * REAR_RIGHT=6,REAR_LEFT=7, R
     * OAD_EDGE_LEFT = 8, ROAD_EDGE_RIGHT=9, INVALID = 255 */
    int16_t position_index;
    /** 车道线跟踪ID，跟踪丢失+1 */
    int track_id;
    /** 跟踪长度，连续跟踪图像帧数 */
    int track_age;
    /** 车道线与自车左保险杠的距离 */
    float distance_to_left_bumper;
    /** 车道线与自车右保险柜的距离 */
    float distance_to_right_bumper;
    /** 车道线宽度，单位cm */
    sapLineWidth laneline_width;
} sapLaneLineResult;

/**
 * @brief 初始化车道线结果信息结构体
 * @param[in,out] lane_line 车道线结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapLaneLineResult(sapLaneLineResult* lane_line);

/**
 * @brief 中心线转向类型
 */
typedef enum {
    /** 直行 */
    TOPOLOGY_STRAIGHT = 0,
    /** 左转 */
    TOPOLOGY_LEFT,
    /** 右转 */
    TOPOLOGY_RIGHT,
    /** 调头 */
    TOPOLOGY_U_TURN,
    /** 禁止通行 */
    TOPOLOGY_FORBIDDEN,
    /** 其他 */
    TOPOLOGY_OTHER
} sapCenterLineType;

#define MAX_NUM_TYPES 6

/**
 * @brief 路面拓扑中心线结构体
 */
typedef struct _sapCenterLine {
    /** BEV坐标系下的中心线点集 */
    sapPoint2f* points;
    /** BEV坐标系下的中心线点个数 */
    uint64_t point_num;
    /** 中心线转向类型 */
    sapCenterLineType* types;
    /** 中心线转向类型个数 */
    uint32_t type_num;
    // @brief attribute confidence
    float confidence;
} sapCenterLine;

/**
 * @brief 初始化中心线结构体信息
 * @param[in,out] lane 中心线结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapCenterLine(sapCenterLine* lane);

/**
 * @brief 路面拓扑中心线结构体
 */
typedef struct _sapCenterLineResult {
    // @brief laneline description
    sapCenterLine data;
    // @brief img laneline description
    sapCenterLine data_img;
    // @brief laneline track id
    uint32_t track_id;
    // @brief laneline tracking age
    uint32_t track_age;
} sapCenterLineResult;

/**
 * @brief 初始化中心线结果结果结构体信息
 * @param[in,out] lane 中心线结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapCenterLineResult(sapCenterLineResult* lane);

/**
 * @brief 现有车道线结果预测出的车道线
 */
typedef struct _sapHolisticPathResult {
    // @brief bv points
    uint64_t point_bv_num = 0;
    sapVector2f* points_bv;
    // @brief bv sampled points
    uint64_t sampled_point_bv_num = 0;
    sapVector2f* sampled_points_bv;
    // @brief img points
    uint64_t point_image_num = 0;
    sapVector2f* points_image;
    // @brief bv fitting results, incl. polyfit coefficients
    sapLaneFittingResult fitting_result;
    // @brief confidence of hpp
    float confidence = 0.0;
    // @brief bv start point of hpp
    sapVector2f start_point;
    // @brief bv end point of hpp
    sapVector2f end_point;
    // @brief hpp track id
    int track_id = 0;
    // @brief hpp tracking age
    int track_age = 0;
} sapHolisticPathResult;

/**
 * @brief 初始化sapHolisticPathResult结构体
 * @param[in,out] holistic_path
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapHolisticPathResult(sapHolisticPathResult* holistic_path);

/**
 * @brief 停止线结果信息
 */
typedef struct _sapStopLineResult {
    /** 跟踪ID */
    int32_t id = -1;
    /** 图像视角(像素坐标)下的停止线点个数 */
    uint64_t image_point_num;
    /** 图像视角下的停止线点集 */
    sapVector2f* image_points;
    /** BV视角下的点个数 */
    uint64_t world_point_num;
    /** BV视角下的点集 */
    sapVector2f* world_points;
    /** 置信度数量 */
    uint64_t conf_num;
    /** 点的置信度 */
    float* confidence;
} sapStopLineResult;

/**
 * @brief 初始化停止线结构体信息
 * @param[in,out] stop_line 停止线结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapStopLineResult(sapStopLineResult* stop_line);

/**
 * @brief 减速带感知结果信息
 */
typedef struct _sapSpeedBumpResult {
    /** 跟踪ID */
    int32_t id = -1;
    /** 图像视角（像素坐标系）下的减速带点数量 */
    uint64_t image_point_num;
    /** 图像视角（像素坐标系）下的减速带点集 */
    sapVector2f* image_points;
    /** BV视角下的减速带点数量 */
    uint64_t world_point_num;
    /** BV视角下的减速带点集 */
    sapVector2f* world_points;
    /** 置信度数量 */
    uint64_t conf_num;
    /** 点置信度 */
    float* confidence;
} sapSpeedBumpResult;

/**
 * @brief 初始化减速带结构体信息
 * @param[in,out] speed_bump 减速带结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapSpeedBumpResult(sapSpeedBumpResult* speed_bump);

/**
 * @brief 分界线
 */
typedef struct _sapBoundaryResult {
    /** 跟踪ID */
    int32_t id;
    /** 图像视图下的点数量 */
    uint64_t image_point_num;
    /** 图像视图（像素坐标）下的点集 */
    sapVector2f* image_points;
    /** BV视图下的点数量 */
    uint64_t world_point_num;
    /** BV视图下的点集 */
    sapVector2f* world_points;
    /** x方向朝向，单位为弧度[rad]，坐标系为自车坐标系 */
    float direction_x;
    /** y方向朝向，单位为弧度[rad]，坐标系为自车坐标系 */
    float direction_y;
    /** 分界线的置信度 */
    float confidence;
    /** 是否是线型,默认0 */
    int8_t is_line;
} sapBoundaryResult;

/**
 * @brief 初始化分界线结构体
 * @param[in] boundary 分界线结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapBoundaryResult(sapBoundaryResult* boundary);

/**
 * @brief 可行驶区域分段信息类型
 */
typedef enum _sapFreespaceSegmentType {
    Unknown = 0,
    RoadBoundary,
    StaticObstacle,
    DynamicObstacle
} sapFreespaceSegmentType;

/**
 * @brief 可行驶区域分段信息
 */
typedef struct _sapFreespaceSegment {
    size_t start_point_index;
    size_t end_point_index;
    sapFreespaceSegmentType type;
} sapFreespaceSegment;

/**
 * @brief 可行驶区域
 */
typedef struct _sapFreespaceResult {
    /** 跟踪ID */
    int32_t id = -1;
    /** 可行驶扇形区域中，每个角度的最大距离点所对应的像素坐标点数量 */
    uint64_t image_point_num = 0;
    /** 可行驶扇形区域中，每个角度的最大距离点所对应的像素坐标 */
    sapVector2f* image_points;
    /** BEV视角下每个角度的最大距离点数量 */
    uint64_t world_point_num = 0;
    /** 可行驶扇形区域中，BEV视角下每个角度的最大距离点坐标，
     * 单位为米，坐标系为自车坐标系 */
    sapVector2f* world_points;
    /** 可行驶区域分段信息集合数量 */
    uint64_t segment_num = 0;
    /** 可行驶区域分段信息集合 */
    sapFreespaceSegment* segments;
    /** 每个角度最大可行驶距离所对应的置信度数量 */
    sapPoint2f camera_optical_center;
    /** 可行驶扇形区域中，每个角度最大可行驶距离所对应的置信度 */
    float* confidence;
    /** 角度数据 */
    float* angle_result;
    /** 角度范围 */
    float* range_result;
    /** 角度范围数量 */
    uint64_t angle_range_num;
    bool* bit_map;
    u_int16_t width = 0;
    u_int16_t height = 0;
    float resolution = 0.0;
} sapFreespaceResult;

/**
 * @brief 初始化可行驶区域结构体
 * @param[in] freespace 可行驶区域结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapFreespaceResult(sapFreespaceResult* freespace);

/**
 * @brief 道路区域信息结构
 */
typedef struct _RoadRegionResult {
    /** 编号 */
    int32_t id = -1;
    /** 区域类型 */
    uint8_t region_type = -1;
    /** 跟踪ID */
    uint32_t track_id = 0;
    /** 区域类型置信度 */
    float region_type_confidence = 1;
    /** 区域边缘点数量 */
    uint64_t region_boundary_num = 0;
    /** 区域边缘点集合 */
    sapVector2f* region_boundary;
    /** 交叉点数量 */
    uint64_t junction_point_num = 0;
    /** 交叉点集合 */
    sapVector2f* junction_point_vector;
} RoadRegionResult;

/**
 * @brief 坐标点数组
 */
typedef struct _sapPointInfo {
    /** 点的数量 */
    uint64_t num = 0;
    /** 点的集合 */
    sapVector2f* points;
} sapPointInfo;

/**
 * @brief 置信度信息集合
 */
typedef struct _sapConfidenceInfo {
    /** 置信度数量 */
    uint64_t num = 0;
    /** 置信度集合 */
    float* val;
} sapConfidenceInfo;

/**
 * @brief 人行道
 */
typedef struct _sapCrosswalkResult {
    /** 跟踪编号 */
    int32_t id;
    /** 图像视图下的人行道点集数量 */
    uint64_t image_point_num;
    /** 图像视图下的人行道点集 */
    sapPointInfo* image_points;
    /** BV视图下的人行道点集数量 */
    uint64_t world_point_num;
    /** BV视图下的人行道点集 */
    sapPointInfo* world_points;
    /** 置信度数量 */
    uint64_t conf_num;
    /** 人行道点集置信度 */
    sapConfidenceInfo* confidence;
    /** 人行道多连通区域对应的连通区域数目 */
    uint32_t instance_id_num;
    /** 人行道多连通区域对应的连通区域id数组 */
    int32_t* instance_id;
    /** 位图，非人行道区域为0，人行道区域为非0 */
    int8_t* bit_map;
    /** 位图数组长度 */
    uint64_t bit_map_num;
    /** 位图宽度 */
    uint16_t width;
    /** 位图高度 */
    uint16_t height;
} sapCrosswalkResult;

/**
 * @brief 初始化人行道结构体
 * @param[in] crosswalk 人行道结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapCrosswalkResult(sapCrosswalkResult* crosswalk);

/**
 * @brief 老IPM车位信息：/parking/perception/camera/parking_slots -SLAM使用
 */
typedef struct _sapParkingSlotResult {
    /** 车位四个角点坐标 */
    sapPoint2f tl, tr, bl, br;
    /** 模型原始检出四个角点坐标 */
    sapPoint2f ori_tl, ori_tr, ori_bl, ori_br;
    /** 车位四个角点坐标 + 角点中点坐标(大卓) */
    sapPoint3f* points;
    // /** 车位点在image上的投影 */
    // sapPoint2f* points_image;
    /** 车位点数量 */
    int16_t points_num;
    /** 置信度
     * 当前车位被支持的角点数量/4，
     * 若车位可以被4个角点生成的anchor框支持，那么其confidence为1.0(4/4)；
     * 若车位只有两个角点支持，那么其confidence为0.5(2/4)
     */
    float confidence;

    /** 车位的宽度
     * 单位为像素坐标，
     * 目前为适应非固定大小车位，
     * 车位的宽度存在三个尺度
     */
    float width;

    /** 车位的长度
     * 单位为像素坐标，
     * 目前为适应非固定大小车位，
     * 车位的宽度存在三个尺度
     */
    float length;

    /** 车位中心点
     * 单位为像素坐标，可能在图像外
     */
    sapPoint3f center;

    /** 车位短边平行方向，
     * 和长度方向构成右手坐标系（不一定完全垂直）
     * 2D单位向量，
     * 自车所在车位或者横向车位时，这个方向有歧义，但是对于生成车位角点不影响；
     */
    sapVector3f wide_direction;
    /** 指向车位开口方向，和车位长边平行
     * 2D单位向量，
     * 自车所在车位或者横向车位时，这个方向有歧义，但是对于生成车位角点不影响；*/
    sapVector3f long_direction;

    /** 车位是否被占据
     * false: 空车位
     * true: 占据车位
     */
    bool is_occupied;

    /** 车位类型
     * vertical:0
     * parallel:1
     * slant:2
     */
    uint8_t type;
    uint64_t id;
    /** 限位器端点坐标 */
    sapPoint2f wheelstop_points[2];
    /** 限位器端点数量
     * 0， 没有限位器点
     * 1， 单个限位器点
     * 2， 两个限位器端点
     */
    int16_t wheelstop_point_num;
    /** 限位器分数 */
    float wheelstop_score;
    /** 限位器类型
     * 0, 没有检测到限位块/杆
     * 1, 检测到限位块/杆
     */
    uint8_t LimiterStatus;
} sapParkingSlotResult;

/**
 * @brief 初始化车位信息结构体
 * @param[in] parking_slot 车位信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapParkingSlotResult(sapParkingSlotResult* parking_slot);

typedef struct _sapParkingSlotStopper {
    /** 轮挡坐标 */
    sapPoint2f pos;
    float confidence;
    uint8_t type;
} sapParkingSlotStopper;

/**
 * @brief 初始化车位轮挡结构体
 * @param[in] parking_slot_stopper 车位信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t
InitsapParkingSlotStopper(sapParkingSlotStopper* parking_slot_stopper);

/**
 * @brief 置信度信息集合
 */
typedef struct _sapApproxBoxPoints {
    sapPoint2f point;
    float border_dist = 0.f;
    float point_score = 0.f;
    float line_len = 0.f;
    float line_score = 0.f;
    bool has_border_point = false;
} sapApproxBoxPoints;

/**
 * @brief 老IPM车位：/perception/camera/parking_bboxes-车位融合使用
 */
typedef struct _sapQuadParkingSlotResult {
    /** 车位四个角点坐标 */
    sapPoint2f tl, tr, bl, br;
    /** 置信度
     * 当前车位被支持的角点数量/4，
     * 若车位可以被4个角点生成的anchor框支持，那么其confidence为1.0(4/4)；
     * 若车位只有两个角点支持，那么其confidence为0.5(2/4)
     */
    float confidence;

    /** 占用类型
     * label == 1判断为occupied
     */
    uint32_t label;

    /** 是否为被过滤（失效）的车位角点信息 */
    bool filtered = false;

    /** 车位类型
     * vertical:0
     * parallel:1
     * slant:2
     */
    int slot_type = -1;

    /* 四个角点的置信度信息 */
    float s_tl, s_tr, s_bl, s_br;

    /** 入口方向 */
    sapPoint2f dir_in;
    /** 宽边方向 */
    sapPoint2f dir_width;
    /** 长边方向 */
    sapPoint2f dir_length;
    /** 中心点坐标 */
    sapPoint2f center;

    /** 是否修正最优边 */
    bool opp_modify = false;

    /** 角点是否组成一个完整车位 */
    bool is_complete = false;

    /** 车位宽度 */
    float width = 0;
    /** 车位长度 */
    float length = 0;

    /** 车位对齐时辅助变量，内部对齐使用*/
    bool is_visited = false;

    /** 角点属性 */
    sapApproxBoxPoints *p_tl, *p_tr, *p_bl, *p_br;
    /** 角点属性数量 */
    uint64_t p_tl_num, p_tr_num, p_bl_num, p_br_num;

    /** 角点的初始坐标 */
    sapPoint2f ori_tl, ori_tr, ori_bl, ori_br;

} sapQuadParkingSlotResult;

/**
 * @brief 初始化车位信息结构体
 * @param[in] parking_slot 车位信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t
InitsapQuadParkingSlotResult(sapQuadParkingSlotResult* quad_parking_slot);

/*
 * @brief roadmarker
 */
typedef struct _sapRoadmarkerResult {
    /** 跟踪ID */
    int32_t id = -1;
    /** 图像视角(像素坐标)下的 road marker 点个数 */
    uint64_t image_point_num;
    /** 图像视角下的 road marker 点集 */
    sapVector2f* image_points;
    /** BV视角下的点个数 */
    uint64_t world_point_num;
    /** BV视角下的点集 */
    sapVector2f* world_points;
    /** 置信度数量 */
    uint64_t conf_num;
    /** 点的置信度 */
    float* confidence;
    int8_t roadmarker_type;
    uint8_t kp_types_num;  // keypoints_type
    int8_t* kp_types;
    float heading;

} sapRoadmarkerResult;

/**
 * @brief 初始化roadmarker结构体
 * @param[in] roadmarker roadmarker结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapRoadmarkerResult(sapRoadmarkerResult* roadmarker);

/**
 * @brief 图像信息
 *
 */
typedef struct _sapImage {
    /** 图像宽 */
    int height;
    /** 图像高 */
    int width;
    /** 图像通道 */
    int channel;
    /** 图像数据长度 */
    int buffer_size;
    /** 编码方式 */
    char encoding[NAME_STR_MAX_LEN];
    void* data;
} sapImage;

/**
 * @brief 路面地理信息
 */
typedef struct _sapRoadStruct {
    /** 车道线数量 */
    uint16_t laneline_num;
    /** 车道线集合 */
    sapLaneLineResult* laneline_results;
    /** 中心线数量 */
    uint16_t centerline_num;
    /** 中心线集合 */
    sapCenterLineResult* centerline_results;
    /** 停止线数量 */
    uint16_t stopline_num;
    /** 停止线集合 */
    sapStopLineResult* stopline_results;
    /** 减速带数量 */
    uint16_t speedbump_num;
    /** 减速带集合 */
    sapSpeedBumpResult* speedbump_results;
    /** 分界线数量 */
    uint16_t boundary_num;
    /** 分界线集合 */
    sapBoundaryResult* boundary_results;
    /** 可行驶区域数量 */
    uint16_t freespace_num;
    /** 可行驶区域集合 */
    sapFreespaceResult* freespace_results;
    /** 人行道数量 */
    uint16_t crosswalk_num;
    /** 人行道集合 */
    sapCrosswalkResult* crosswalk_results;
    /** roadmarker数量 */
    uint16_t roadmarker_num;
    /** roadmarker集合 */
    sapRoadmarkerResult* roadmarker_results;
    /** 预测的可参考车道线数量 */
    uint16_t holistic_path_num;
    /** 预测的可参考车道线集合 */
    sapHolisticPathResult* holistic_path_results;
    /** 车位数量 */
    uint16_t parking_slot_num;
    /** 车位结果 */
    sapParkingSlotResult* parking_slot_results;
    /** IPM车位数量 */
    uint16_t ipm_parking_slot_num;
    /** IPM车位结果 */
    sapParkingSlotResult* ipm_parking_slot_results;
    /** 车位数量(老IPM车位结果-车位融合用) */
    uint16_t quad_parking_slot_num;
    /** 车位结果(老IPM车位结果-车位融合用) */
    sapQuadParkingSlotResult* quad_parking_slot_results;
    /** IPM车位轮挡 */
    uint16_t ipm_parking_slot_stopper_num;
    sapParkingSlotStopper* ipm_parking_slot_stoppers;
    sapImage* ipm_image;
    sapImage* parking_slot_seg;
} sapRoadStruct;

/**
 * @brief 单帧路面道路几何信息感知结果
 */
typedef struct _sapPerceptionRoadFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 传感器名称 */
    char sensor_name[NAME_STR_MAX_LEN];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 路面信息 */
    sapRoadStruct road_struct;
} sapPerceptionRoadFrame;

/**
 * @brief 初始化路面地理信息结构体
 * @param[in,out] frame 路面道路几何信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionRoadFrame(sapPerceptionRoadFrame* frame);

/**
 * @brief 释放
 * @param[in,out] frame 路面道路几何信息结构体指针
 * @return 成功返回0，否则返回错误码
 * @note
 * 此方法只释放结构体内部指针指向的内存，并不释放sapPerceptionRoadFrame结构体本身的内存
 */
SAP_API void ReleaseRoadFrameMemory(sapPerceptionRoadFrame* frame);

#ifdef __cplusplus
}
#endif
