/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * He Yuan <heyuan@sensetime.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "perception_common/base/data_type/base.hpp"

#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/object/camera/traffic_light.hpp"
#include "perception_common/data_type/object/camera/traffic_sign.hpp"
#include "perception_common/data_type/object/camera/road_marker_object.hpp"
#include "perception_common/data_type/object/camera/board_pole.hpp"
namespace senseAD {
namespace perception {

typedef struct StructBoundary {
    std::vector<Eigen::Vector2f> image_points;
    std::vector<Eigen::Vector2f> world_points;
    float direction_x = 0;
    float direction_y = 0;
    float confidence = -1.f;
    bool is_line = false;
} Boundary;

typedef struct StructStopLine {
    std::vector<Eigen::Vector2f> image_points;
    std::vector<Eigen::Vector2f> world_points;
    std::vector<float> confidence;
} StopLine;

typedef struct StructSpeedBump {
    std::vector<Eigen::Vector2f> image_points;
    std::vector<Eigen::Vector2f> world_points;
    std::vector<float> confidence;
} SpeedBump;

typedef struct StructFreespace {
    std::vector<Eigen::Vector2f> image_points;
    std::vector<Eigen::Vector2f> world_points;
    Eigen::Vector2f camera_optical_center;
    std::vector<float> confidence;
    std::vector<float> angle_results;
    std::vector<float> range_result;

    std::vector<bool> bit_map;
    uint16_t width;
    uint16_t height;
    float resolution;
    void Init(uint16_t input_width, uint16_t input_height) {
        width = input_width;
        height = input_height;
        if (width > 0 && height > 0) {
            bit_map.resize(width * height, false);
        }
    }
} Freespace;

typedef struct StructRoadRegion {
    uint8_t region_type = -1;
    uint32_t track_id = 0;
    float region_type_confidence = 1;
    std::vector<Eigen::Vector2f> region_boundary;
    std::vector<Eigen::Vector2f> junction_point_vector;
} RoadRegion;

/**
 * @brief laneline's positional type
 **/
enum class LaneType {
    NO_LANE = -1,
    LEFT_LEFT_LANE,   /**< left left laneline*/
    LEFT_LANE,        /**< left laneline*/
    RIGHT_LANE,       /**< right laneline*/
    RIGHT_RIGHT_LANE, /**< right right laneline*/
    NUM_LANES_MAX     /**< max laneline number limitation*/
};

/**
 * @brief road type for roadside
 **/
enum class RoadSideType {
    NO_ROAD_SIDE = -1,
    LEFT_ROAD_SIDE,
    RIGHT_ROAD_SIDE,
    NUM_ROADSIDETYPE_MAX /**< max roadside type number limitation*/
};

/**
 * @brief laneline's positional index
 **/
enum LanePoseIndex {
    HOST_LEFT = 0,
    HOST_RIGHT,
    LEFT_NEIGHBOR_LANE_LEFT,
    LEFT_NEIGHBOR_LANE_RIGHT,
    RIGHT_NEIGHBOR_LANE_LEFT,
    RIGHT_NEIGHBOR_LANE_RIGHT,
    REAR_RIGHT,
    REAR_LEFT,
    ROAD_EDGE_LEFT,
    ROAD_EDGE_RIGHT,
    FRONT_RIGHT,
    FRONT_LEFT,
    INVALID = 255
};

/**
 * @brief color type for laneline
 **/
enum class ColorType {
    NO_COLOR_LINE = 0,
    OTHER_COLOR,
    WHITE_COLOR,
    YELLOW_COLOR,
    ORANGE_COLOR,
    BLUE_COLOR,
    NUM_COLORTYPES_MAX /**< max colortype number limitation*/
};

/**
 * @brief line type for laneline
 **/
enum class LineType {
    NO_LINE_LINE = 0,
    OTHER_LINE,
    DASHED_LINE,
    SOLID_LINE,
    DOUBLE_DASHED_LINE,
    DOUBLE_SOLID_LINE,
    DASHED_SOLID_LINE,
    SOLID_DASHED_LINE,
    TRIPLE_DASHED_LINE,
    DASHED_SOLID_DASHED_LINE,
    SOLID_TO_DASHED,
    DASHED_TO_SOLID,
    NUM_LINETYPES_MAX /**< max linetype number limitation*/
};

/*
 * @brief: Package of lm lane line type
 */
enum class LaneLineType : std::uint8_t {
    LANE_LINE_UNKNOWN_TYPE = 0,
    SINGLE_DASHED,
    SINGLE_SOLID,
    DOUBLE_DASHED,
    DOUBLE_SOLID,
    LEFT_DASHED_RIGHT_SOLID,
    LEFT_SOLID_RIGHT_DASHED,
    ROAD_EDGE,
    /** 短粗虚线 */
    SHORT_THICK_DASHED
    // bit 5 - FISHBONE FLAG
    // NO_FISHBONE = 0,
    // HAS_FISHBONE = 1,
    // bit 6, 7 reserved
};

/**
 * @brief edge type for laneline
 **/
enum class EdgeType {
    NO_EDGE_LINE = 0,
    ROAD_CURB_EDGE,
    FENCE_EDGE,
    WALL_EDGE_OR_PLANT_EDGE,
    MOVABLE_EDGE,
    OTHER_EDGE,
    NUM_EDGETYPES_MAX /**< max edgetype number limitation*/
};

enum class LaneCategory {
    NO_CATEGORY = -1,
    WHITE_DASHED_LANE,
    WHITE_SOLID_LANE,
    DOUBLE_WHITE_DASHED_LANE,
    DOUBLE_WHITE_SOLID_LANE,
    WHITE_DASHED_SOLID_LANE,
    WHITE_SOLID_DASHED_LANE,
    YELLOW_DASHED_LANE,
    YELLOW_SOLID_LANE,
    DOUBLE_YELLOW_DASHED_LANE,
    DOUBLE_YELLOW_SOLID_LANE,
    YELLOW_DASHED_SOLID_LANE,
    YELLOW_SOLID_DASHED_LANE,
    SERRATED_SOLID_LANE,
    LEFT_ROADSIDE_LANE,
    RIGHT_ROADSIDE_LANE,
    NUM_LANES_CATEGORY_MAX
};

/**
 * @brief laneline's fitting type
 **/
enum class LaneFittingType {
    UNKNOWN = -1,
    POLYNOMIAL_FITTING,      /**< Polynomial fitting*/
    NUM_LANEFITTINGTYPES_MAX /**< max lane fitting types number limitation*/
};

typedef struct StructPolynomialFittingResult {
    // @brief Polynomial fitting degree
    int fitting_degree = -1;
    // @brief Polynomial fitting coefficients
    std::vector<float> polyfit_coefficients;
} PolynomialFittingResult;

typedef struct StructLaneFittingResult {
    // @brief fitting type
    LaneFittingType fitting_type = LaneFittingType::UNKNOWN;
    // @brief Polynomial fitting result
    PolynomialFittingResult polyfit_result;
} LaneFittingResult;

typedef struct StructLineJunction : ValidInfo {
    std::vector<Eigen::Vector2f> junction_points;
} LineJunction;

/**
 * @brief laneline's line width
 **/
typedef struct StructLineWidth {
    // laneline_width [cm]
    uint8_t width = 0;
    // left laneline width [cm]
    uint8_t width_left = 0;
    // right laneline width [cm]
    uint8_t width_right = 0;
} LineWidth;

/**
 * @brief laneline's merge/split point
 **/
typedef struct StructMergeSplitPoint {
    float x = 0.f;
    float y = 0.f;
    float image_x = 0.f;
    float image_y = 0.f;
    uint8_t point_type = 0;
    uint8_t next_segment_laneline_type = 0;
    ColorType next_segment_laneline_color = ColorType::NO_COLOR_LINE;
    LineWidth next_segment_laneline_width;
    float next_segment_laneline_color_confidence = 0.0;
    float next_segment_laneline_type_confidence = 0.0;
    float next_segment_laneline_width_confidence = 0.0;
} MergeSplitPoint;

typedef struct StructCrosswalk {
    std::vector<std::vector<Point2f>> image_points;
    std::vector<std::vector<Point2f>> world_points;
    std::vector<std::vector<float>> confidence;
    std::vector<int32_t> instance_id;
    std::vector<bool> bit_map;
    uint16_t width{0};
    uint16_t height{0};
} Crosswalk;

typedef struct StructLaneLine {
    // @brief lane appearing description, incl. color,
    // linetype and egde type
    LaneCategory lane_category = LaneCategory::NO_CATEGORY;
    ColorType color = ColorType::NO_COLOR_LINE;
    LineType line_type = LineType::NO_LINE_LINE;
    uint8_t laneline_type = 0;
    EdgeType edge_type = EdgeType::NO_EDGE_LINE;
    // @brief attribute confidence
    float lane_confidence = -1.f;
    float color_confidence = -1.f;
    float line_type_confidence = -1.f;
    float laneline_type_confidence = -1.f;
    float edge_type_confidence = -1.f;
    float laneline_width_confidence = -1.f;

    // @brief bv laneline start/end point
    Eigen::Vector2f start_point;
    Eigen::Vector2f end_point;

    /// @brief merge/split points
    std::vector<MergeSplitPoint> merge_split_points;
    // @brief image points
    std::vector<Eigen::Vector2f> image_points;
    // std::vector<Eigen::Vector2f> world_points;
    // @brief bv sampled points, for HMI
    std::vector<Eigen::Vector2f> world_points_reproj;
    // @brief bv fitting results, incl. polyfit coefficients
    LaneFittingResult bv_fitting_results;
    // @brief img fitting results, incl. polyfit coefficients
    LaneFittingResult img_fitting_results;
    // @brief cross junction of different line types, e.g. solid & dashed
    LineJunction line_junction;
} LaneLine;

typedef struct StructCenterLine {

    // @brief attribute confidence
    float confidence = -1.f;

    // @brief bv points
    std::vector<Eigen::Vector2f> bv_points;

    // @brief bv points type, support multi types
    // STRAIGHT         = 1 << 0,  // 0000 0000 0000 0001
    // TURN_LEFT        = 1 << 1,  // 0000 0000 0000 0010
    // TURN_RIGHT       = 1 << 2,  // 0000 0000 0000 0100
    // TURN_LEFT_AROUND = 1 << 3,  // 0000 0000 0000 1000
    // TURN_RIGHT_AROUND= 1 << 4,  // 0000 0000 0001 0000
    // TURN_LEFT_AREA   = 1 << 5,  // 0000 0000 0010 0000
    // UNKNOWN          = 1 << 6   // 0000 0000 0100 0000
    std::vector<uint16_t> bv_points_type;

} CenterLine;

/*
 * @brief: Package of lane line info
 */
class LaneLineResult {
 public:
    LaneLineResult() {}
    explicit LaneLineResult(const LaneLine& laneline,
                            LaneType lane_id = LaneType::NO_LANE,
                            int lane_track_id = -1,
                            int lane_track_age = 0,
                            float left_bumper_distance = 1000,
                            float right_bumper_distance = 1000)
        : data(laneline),
          id(lane_id),
          track_id(lane_track_id),
          track_age(lane_track_age),
          distance_to_left_bumper(left_bumper_distance),
          distance_to_right_bumper(right_bumper_distance) {}
    LaneLineResult(const LaneLineResult& laneline_result) = default;

 public:
    // @brief laneline description
    LaneLine data;
    // @brief img laneline description
    LaneLine data_img;
    // @brief laneline id, e.g. Left/Right
    LaneType id = LaneType::NO_LANE;
    // @brief laneline position index
    LanePoseIndex position_index = LanePoseIndex::INVALID;
    // @brief laneline track id
    int track_id = 0;
    // @brief laneline tracking age
    int track_age = 0;
    // @brief laneline distance to ego car left bumper
    float distance_to_left_bumper = 0;
    // @brief laneline distance to ego car right bumper
    float distance_to_right_bumper = 0;
    // laneline_width [cm]
    LineWidth laneline_width;
};
typedef std::shared_ptr<LaneLineResult> LaneLineResultPtr;
typedef std::shared_ptr<const LaneLineResult> LaneLineResultConstPtr;

/*
 * @brief: Package of center line info
 */
class CenterLineResult {
 public:
    CenterLineResult() {}
    explicit CenterLineResult(const CenterLine& centerline,
                            int lane_track_id = -1,
                            int lane_track_age = 0)
        : data(centerline),
          track_id(lane_track_id),
          track_age(lane_track_age) {}
    CenterLineResult(const CenterLineResult& centerline_result) = default;

 public:
    // @brief laneline description
    CenterLine data;
    // @brief img laneline description
    CenterLine data_img;
    // @brief laneline track id
    int track_id = -1;
    // @brief laneline tracking age
    int track_age = 0;

};
typedef std::shared_ptr<CenterLineResult> CenterLineResultPtr;
typedef std::shared_ptr<const CenterLineResult> CenterLineResultConstPtr;

/*
 * @brief: Package of boundary info
 */
class BoundaryResult {
 public:
    BoundaryResult() {}
    explicit BoundaryResult(const Boundary& boundary, int32_t boundary_id = -1)
        : data(boundary), id(boundary_id) {}
    BoundaryResult(const BoundaryResult& boundary_result) = default;

 public:
    Boundary data;
    int32_t id = -1;
};

/*
 * @brief: Package of stop line
 */
class StopLineResult {
 public:
    StopLineResult() {}
    explicit StopLineResult(const StopLine& stopline_data,
                            int32_t stopline_id = -1)
        : data(stopline_data), id(stopline_id) {}
    StopLineResult(const StopLineResult& stopline_result) = default;

 public:
    StopLine data;
    int32_t id = -1;
};

/*
 * @brief: Package of stop line
 */
class SpeedBumpResult {
 public:
    SpeedBumpResult() {}
    explicit SpeedBumpResult(const SpeedBump& speed_bump_data,
                            int32_t speed_bump_id = -1)
        : data(speed_bump_data), id(speed_bump_id) {}
    SpeedBumpResult(const SpeedBumpResult& speed_bump_result) = default;

 public:
    SpeedBump data;
    int32_t id = -1;
};

/*
 * @brief: Package of free space
 */
class FreespaceResult {
 public:
    FreespaceResult() {}
    explicit FreespaceResult(const Freespace& stopline_data,
                             int32_t stopline_id = -1)
        : data(stopline_data), id(stopline_id) {}
    FreespaceResult(const FreespaceResult& stopline_result) = default;

 public:
    Freespace data;
    int32_t id = -1;
};

class CrosswalkResult {
 public:
    CrosswalkResult() = default;
    void Init(uint16_t width, uint16_t height) {
        data.width = width;
        data.height = height;
        data.bit_map.resize(data.width * data.height, false);
    }

 public:
    Crosswalk data;
    int32_t id = -1;
};

/*
 * @brief: Package of road region
 */
class RoadRegionResult {
 public:
    RoadRegionResult() {}
    explicit RoadRegionResult(const RoadRegion& stopline_data,
                              int32_t stopline_id = -1)
        : data(stopline_data), id(stopline_id) {}
    RoadRegionResult(const RoadRegionResult& stopline_result) = default;

 public:
    RoadRegion data;
    int32_t id = -1;
};

typedef struct StructRoadStruct {
    std::vector<LaneLineResult> laneline_results;
    std::vector<StopLineResult> stopline_results;
    std::vector<SpeedBumpResult> speed_bump_results;
    std::vector<BoundaryResult> boundary_results;
    std::vector<FreespaceResult> freespace_results;
    std::vector<RoadRegionResult> road_region_results;
    std::vector<CrosswalkResult> crosswalk_results;
    std::vector<TrafficLight> traffic_light_results;
    std::vector<TrafficSign> traffic_sign_results;
    std::vector<RoadMarker> roadmarker_results;
    std::vector<BoardPole> board_pole_results;
    std::vector<CenterLineResult> centerline_results;
} RoadStruct;

typedef struct StructRoadGeometry {
    std::vector<LaneLineResult> laneline_results;
    std::vector<StopLineResult> stopline_results;
    std::vector<FreespaceResult> freespace_results;
} RoadGeometry;

typedef struct StructRoadGeometryFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @brief timestamp of current frame
    uint64_t timestamp_ns;
    RoadGeometry road_geometry;

    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        road_geometry.laneline_results.clear();
        road_geometry.stopline_results.clear();
        road_geometry.freespace_results.clear();
    }
} RoadGeometryFrame;

typedef struct StructPerceptionRoadFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @brief timestamp of current frame
    uint64_t timestamp_ns;
    RoadStruct road_struct;

    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        road_struct.laneline_results.clear();
        road_struct.stopline_results.clear();
        road_struct.speed_bump_results.clear();
        road_struct.boundary_results.clear();
        road_struct.freespace_results.clear();
        road_struct.road_region_results.clear();
        road_struct.traffic_light_results.clear();
        road_struct.traffic_sign_results.clear();
        road_struct.centerline_results.clear();
    }
} PerceptionRoadFrame;

}  // namespace perception
}  // namespace senseAD
