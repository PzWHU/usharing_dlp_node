// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
// Purpose:
//   Preserve RoadGeometryFrame -> sapPerceptionRoadFrame mapping without
//   RSCL/SWCFL convertor registration dependencies.

#include "transport/rscl/converters/road_geometry_frame_converter.hpp"

#include <cstddef>
#include <cstdint>

namespace usharing_dlp_node {
namespace senseauto {

using RoadGeometryFrame = ::senseAD::msg::perception::RoadGeometryFrame;

namespace {

uint8_t Msg2SapLaneLineType(uint8_t msg_laneline_type) {
  // Source mapping:
  //   Msg2PerceptionLaneLineType()
  //   adapter/perception_camera/include/message_helper/message_road_geometry_label.hpp
  // Difference from source:
  //   Return the numeric sapLaneLineType-compatible value directly to avoid
  //   pulling perception_common and SWCFL logging into the standalone port.
  enum class MsgLaneLineType : uint8_t {
    kNone = 0,
    kSingleDashed,
    kSingleSolid,
    kLeftDashedRightSolid,
    kLeftSolidRightDashed,
    kDoubleSolid,
    kDoubleDashed,
    kSawtoothSolid,
    kFishboneDashed,
    kFishboneSolid,
    kRoadEdge,
    kFence,
    kShortThickDashed,
    kUnknown,
  };

  switch (static_cast<MsgLaneLineType>(msg_laneline_type)) {
    case MsgLaneLineType::kSingleDashed:
      return static_cast<uint8_t>(LANE_LINE_SINGLE_DASHED);
    case MsgLaneLineType::kSingleSolid:
      return static_cast<uint8_t>(LANE_LINE_SINGLE_SOLID);
    case MsgLaneLineType::kLeftDashedRightSolid:
      return static_cast<uint8_t>(LANE_LINE_LEFT_DASHED_RIGHT_SOLID);
    case MsgLaneLineType::kLeftSolidRightDashed:
      return static_cast<uint8_t>(LANE_LINE_LEFT_SOLID_RIGHT_DASHED);
    case MsgLaneLineType::kDoubleSolid:
      return static_cast<uint8_t>(LANE_LINE_DOUBLE_SOLID);
    case MsgLaneLineType::kDoubleDashed:
      return static_cast<uint8_t>(LANE_LINE_DOUBLE_DASHED);
    case MsgLaneLineType::kFishboneDashed:
      return static_cast<uint8_t>(LANE_LINE_SINGLE_DASHED) | (1U << 5);
    case MsgLaneLineType::kFishboneSolid:
      return static_cast<uint8_t>(LANE_LINE_SINGLE_SOLID) | (1U << 5);
    case MsgLaneLineType::kRoadEdge:
      return static_cast<uint8_t>(LANE_LINE_ROAD_EDGE);
    case MsgLaneLineType::kShortThickDashed:
      return static_cast<uint8_t>(LANE_LINE_SHORT_THICK_DASHED);
    case MsgLaneLineType::kNone:
    case MsgLaneLineType::kSawtoothSolid:
    case MsgLaneLineType::kFence:
    case MsgLaneLineType::kUnknown:
    default:
      return static_cast<uint8_t>(LANE_LINE_UNKNOWN);
  }
}

uint8_t* AllocateBytes(std::size_t size) {
  return size > 0 ? new uint8_t[size]() : nullptr;
}

// RoadFrame 中很多数组字段在旧 convertor 中共用连续内存；本文件保留
// 该布局，并依赖 ReleaseRoadFrameMemory 按 SDK 规则释放。
template <typename T>
T* TakeBuffer(uint8_t** ptr, std::size_t count) {
  if (ptr == nullptr || *ptr == nullptr || count == 0) {
    return nullptr;
  }
  auto* value = reinterpret_cast<T*>(*ptr);
  *ptr += sizeof(T) * count;
  return value;
}

bool ConvertLaneLine(const ::senseAD::msg::perception::LaneLine::Reader& msg,
                     sapLaneLineResult* lane_line) {
  if (lane_line == nullptr) {
    return false;
  }

  lane_line->position_index = static_cast<int16_t>(msg.getId());
  lane_line->track_id = static_cast<int>(msg.getTrackId());
  lane_line->track_age = static_cast<int>(msg.getTrackAge());
  lane_line->distance_to_left_bumper = msg.getDistanceToLeftBumper();
  lane_line->distance_to_right_bumper = msg.getDistanceToRightBumper();
  lane_line->laneline_width.width =
      static_cast<uint8_t>(msg.getLanelineWidth());

  lane_line->data.color = static_cast<int16_t>(msg.getColorId());
  lane_line->data.laneline_type =
      Msg2SapLaneLineType(msg.getLanelineTypeId());
  lane_line->data.edge_type = static_cast<int8_t>(msg.getEdgeTypeId());
  lane_line->data.lane_confidence = msg.getLaneConfidence();
  lane_line->data.color_confidence = msg.getColorConfidence();
  lane_line->data.laneline_type_confidence =
      msg.getLanelineTypeConfidence();
  lane_line->data.edge_type_confidence = msg.getEdgeTypeConfidence();

  const auto start_point = msg.getStartPoint();
  lane_line->data.start_point.x = start_point.getX();
  lane_line->data.start_point.y = start_point.getY();
  const auto end_point = msg.getEndPoint();
  lane_line->data.end_point.x = end_point.getX();
  lane_line->data.end_point.y = end_point.getY();

  lane_line->data.image_point_num = msg.getImagePoints().size();
  lane_line->data.world_point_num = msg.getWorldPointsReproj().size();
  lane_line->data.merge_split_point_num = msg.getJunctionPoints().size();
  lane_line->data.bv_fitting_results.polyfit_result.polyfit_coef_num =
      msg.getPolyfitCoefficients().size();

  const std::size_t image_points_size =
      sizeof(sapVector2f) * lane_line->data.image_point_num;
  const std::size_t world_points_size =
      sizeof(sapPoint2fExtended) * lane_line->data.world_point_num;
  const std::size_t merge_split_points_size =
      sizeof(sapMergeSplitPoint) * lane_line->data.merge_split_point_num;
  const std::size_t polyfit_coefs_size =
      sizeof(float) *
      lane_line->data.bv_fitting_results.polyfit_result.polyfit_coef_num;
  auto* buffer = AllocateBytes(image_points_size + world_points_size +
                               merge_split_points_size +
                               polyfit_coefs_size);
  auto* ptr = buffer;
  lane_line->data.image_points =
      TakeBuffer<sapVector2f>(&ptr, lane_line->data.image_point_num);
  lane_line->data.world_points_reproj =
      TakeBuffer<sapPoint2fExtended>(&ptr,
                                     lane_line->data.world_point_num);
  lane_line->data.merge_split_points =
      TakeBuffer<sapMergeSplitPoint>(&ptr,
                                     lane_line->data.merge_split_point_num);
  lane_line->data.bv_fitting_results.polyfit_result.polyfit_coefficients =
      TakeBuffer<float>(
          &ptr,
          lane_line->data.bv_fitting_results.polyfit_result.polyfit_coef_num);

  const auto image_points = msg.getImagePoints();
  for (uint64_t i = 0; i < lane_line->data.image_point_num; ++i) {
    lane_line->data.image_points[i].x = image_points[i].getX();
    lane_line->data.image_points[i].y = image_points[i].getY();
  }

  const auto world_points = msg.getWorldPointsReproj();
  const auto world_conf = msg.getWorldPointsReprojConfidence();
  for (uint64_t i = 0; i < lane_line->data.world_point_num; ++i) {
    lane_line->data.world_points_reproj[i].x = world_points[i].getX();
    lane_line->data.world_points_reproj[i].y = world_points[i].getY();
    lane_line->data.world_points_reproj[i].geometric_confidence =
        i < world_conf.size() ? world_conf[i] : 1.0f;
  }

  const auto junction_points = msg.getJunctionPoints();
  for (uint64_t i = 0; i < lane_line->data.merge_split_point_num; ++i) {
    lane_line->data.merge_split_points[i].x = junction_points[i].getX();
    lane_line->data.merge_split_points[i].y = junction_points[i].getY();
    lane_line->data.merge_split_points[i].point_type =
        junction_points[i].getPointType() & 0x0F;
    lane_line->data.merge_split_points[i].next_segment_laneline_type =
        static_cast<uint16_t>(
            junction_points[i].getNextSegmentLaneLineType());
    lane_line->data.merge_split_points[i].next_segment_laneline_color =
        static_cast<int16_t>(
            junction_points[i].getNextSegmentLaneLineColor());
    lane_line->data.merge_split_points[i].next_segment_laneline_width.width =
        static_cast<uint8_t>(
            junction_points[i].getNextSegmentLaneLineWidth());
  }

  lane_line->data.bv_fitting_results.polyfit_result.fitting_degree =
      msg.getFittingDegree();
  const auto polyfit_coefs = msg.getPolyfitCoefficients();
  for (uint64_t i = 0;
       i < lane_line->data.bv_fitting_results.polyfit_result.polyfit_coef_num;
       ++i) {
    lane_line->data.bv_fitting_results.polyfit_result
        .polyfit_coefficients[i] = polyfit_coefs[i];
  }
  lane_line->data.line_junction.is_valid = msg.getIsLineJunctionValid();
  return true;
}

bool ConvertStopLine(const ::senseAD::msg::perception::StopLine::Reader& msg,
                     sapStopLineResult* stop_line) {
  if (stop_line == nullptr) {
    return false;
  }

  stop_line->id = msg.getId();
  stop_line->image_point_num = msg.getImagePoints().size();
  stop_line->world_point_num = msg.getWorldPoints().size();
  stop_line->conf_num = msg.getConfidence().size();

  auto* buffer = AllocateBytes(sizeof(sapVector2f) *
                                   stop_line->image_point_num +
                               sizeof(sapVector2f) *
                                   stop_line->world_point_num +
                               sizeof(float) * stop_line->conf_num);
  auto* ptr = buffer;
  stop_line->image_points =
      TakeBuffer<sapVector2f>(&ptr, stop_line->image_point_num);
  stop_line->world_points =
      TakeBuffer<sapVector2f>(&ptr, stop_line->world_point_num);
  stop_line->confidence = TakeBuffer<float>(&ptr, stop_line->conf_num);

  const auto image_points = msg.getImagePoints();
  for (uint64_t i = 0; i < stop_line->image_point_num; ++i) {
    stop_line->image_points[i].x = image_points[i].getX();
    stop_line->image_points[i].y = image_points[i].getY();
  }
  const auto world_points = msg.getWorldPoints();
  for (uint64_t i = 0; i < stop_line->world_point_num; ++i) {
    stop_line->world_points[i].x = world_points[i].getX();
    stop_line->world_points[i].y = world_points[i].getY();
  }
  const auto confidence = msg.getConfidence();
  for (uint64_t i = 0; i < stop_line->conf_num; ++i) {
    stop_line->confidence[i] = confidence[i];
  }
  return true;
}

bool ConvertFreespace(const ::senseAD::msg::perception::Freespace::Reader& msg,
                      sapFreespaceResult* freespace) {
  if (freespace == nullptr) {
    return false;
  }

  freespace->id = msg.getId();
  freespace->image_point_num = msg.getImagePoints().size();
  freespace->world_point_num = msg.getWorldPoints().size();
  freespace->width = static_cast<uint16_t>(msg.getCol());
  freespace->height = static_cast<uint16_t>(msg.getRow());
  freespace->resolution = msg.getResolution();

  const std::size_t bitmap_size =
      static_cast<std::size_t>(freespace->width) * freespace->height;
  if (msg.getOccupancyGrid().size() != bitmap_size) {
    return false;
  }

  auto* buffer = AllocateBytes(sizeof(sapVector2f) *
                                   freespace->image_point_num +
                               sizeof(sapVector2f) *
                                   freespace->world_point_num +
                               sizeof(bool) * bitmap_size);
  auto* ptr = buffer;
  freespace->image_points =
      TakeBuffer<sapVector2f>(&ptr, freespace->image_point_num);
  freespace->world_points =
      TakeBuffer<sapVector2f>(&ptr, freespace->world_point_num);
  freespace->bit_map = TakeBuffer<bool>(&ptr, bitmap_size);

  const auto image_points = msg.getImagePoints();
  for (uint64_t i = 0; i < freespace->image_point_num; ++i) {
    freespace->image_points[i].x = image_points[i].getX();
    freespace->image_points[i].y = image_points[i].getY();
  }
  const auto world_points = msg.getWorldPoints();
  for (uint64_t i = 0; i < freespace->world_point_num; ++i) {
    freespace->world_points[i].x = world_points[i].getX();
    freespace->world_points[i].y = world_points[i].getY();
  }
  const auto occupancy_grid = msg.getOccupancyGrid();
  for (std::size_t i = 0; i < bitmap_size; ++i) {
    freespace->bit_map[i] = occupancy_grid[i] != 0;
  }
  return true;
}

bool ConvertCrosswalk(const ::senseAD::msg::perception::Crosswalk::Reader& msg,
                      sapCrosswalkResult* crosswalk) {
  if (crosswalk == nullptr) {
    return false;
  }

  crosswalk->id = msg.getId();
  crosswalk->image_point_num = msg.getImagePoints().size();
  crosswalk->world_point_num = msg.getWorldPoints().size();
  crosswalk->conf_num = msg.getConfidence().size();
  crosswalk->width = msg.getWidth();
  crosswalk->height = msg.getHeight();
  crosswalk->bit_map_num =
      static_cast<uint64_t>(crosswalk->width) * crosswalk->height;
  if (msg.getBitMap().size() < crosswalk->bit_map_num) {
    return false;
  }

  auto* buffer = AllocateBytes(sizeof(sapPointInfo) *
                                   crosswalk->image_point_num +
                               sizeof(sapPointInfo) *
                                   crosswalk->world_point_num +
                               sizeof(sapConfidenceInfo) *
                                   crosswalk->conf_num +
                               sizeof(int8_t) * crosswalk->bit_map_num);
  auto* ptr = buffer;
  crosswalk->image_points =
      TakeBuffer<sapPointInfo>(&ptr, crosswalk->image_point_num);
  crosswalk->world_points =
      TakeBuffer<sapPointInfo>(&ptr, crosswalk->world_point_num);
  crosswalk->confidence =
      TakeBuffer<sapConfidenceInfo>(&ptr, crosswalk->conf_num);
  crosswalk->bit_map = TakeBuffer<int8_t>(&ptr, crosswalk->bit_map_num);

  std::size_t sub_buffer_size = 0;
  for (uint64_t i = 0; i < crosswalk->image_point_num; ++i) {
    crosswalk->image_points[i].num = msg.getImagePoints()[i].getP2d().size();
    sub_buffer_size +=
        sizeof(sapVector2f) * crosswalk->image_points[i].num;
  }
  for (uint64_t i = 0; i < crosswalk->world_point_num; ++i) {
    crosswalk->world_points[i].num = msg.getWorldPoints()[i].getP2d().size();
    sub_buffer_size +=
        sizeof(sapVector2f) * crosswalk->world_points[i].num;
  }
  for (uint64_t i = 0; i < crosswalk->conf_num; ++i) {
    crosswalk->confidence[i].num = msg.getConfidence()[i].getVector().size();
    sub_buffer_size += sizeof(float) * crosswalk->confidence[i].num;
  }

  auto* sub_buffer = AllocateBytes(sub_buffer_size);
  ptr = sub_buffer;
  for (uint64_t i = 0; i < crosswalk->image_point_num; ++i) {
    crosswalk->image_points[i].points =
        TakeBuffer<sapVector2f>(&ptr, crosswalk->image_points[i].num);
    const auto points = msg.getImagePoints()[i].getP2d();
    for (uint64_t j = 0; j < crosswalk->image_points[i].num; ++j) {
      crosswalk->image_points[i].points[j].x = points[j].getX();
      crosswalk->image_points[i].points[j].y = points[j].getY();
    }
  }
  for (uint64_t i = 0; i < crosswalk->world_point_num; ++i) {
    crosswalk->world_points[i].points =
        TakeBuffer<sapVector2f>(&ptr, crosswalk->world_points[i].num);
    const auto points = msg.getWorldPoints()[i].getP2d();
    for (uint64_t j = 0; j < crosswalk->world_points[i].num; ++j) {
      crosswalk->world_points[i].points[j].x = points[j].getX();
      crosswalk->world_points[i].points[j].y = points[j].getY();
    }
  }
  for (uint64_t i = 0; i < crosswalk->conf_num; ++i) {
    crosswalk->confidence[i].val =
        TakeBuffer<float>(&ptr, crosswalk->confidence[i].num);
    const auto values = msg.getConfidence()[i].getVector();
    for (uint64_t j = 0; j < crosswalk->confidence[i].num; ++j) {
      crosswalk->confidence[i].val[j] = values[j];
    }
  }
  const auto bitmap = msg.getBitMap();
  for (uint64_t i = 0; i < crosswalk->bit_map_num; ++i) {
    crosswalk->bit_map[i] = static_cast<int8_t>(bitmap[i]);
  }
  return true;
}

bool ConvertSpeedBump(
    const ::senseAD::msg::perception::SpeedBump::Reader& msg,
    sapSpeedBumpResult* speed_bump) {
  if (speed_bump == nullptr) {
    return false;
  }

  speed_bump->id = msg.getId();
  speed_bump->image_point_num = msg.getImagePoints().size();
  speed_bump->world_point_num = msg.getWorldPoints().size();
  speed_bump->conf_num = msg.getConfidence().size();

  auto* buffer = AllocateBytes(sizeof(sapVector2f) *
                                   speed_bump->image_point_num +
                               sizeof(sapVector2f) *
                                   speed_bump->world_point_num +
                               sizeof(float) * speed_bump->conf_num);
  auto* ptr = buffer;
  speed_bump->image_points =
      TakeBuffer<sapVector2f>(&ptr, speed_bump->image_point_num);
  speed_bump->world_points =
      TakeBuffer<sapVector2f>(&ptr, speed_bump->world_point_num);
  speed_bump->confidence = TakeBuffer<float>(&ptr, speed_bump->conf_num);

  const auto image_points = msg.getImagePoints();
  for (uint64_t i = 0; i < speed_bump->image_point_num; ++i) {
    speed_bump->image_points[i].x = image_points[i].getX();
    speed_bump->image_points[i].y = image_points[i].getY();
  }
  const auto world_points = msg.getWorldPoints();
  for (uint64_t i = 0; i < speed_bump->world_point_num; ++i) {
    speed_bump->world_points[i].x = world_points[i].getX();
    speed_bump->world_points[i].y = world_points[i].getY();
  }
  const auto confidence = msg.getConfidence();
  for (uint64_t i = 0; i < speed_bump->conf_num; ++i) {
    speed_bump->confidence[i] = confidence[i];
  }
  return true;
}

bool ConvertHolisticPath(
    const ::senseAD::msg::perception::HolisticPath::Reader& msg,
    sapHolisticPathResult* holistic_path) {
  if (holistic_path == nullptr) {
    return false;
  }

  holistic_path->confidence = msg.getConfidence();
  holistic_path->point_bv_num = msg.getPointsBv().size();
  holistic_path->track_id = static_cast<int>(msg.getTrackId());
  holistic_path->track_age = static_cast<int>(msg.getTrackAge());
  const auto start_point = msg.getStartPoint();
  holistic_path->start_point.x = start_point.getX();
  holistic_path->start_point.y = start_point.getY();
  const auto end_point = msg.getEndPoint();
  holistic_path->end_point.x = end_point.getX();
  holistic_path->end_point.y = end_point.getY();

  holistic_path->points_bv =
      holistic_path->point_bv_num > 0
          ? new sapVector2f[holistic_path->point_bv_num]()
          : nullptr;
  const auto points_bv = msg.getPointsBv();
  for (uint64_t i = 0; i < holistic_path->point_bv_num; ++i) {
    holistic_path->points_bv[i].x = points_bv[i].getX();
    holistic_path->points_bv[i].y = points_bv[i].getY();
  }
  return true;
}

bool ConvertRoadmarker(
    const ::senseAD::msg::perception::RoadMarker::Reader& msg,
    sapRoadmarkerResult* roadmarker) {
  if (roadmarker == nullptr) {
    return false;
  }

  roadmarker->id = msg.getId();
  roadmarker->image_point_num = msg.getImagePointNum();
  roadmarker->world_point_num = msg.getWorldPointNum();
  roadmarker->conf_num = msg.getConfNum();
  roadmarker->roadmarker_type = msg.getType();
  roadmarker->kp_types_num = msg.getKeyPointTypesNum();
  roadmarker->heading = msg.getHeading();

  auto* buffer = AllocateBytes(sizeof(sapVector2f) *
                                   roadmarker->image_point_num +
                               sizeof(sapVector2f) *
                                   roadmarker->world_point_num +
                               sizeof(float) * roadmarker->conf_num +
                               sizeof(int8_t) * roadmarker->kp_types_num);
  auto* ptr = buffer;
  roadmarker->image_points =
      TakeBuffer<sapVector2f>(&ptr, roadmarker->image_point_num);
  roadmarker->world_points =
      TakeBuffer<sapVector2f>(&ptr, roadmarker->world_point_num);
  roadmarker->confidence = TakeBuffer<float>(&ptr, roadmarker->conf_num);
  roadmarker->kp_types = TakeBuffer<int8_t>(&ptr, roadmarker->kp_types_num);

  const auto image_points = msg.getImagePoints();
  for (uint64_t i = 0; i < roadmarker->image_point_num; ++i) {
    roadmarker->image_points[i].x = image_points[i].getX();
    roadmarker->image_points[i].y = image_points[i].getY();
  }
  const auto world_points = msg.getWorldPoints();
  for (uint64_t i = 0; i < roadmarker->world_point_num; ++i) {
    roadmarker->world_points[i].x = world_points[i].getX();
    roadmarker->world_points[i].y = world_points[i].getY();
  }
  const auto confidence = msg.getConfidence();
  for (uint64_t i = 0; i < roadmarker->conf_num; ++i) {
    roadmarker->confidence[i] = confidence[i];
  }
  const auto key_point_types = msg.getKeyPointTypes();
  for (uint64_t i = 0; i < roadmarker->kp_types_num; ++i) {
    roadmarker->kp_types[i] = static_cast<int8_t>(key_point_types[i]);
  }
  return true;
}

bool ConvertRoadGeometryFrame(const RoadGeometryFrame::Reader& msg,
                              sapPerceptionRoadFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  frame->sensor_id = static_cast<uint16_t>(msg.getSensorId());
  frame->timestamp_ns = msg.getFrameTimestampNs();
  frame->sensor_send_timestamp_ns = 0;

  const auto road_geometry = msg.getRoadGeometry();
  const auto lanelines = road_geometry.getLanelineResults();
  const auto stoplines = road_geometry.getStoplineResults();
  const auto freespaces = road_geometry.getFreespaceResults();
  const auto crosswalks = road_geometry.getCrosswalkResults();
  const auto speedbumps = road_geometry.getSpeedBumpResults();
  const auto holistic_paths = road_geometry.getHolisticPathResults();
  const auto roadmarkers = road_geometry.getRoadMarkerResults();

  auto& road_struct = frame->road_struct;
  // RoadGeometryFrame 内含多类道路元素，先统计数量，再为各类结果数组
  // 分配一块连续内存，最后逐类填充。
  road_struct.laneline_num = static_cast<uint16_t>(lanelines.size());
  road_struct.stopline_num = static_cast<uint16_t>(stoplines.size());
  road_struct.freespace_num = static_cast<uint16_t>(freespaces.size());
  road_struct.crosswalk_num = static_cast<uint16_t>(crosswalks.size());
  road_struct.speedbump_num = static_cast<uint16_t>(speedbumps.size());
  road_struct.holistic_path_num =
      static_cast<uint16_t>(holistic_paths.size());
  road_struct.roadmarker_num = static_cast<uint16_t>(roadmarkers.size());

  auto* buffer = AllocateBytes(sizeof(sapLaneLineResult) *
                                   road_struct.laneline_num +
                               sizeof(sapStopLineResult) *
                                   road_struct.stopline_num +
                               sizeof(sapFreespaceResult) *
                                   road_struct.freespace_num +
                               sizeof(sapCrosswalkResult) *
                                   road_struct.crosswalk_num +
                               sizeof(sapSpeedBumpResult) *
                                   road_struct.speedbump_num +
                               sizeof(sapHolisticPathResult) *
                                   road_struct.holistic_path_num +
                               sizeof(sapRoadmarkerResult) *
                                   road_struct.roadmarker_num);
  auto* ptr = buffer;
  road_struct.laneline_results =
      TakeBuffer<sapLaneLineResult>(&ptr, road_struct.laneline_num);
  road_struct.stopline_results =
      TakeBuffer<sapStopLineResult>(&ptr, road_struct.stopline_num);
  road_struct.freespace_results =
      TakeBuffer<sapFreespaceResult>(&ptr, road_struct.freespace_num);
  road_struct.crosswalk_results =
      TakeBuffer<sapCrosswalkResult>(&ptr, road_struct.crosswalk_num);
  road_struct.speedbump_results =
      TakeBuffer<sapSpeedBumpResult>(&ptr, road_struct.speedbump_num);
  road_struct.holistic_path_results =
      TakeBuffer<sapHolisticPathResult>(&ptr,
                                        road_struct.holistic_path_num);
  road_struct.roadmarker_results =
      TakeBuffer<sapRoadmarkerResult>(&ptr, road_struct.roadmarker_num);

  for (uint64_t i = 0; i < road_struct.laneline_num; ++i) {
    if (!ConvertLaneLine(lanelines[i], &road_struct.laneline_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.stopline_num; ++i) {
    if (!ConvertStopLine(stoplines[i], &road_struct.stopline_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.freespace_num; ++i) {
    if (!ConvertFreespace(freespaces[i], &road_struct.freespace_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.crosswalk_num; ++i) {
    if (!ConvertCrosswalk(crosswalks[i], &road_struct.crosswalk_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.speedbump_num; ++i) {
    if (!ConvertSpeedBump(speedbumps[i],
                          &road_struct.speedbump_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.holistic_path_num; ++i) {
    if (!ConvertHolisticPath(holistic_paths[i],
                             &road_struct.holistic_path_results[i])) {
      return false;
    }
  }
  for (uint64_t i = 0; i < road_struct.roadmarker_num; ++i) {
    if (!ConvertRoadmarker(roadmarkers[i],
                           &road_struct.roadmarker_results[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool ConvertSenseAutoRoadGeometryFrameToSap(RoadGeometryFrame::Reader msg,
                                            SapRoadFramePtr* output) {
  if (output == nullptr) {
    return false;
  }

  auto* raw_frame = new sapPerceptionRoadFrame();
  InitsapPerceptionRoadFrame(raw_frame);
  // road frame 的内部数组使用 SDK ReleaseRoadFrameMemory 释放。
  auto frame = MakeSapInputPtr<sapPerceptionRoadFrame>(
      raw_frame, [](sapPerceptionRoadFrame* value) {
        ReleaseRoadFrameMemory(value);
      });
  if (!ConvertRoadGeometryFrame(msg, frame.get())) {
    return false;
  }
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
