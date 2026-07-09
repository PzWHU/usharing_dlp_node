// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_map_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_navi_topo_convertor.cpp
// Purpose:
//   Convert SceneNaviMap into sap SDMap/RoadStructure/CrossInfo/NaviTopo DTOs
//   without SWCFL multi-convertor fan-out.

#include "transport/rscl/converters/scene_navi_map_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "transport/rscl/converters/converter_common.hpp"

namespace usharing_dlp_node {
namespace senseauto {

using SceneNaviMap = ::senseAD::msg::maplesslm::SceneNaviMap;
using SDMap = ::senseAD::msg::sdmap::SDMap;

namespace {

using detail::AllocateBytes;
using detail::CopyCapnpText;
using detail::CopyPoint3f;
using detail::NewArrayOrNull;
using detail::TakeBuffer;

void ReleaseLaneData(sapLaneData* lane) {
  if (lane == nullptr) {
    return;
  }
  // sapLaneData 在 RoadStructure 和 NaviTopo 中复用，统一释放其内部数组。
  delete[] lane->successor_link_ids;
  delete[] lane->predecessor_link_ids;
  delete[] lane->center_points;
  delete[] lane->lane_widths;
  delete[] lane->longitudinal_slopes;
  delete[] lane->road_marker_ids;
}

bool ConvertLocalMapLane(const ::senseAD::msg::localmap::Lane::Reader& msg,
                         sapLaneData* lane, bool copy_longitudinal_slopes) {
  if (lane == nullptr) {
    return false;
  }

  lane->id = msg.getId();
  lane->section_id = msg.getSectionId();
  lane->sequence_id = msg.getSequenceId();
  lane->intersection_id = msg.getIntersectionId();
  lane->lane_type = static_cast<uint64_t>(msg.getLaneType());
  lane->turn_type = static_cast<uint64_t>(msg.getTurnType());
  lane->trans_type = static_cast<uint64_t>(msg.getTransType());
  lane->left_line_id = msg.getLeftLineId();
  lane->right_line_id = msg.getRightLineId();
  lane->left_lane_id = msg.getLeftLaneId();
  lane->right_lane_id = msg.getRightLaneId();
  lane->length = msg.getLength();

  const auto successor_ids = msg.getSuccessorLinkIds();
  lane->successor_link_ids_num = successor_ids.size();
  lane->successor_link_ids =
      NewArrayOrNull<uint64_t>(lane->successor_link_ids_num);
  for (uint64_t i = 0; i < lane->successor_link_ids_num; ++i) {
    lane->successor_link_ids[i] = successor_ids[i];
  }

  const auto predecessor_ids = msg.getPredecessorLinkIds();
  lane->predecessor_link_ids_num = predecessor_ids.size();
  lane->predecessor_link_ids =
      NewArrayOrNull<uint64_t>(lane->predecessor_link_ids_num);
  for (uint64_t i = 0; i < lane->predecessor_link_ids_num; ++i) {
    lane->predecessor_link_ids[i] = predecessor_ids[i];
  }

  const auto center_points = msg.getCenterPoints();
  lane->center_points_num = center_points.size();
  lane->center_points = NewArrayOrNull<sapPoint3f>(lane->center_points_num);
  for (uint64_t i = 0; i < lane->center_points_num; ++i) {
    CopyPoint3f(center_points[i], &lane->center_points[i]);
  }

  const auto lane_widths = msg.getLaneWidths();
  lane->lane_widths_num = lane_widths.size();
  lane->lane_widths = NewArrayOrNull<float>(lane->lane_widths_num);
  for (uint64_t i = 0; i < lane->lane_widths_num; ++i) {
    lane->lane_widths[i] = lane_widths[i];
  }

  // copy_longitudinal_slopes 区分 RoadStructure 与 NaviTopo 两个旧 convertor
  // 的行为：NaviTopo 需要纵坡，RoadStructure 不填该字段。
  if (copy_longitudinal_slopes) {
    const auto slopes = msg.getLongitudinalSlopes();
    lane->longitudinal_slopes_num = slopes.size();
    lane->longitudinal_slopes =
        NewArrayOrNull<float>(lane->longitudinal_slopes_num);
    for (uint64_t i = 0; i < lane->longitudinal_slopes_num; ++i) {
      lane->longitudinal_slopes[i] = slopes[i];
    }
  }

  const auto road_marker_ids = msg.getRoadMarkerIds();
  lane->road_marker_ids_num = road_marker_ids.size();
  lane->road_marker_ids =
      NewArrayOrNull<uint64_t>(lane->road_marker_ids_num);
  for (uint64_t i = 0; i < lane->road_marker_ids_num; ++i) {
    lane->road_marker_ids[i] = road_marker_ids[i];
  }
  return true;
}

bool ConvertRoadStructure(const SceneNaviMap::Reader& msg,
                          sapRoadStructure* road_struct) {
  if (road_struct == nullptr) {
    return false;
  }

  const auto source = msg.getRoadStructure();
  road_struct->timestamp_ns = source.getMapHeader().getTimestampNs();
  road_struct->cur_status = msg.getVehicleState().getCurLaneId();

  auto& semantic = road_struct->semantic_map_data;
  const auto lines = source.getLines();
  semantic.lines_num = lines.size();
  semantic.lines = NewArrayOrNull<sapLineData>(semantic.lines_num);
  for (uint64_t i = 0; i < semantic.lines_num; ++i) {
    auto& line = semantic.lines[i];
    const auto msg_line = lines[i];
    line.id = msg_line.getId();
    line.left_lane_id = msg_line.getLeftLaneId();
    line.right_lane_id = msg_line.getRightLaneId();
    line.predecessor_laneline_id = msg_line.getPredecessorLanelineId();
    line.successor_laneline_id = msg_line.getSuccessorLanelineId();

    const auto segments = msg_line.getLineSegments();
    line.line_segments_num = segments.size();
    line.line_segments =
        NewArrayOrNull<sapLineSegmentData>(line.line_segments_num);
    for (uint64_t j = 0; j < line.line_segments_num; ++j) {
      auto& segment = line.line_segments[j];
      const auto msg_segment = segments[j];
      segment.line_type = static_cast<uint64_t>(msg_segment.getType());
      segment.line_style = static_cast<uint64_t>(msg_segment.getStyle());
      segment.color = static_cast<uint64_t>(msg_segment.getColor());
      segment.width = msg_segment.getWidth();
      const auto points = msg_segment.getPoints();
      segment.points_num = points.size();
      segment.points = NewArrayOrNull<sapPoint3f>(segment.points_num);
      for (uint64_t k = 0; k < segment.points_num; ++k) {
        CopyPoint3f(points[k], &segment.points[k]);
      }
    }
  }

  const auto road_markers = source.getRoadMarkers();
  semantic.road_markers_num = road_markers.size();
  semantic.road_markers =
      NewArrayOrNull<sapRoadMarkerData>(semantic.road_markers_num);
  for (uint64_t i = 0; i < semantic.road_markers_num; ++i) {
    auto& marker = semantic.road_markers[i];
    const auto msg_marker = road_markers[i];
    marker.id = msg_marker.getId();
    marker.roadmarker_type = static_cast<uint64_t>(msg_marker.getType());
    marker.color = static_cast<uint64_t>(msg_marker.getColor());
    CopyCapnpText(msg_marker.getSemantic(), marker.semantic);
    CopyPoint3f(msg_marker.getCentroid(), &marker.centroid);
    marker.lane_id = static_cast<uint64_t>(msg_marker.getLaneId());
  }

  auto& route = road_struct->route_map_data;
  const auto lanes = source.getLanes();
  route.lanes_num = lanes.size();
  route.lanes = NewArrayOrNull<sapLaneData>(route.lanes_num);
  for (uint64_t i = 0; i < route.lanes_num; ++i) {
    if (!ConvertLocalMapLane(lanes[i], &route.lanes[i], false)) {
      return false;
    }
  }

  const auto lane_links = source.getLaneLinks();
  route.lane_links_num = lane_links.size();
  route.lane_links = NewArrayOrNull<sapLaneLinkData>(route.lane_links_num);
  for (uint64_t i = 0; i < route.lane_links_num; ++i) {
    auto& lane_link = route.lane_links[i];
    const auto msg_link = lane_links[i];
    lane_link.id = msg_link.getId();
    lane_link.from_lane_id = msg_link.getFromLaneId();
    lane_link.to_lane_id = msg_link.getToLaneId();
    lane_link.link_type = static_cast<uint64_t>(msg_link.getLinkType());
    const auto points = msg_link.getRefPoints();
    lane_link.ref_points_num = points.size();
    lane_link.ref_points = NewArrayOrNull<sapPoint3f>(lane_link.ref_points_num);
    for (uint64_t j = 0; j < lane_link.ref_points_num; ++j) {
      CopyPoint3f(points[j], &lane_link.ref_points[j]);
    }
  }

  const auto sections = source.getSections();
  route.sections_num = sections.size();
  route.sections = NewArrayOrNull<sapSectionData>(route.sections_num);
  for (uint64_t i = 0; i < route.sections_num; ++i) {
    auto& section = route.sections[i];
    const auto msg_section = sections[i];
    section.id = msg_section.getId();
    section.section_class = static_cast<uint64_t>(msg_section.getSectionClass());
    section.section_type = static_cast<uint64_t>(msg_section.getSectionType());
    section.intersection_id = msg_section.getIntersectionId();
    const auto lane_ids = msg_section.getLaneIds();
    section.lane_ids_num = lane_ids.size();
    section.lane_ids = NewArrayOrNull<uint64_t>(section.lane_ids_num);
    for (uint64_t j = 0; j < section.lane_ids_num; ++j) {
      section.lane_ids[j] = lane_ids[j];
    }
  }

  return true;
}

void ReleaseRoadStructure(sapRoadStructure* road_struct) {
  if (road_struct == nullptr) {
    return;
  }
  // RoadStructure 包含 semantic_map_data 和 route_map_data 两部分，释放时
  // 需要递归释放 line segment、lane、lane link、section 的内部数组。
  auto& semantic = road_struct->semantic_map_data;
  for (uint64_t i = 0; i < semantic.lines_num; ++i) {
    for (uint64_t j = 0; j < semantic.lines[i].line_segments_num; ++j) {
      delete[] semantic.lines[i].line_segments[j].points;
    }
    delete[] semantic.lines[i].line_segments;
  }
  delete[] semantic.lines;
  delete[] semantic.road_markers;

  auto& route = road_struct->route_map_data;
  for (uint64_t i = 0; i < route.lanes_num; ++i) {
    ReleaseLaneData(&route.lanes[i]);
  }
  delete[] route.lanes;
  for (uint64_t i = 0; i < route.lane_links_num; ++i) {
    delete[] route.lane_links[i].ref_points;
  }
  delete[] route.lane_links;
  for (uint64_t i = 0; i < route.sections_num; ++i) {
    delete[] route.sections[i].lane_ids;
  }
  delete[] route.sections;
}

bool ConvertNaviTopo(const SceneNaviMap::Reader& msg,
                     sapNaviTopo* navi_topo) {
  if (navi_topo == nullptr) {
    return false;
  }

  const auto source = msg.getNaviTopo();
  navi_topo->timestamp_ns = source.getMapHeader().getTimestampNs();
  navi_topo->cur_status = msg.getVehicleState().getCurLaneId();

  const auto lanes = source.getLanes();
  navi_topo->lanes_num = lanes.size();
  navi_topo->lanes = NewArrayOrNull<sapLaneData>(navi_topo->lanes_num);
  for (uint64_t i = 0; i < navi_topo->lanes_num; ++i) {
    if (!ConvertLocalMapLane(lanes[i], &navi_topo->lanes[i], true)) {
      return false;
    }
  }
  return true;
}

void ReleaseNaviTopo(sapNaviTopo* navi_topo) {
  if (navi_topo == nullptr) {
    return;
  }
  for (uint64_t i = 0; i < navi_topo->lanes_num; ++i) {
    ReleaseLaneData(&navi_topo->lanes[i]);
  }
  delete[] navi_topo->lanes;
}

void ReleaseCrossInfo(sapCrossInfo* cross_info) {
  if (cross_info == nullptr) {
    return;
  }
  // CrossInfo 使用多级二维数组表达路口线、中心线和停止线，必须先释放
  // 每条线的点数组，再释放外层数组。
  delete[] cross_info->lines_idx;
  for (uint32_t i = 0; i < cross_info->lines_num; ++i) {
    delete[] cross_info->lines_pts[i];
  }
  delete[] cross_info->lines_pts_num;
  delete[] cross_info->lines_pts;
  for (uint32_t i = 0; i < cross_info->lanecenter_num; ++i) {
    delete[] cross_info->lanecenter_pts[i];
  }
  delete[] cross_info->lanecenter_pts_num;
  delete[] cross_info->lanecenter_pts;
  for (uint32_t i = 0; i < cross_info->stoplines_num; ++i) {
    delete[] cross_info->stoplines[i].laneids;
    delete[] cross_info->stoplines[i].pts;
  }
  delete[] cross_info->stoplines;
  delete[] cross_info->lane_type;
  delete[] cross_info->next_lane_type;
}

bool ConvertCrossInfo(const SceneNaviMap::Reader& msg,
                      sapCrossInfo* cross_info) {
  if (cross_info == nullptr) {
    return false;
  }

  const auto road_struct = msg.getRoadStructure();
  const uint64_t ego_lane_id = msg.getVehicleState().getCurLaneId();
  cross_info->timestamp_ns = road_struct.getMapHeader().getTimestampNs();
  cross_info->ego_line_id = static_cast<uint32_t>(ego_lane_id);
  // 以下默认值保持旧 LocalMapConvertor 行为，表示没有可用交通灯中心/停车点。
  cross_info->stop_x = static_cast<uint32_t>(-10000);
  cross_info->ego_lane_traffic_light = 1;
  cross_info->light_center.x = 1000.0f;
  cross_info->light_center.y = 1000.0f;

  const auto lines = road_struct.getLines();
  cross_info->lines_idx_num = lines.size();
  cross_info->lines_idx = NewArrayOrNull<uint64_t>(cross_info->lines_idx_num);
  cross_info->lines_num = static_cast<uint32_t>(lines.size());
  cross_info->lines_pts =
      NewArrayOrNull<sapPairPoint*>(cross_info->lines_num);
  cross_info->lines_pts_num =
      NewArrayOrNull<uint32_t>(cross_info->lines_num);
  for (uint32_t i = 0; i < cross_info->lines_num; ++i) {
    cross_info->lines_idx[i] = lines[i].getId();
    uint64_t total_points = 0;
    for (const auto segment : lines[i].getLineSegments()) {
      total_points += segment.getPoints().size();
    }
    cross_info->lines_pts_num[i] = static_cast<uint32_t>(total_points);
    cross_info->lines_pts[i] = NewArrayOrNull<sapPairPoint>(total_points);
    uint64_t point_index = 0;
    for (const auto segment : lines[i].getLineSegments()) {
      for (const auto point : segment.getPoints()) {
        cross_info->lines_pts[i][point_index].key = point.getX();
        cross_info->lines_pts[i][point_index].value = point.getY();
        ++point_index;
      }
    }
  }

  const auto lanes = road_struct.getLanes();
  cross_info->lanecenter_num = static_cast<uint32_t>(lanes.size());
  cross_info->lanecenter_pts =
      NewArrayOrNull<sapPairPoint*>(cross_info->lanecenter_num);
  cross_info->lanecenter_pts_num =
      NewArrayOrNull<uint32_t>(cross_info->lanecenter_num);

  for (uint32_t i = 0; i < cross_info->lanecenter_num; ++i) {
    const auto center_points = lanes[i].getCenterPoints();
    cross_info->lanecenter_pts_num[i] =
        static_cast<uint32_t>(center_points.size());
    cross_info->lanecenter_pts[i] =
        NewArrayOrNull<sapPairPoint>(center_points.size());
    for (uint64_t j = 0; j < center_points.size(); ++j) {
      cross_info->lanecenter_pts[i][j].key = center_points[j].getX();
      cross_info->lanecenter_pts[i][j].value = center_points[j].getY();
    }
  }

  uint64_t ego_index = 0;
  bool found_ego_lane = false;
  uint32_t ego_lane_turn_type = 0;
  uint32_t ego_lane_trans_type = 0;
  for (uint64_t i = 0; i < lanes.size(); ++i) {
    if (lanes[i].getId() == ego_lane_id) {
      ego_index = i;
      ego_lane_turn_type = static_cast<uint32_t>(lanes[i].getTurnType());
      ego_lane_trans_type = static_cast<uint32_t>(lanes[i].getTransType());
      found_ego_lane = true;
      break;
    }
  }
  cross_info->lane_type_num = found_ego_lane ? 1 : 0;
  cross_info->lane_type = NewArrayOrNull<uint32_t>(cross_info->lane_type_num);
  cross_info->next_lane_type_num = found_ego_lane ? 1 : 0;
  cross_info->next_lane_type =
      NewArrayOrNull<uint32_t>(cross_info->next_lane_type_num);
  if (found_ego_lane) {
    cross_info->ego_line_id = static_cast<uint32_t>(ego_index);
    cross_info->lane_type[0] = ego_lane_turn_type;
    cross_info->next_lane_type[0] = ego_lane_trans_type;
  }

  const auto traffic_lights = road_struct.getTrafficLights();
  cross_info->lights_flag = traffic_lights.size() > 0;
  for (const auto light : traffic_lights) {
    bool applies_to_ego = false;
    for (const auto lane_id : light.getLaneIdVector()) {
      if (lane_id == ego_lane_id) {
        applies_to_ego = true;
        break;
      }
    }
    if (!applies_to_ego) {
      continue;
    }
    cross_info->light_center.x = light.getCentroid().getX();
    cross_info->light_center.y = light.getCentroid().getY();
    for (const auto signal : light.getSignals()) {
      int status = static_cast<int>(signal.getStatus());
      if (status == 6) status = 3;
      if (status == 7) status = 4;
      if (status == 8) status = 5;
      if (status == 9) status = 1;
      cross_info->ego_lane_traffic_light = static_cast<uint32_t>(status);
      cross_info->red_light = status == 4 || status == 5;
      break;
    }
    break;
  }

  const auto stop_lines = road_struct.getStopLines();
  cross_info->stoplines_num = static_cast<uint32_t>(stop_lines.size());
  cross_info->stoplines =
      NewArrayOrNull<sapVDStopLine>(cross_info->stoplines_num);
  float nearest_stop_x = 1000.0f;
  bool found_stop_x = false;
  for (uint32_t i = 0; i < cross_info->stoplines_num; ++i) {
    const auto lane_ids = stop_lines[i].getLaneIds();
    cross_info->stoplines[i].laneids_num = lane_ids.size();
    cross_info->stoplines[i].laneids =
        NewArrayOrNull<uint64_t>(lane_ids.size());
    bool applies_to_ego = false;
    for (uint64_t j = 0; j < lane_ids.size(); ++j) {
      cross_info->stoplines[i].laneids[j] = lane_ids[j];
      applies_to_ego = applies_to_ego || lane_ids[j] == ego_lane_id;
    }

    const auto points = stop_lines[i].getPoints();
    cross_info->stoplines[i].pts_num = points.size();
    cross_info->stoplines[i].pts = NewArrayOrNull<sapPairPoint>(points.size());
    for (uint64_t j = 0; j < points.size(); ++j) {
      cross_info->stoplines[i].pts[j].key = points[j].getX();
      cross_info->stoplines[i].pts[j].value = points[j].getY();
    }
    if (applies_to_ego && points.size() > 0 && points[0].getX() > -50.0f &&
        points[0].getX() < nearest_stop_x) {
      nearest_stop_x = points[0].getX();
      found_stop_x = true;
    }
  }
  if (found_stop_x) {
    cross_info->stop_x = static_cast<uint32_t>(10 * nearest_stop_x);
  }
  return true;
}

void CopySapLinkID(const ::senseAD::msg::sdmap::LinkIDType::Reader& source,
                   sapLinkIDType* target) {
  target->tile_id = source.getTileID();
  target->link_id = source.getLinkID();
  target->raw_id = source.getRawID();
}

bool ConvertSDMapLink(const ::senseAD::msg::sdmap::Link::Reader& msg,
                      sapLink* link) {
  if (link == nullptr) {
    return false;
  }

  CopySapLinkID(msg.getLinkID(), &link->link_id);
  link->lane_num_sum = msg.getLaneNumSum();
  link->link_direction = msg.getLinkDirection();
  link->speed_limit_s2e = msg.getSpeedLimitS2E();
  link->speed_limit_e2s = msg.getSpeedLimitE2S();
  link->speed_class_type = msg.getSpeedClassType();
  link->start_angle = msg.getStartAngle();
  link->end_angle = msg.getEndAngle();
  link->road_class_type = msg.getRoadClassType();

  const auto road_kinds = msg.getRoadKindTypes();
  const auto geometry = msg.getGeometry();
  const auto curvatures = msg.getLinkCurvatures();
  const auto guidance_infos = msg.getGuidanceInfos();
  const auto next_ids = msg.getNextLinkIDs();
  const auto previous_ids = msg.getPreviousLinkIDs();

  std::size_t lane_infos_num = 0;
  for (const auto guidance : guidance_infos) {
    lane_infos_num += guidance.getLaneInfos().size();
  }

  auto* buffer = AllocateBytes(
      sizeof(uint64_t) * road_kinds.size() +
      sizeof(sapPointLLH) * geometry.size() +
      sizeof(sapPointENU) * geometry.size() +
      sizeof(sapLinkCurvature) * curvatures.size() +
      sizeof(sapLaneInfoMeta) * guidance_infos.size() +
      sizeof(sapLaneTrafficInfo) * lane_infos_num +
      sizeof(sapLinkIDType) * next_ids.size() +
      sizeof(sapLinkIDType) * previous_ids.size());
  auto* ptr = buffer;
  link->road_kind_types = TakeBuffer<uint64_t>(&ptr, road_kinds.size());
  link->road_kind_types_num = road_kinds.size();
  for (uint64_t i = 0; i < link->road_kind_types_num; ++i) {
    link->road_kind_types[i] = road_kinds[i];
  }

  link->geometry = TakeBuffer<sapPointLLH>(&ptr, geometry.size());
  link->geometry_num = geometry.size();
  link->geometry_lcal = TakeBuffer<sapPointENU>(&ptr, geometry.size());
  link->geometry_lcal_num = geometry.size();
  for (uint64_t i = 0; i < link->geometry_num; ++i) {
    link->geometry[i].lon = geometry[i].getX();
    link->geometry[i].lat = geometry[i].getY();
    link->geometry[i].height = geometry[i].getZ();
    link->geometry_lcal[i].x = geometry[i].getX();
    link->geometry_lcal[i].y = geometry[i].getY();
    link->geometry_lcal[i].z = geometry[i].getZ();
  }

  link->link_curvature =
      TakeBuffer<sapLinkCurvature>(&ptr, curvatures.size());
  link->link_curvature_num = curvatures.size();
  for (uint64_t i = 0; i < link->link_curvature_num; ++i) {
    link->link_curvature[i].offset = curvatures[i].getOffset();
    link->link_curvature[i].curvature = curvatures[i].getCurvature();
  }

  link->guidance_infos =
      TakeBuffer<sapLaneInfoMeta>(&ptr, guidance_infos.size());
  link->guidance_infos_num = guidance_infos.size();
  for (uint64_t i = 0; i < link->guidance_infos_num; ++i) {
    const auto lane_infos = guidance_infos[i].getLaneInfos();
    link->guidance_infos[i].lane_num = guidance_infos[i].getLaneNum();
    link->guidance_infos[i].lane_infos =
        TakeBuffer<sapLaneTrafficInfo>(&ptr, lane_infos.size());
    link->guidance_infos[i].lane_infos_num = lane_infos.size();
    for (uint64_t j = 0; j < lane_infos.size(); ++j) {
      link->guidance_infos[i].lane_infos[j].is_bus =
          lane_infos[j].getIsBus();
      link->guidance_infos[i].lane_infos[j].lane_type =
          lane_infos[j].getLaneType();
      link->guidance_infos[i].lane_infos[j].arrow =
          lane_infos[j].getArrow();
    }
  }

  link->next_link_ids = TakeBuffer<sapLinkIDType>(&ptr, next_ids.size());
  link->next_link_ids_num = next_ids.size();
  for (uint64_t i = 0; i < link->next_link_ids_num; ++i) {
    CopySapLinkID(next_ids[i], &link->next_link_ids[i]);
  }
  link->previous_link_ids =
      TakeBuffer<sapLinkIDType>(&ptr, previous_ids.size());
  link->previous_link_ids_num = previous_ids.size();
  for (uint64_t i = 0; i < link->previous_link_ids_num; ++i) {
    CopySapLinkID(previous_ids[i], &link->previous_link_ids[i]);
  }
  return true;
}

void ReleaseSDMapLink(sapLink* link) {
  if (link == nullptr) {
    return;
  }
  auto* buffer = reinterpret_cast<uint8_t*>(link->road_kind_types);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->geometry);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->geometry_lcal);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->link_curvature);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->guidance_infos);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->next_link_ids);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(link->previous_link_ids);
  delete[] buffer;
}

void CopySapNodeID(const ::senseAD::msg::sdmap::NodeIDType::Reader& source,
                   sapNodeIDType* target) {
  target->tile_id = source.getTileID();
  target->node_id = source.getNodeID();
}

bool ConvertSDMapLinkNode(
    const ::senseAD::msg::sdmap::LinkNode::Reader& msg,
    sapLinkNode* node) {
  if (node == nullptr) {
    return false;
  }
  CopySapNodeID(msg.getNodeID(), &node->node_id);
  node->position.lon = msg.getPosition().getX();
  node->position.lat = msg.getPosition().getY();
  node->position.height = msg.getPosition().getZ();
  node->position_local.x = msg.getPosition().getX();
  node->position_local.y = msg.getPosition().getY();
  node->position_local.z = msg.getPosition().getZ();
  node->crossFlag = msg.getCrossFlag();
  node->traffic_light_flag = msg.getTrafficLightFlag();

  const auto enter_ids = msg.getEnterLinkIDs();
  const auto out_ids = msg.getOutLinkIDs();
  auto* buffer = AllocateBytes(sizeof(sapLinkIDType) * enter_ids.size() +
                               sizeof(sapLinkIDType) * out_ids.size());
  auto* ptr = buffer;
  node->enter_link_ids = TakeBuffer<sapLinkIDType>(&ptr, enter_ids.size());
  node->enter_link_ids_num = enter_ids.size();
  for (uint64_t i = 0; i < node->enter_link_ids_num; ++i) {
    CopySapLinkID(enter_ids[i], &node->enter_link_ids[i]);
  }
  node->out_link_ids = TakeBuffer<sapLinkIDType>(&ptr, out_ids.size());
  node->out_link_ids_num = out_ids.size();
  for (uint64_t i = 0; i < node->out_link_ids_num; ++i) {
    CopySapLinkID(out_ids[i], &node->out_link_ids[i]);
  }
  return true;
}

void ReleaseSDMapLinkNode(sapLinkNode* node) {
  if (node == nullptr) {
    return;
  }
  auto* buffer = reinterpret_cast<uint8_t*>(node->enter_link_ids);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(node->out_link_ids);
  delete[] buffer;
}

void ConvertNavPathInfo(
    const ::senseAD::msg::sdmap::NavPathInfo::Reader& msg,
    sapNavRouteInfo* nav_route_info) {
  const auto raw_ids = msg.getLinkRawIDs();
  for (uint64_t i = 0; i < nav_route_info->link_raw_ids_num; ++i) {
    nav_route_info->link_raw_ids[i] = raw_ids[i];
  }
  const auto coordinates = msg.getCoordinates();
  for (uint64_t i = 0; i < nav_route_info->points_llh_num; ++i) {
    nav_route_info->points_llh[i].lon = coordinates[i].getX();
    nav_route_info->points_llh[i].lat = coordinates[i].getY();
    nav_route_info->points_llh[i].height = coordinates[i].getZ();
    nav_route_info->local_pts[i].x = coordinates[i].getX();
    nav_route_info->local_pts[i].y = coordinates[i].getY();
    nav_route_info->local_pts[i].z = coordinates[i].getZ();
  }
}

bool ConvertSDMap(const SDMap::Reader& msg, sapSDMap* sdmap) {
  if (sdmap == nullptr) {
    return false;
  }

  sdmap->timestamp_ns = msg.getMapHeader().getTimestampns();
  const auto links = msg.getLinks();
  const auto start_nodes = msg.getStartLinkNodes();
  const auto end_nodes = msg.getEndLinkNodes();
  const auto nav_info = msg.getNavPathInfo();
  const auto raw_ids = nav_info.getLinkRawIDs();
  const auto coordinates = nav_info.getCoordinates();

  auto* buffer = AllocateBytes(sizeof(sapLink) * links.size() +
                               sizeof(sapLinkNode) * start_nodes.size() +
                               sizeof(sapLinkNode) * end_nodes.size() +
                               sizeof(uint64_t) * raw_ids.size() +
                               sizeof(sapPointLLH) * coordinates.size() +
                               sizeof(sapPoint3f) * coordinates.size());
  auto* ptr = buffer;
  sdmap->links = TakeBuffer<sapLink>(&ptr, links.size());
  sdmap->links_num = links.size();
  for (uint64_t i = 0; i < sdmap->links_num; ++i) {
    if (!ConvertSDMapLink(links[i], &sdmap->links[i])) {
      return false;
    }
  }

  sdmap->start_link_nodes =
      TakeBuffer<sapLinkNode>(&ptr, start_nodes.size());
  sdmap->start_link_nodes_num = start_nodes.size();
  for (uint64_t i = 0; i < sdmap->start_link_nodes_num; ++i) {
    if (!ConvertSDMapLinkNode(start_nodes[i], &sdmap->start_link_nodes[i])) {
      return false;
    }
  }

  sdmap->end_link_nodes = TakeBuffer<sapLinkNode>(&ptr, end_nodes.size());
  sdmap->end_link_nodes_num = end_nodes.size();
  for (uint64_t i = 0; i < sdmap->end_link_nodes_num; ++i) {
    if (!ConvertSDMapLinkNode(end_nodes[i], &sdmap->end_link_nodes[i])) {
      return false;
    }
  }

  sdmap->nav_route_info.link_raw_ids =
      TakeBuffer<uint64_t>(&ptr, raw_ids.size());
  sdmap->nav_route_info.link_raw_ids_num = raw_ids.size();
  sdmap->nav_route_info.points_llh =
      TakeBuffer<sapPointLLH>(&ptr, coordinates.size());
  sdmap->nav_route_info.points_llh_num = coordinates.size();
  sdmap->nav_route_info.local_pts =
      TakeBuffer<sapPoint3f>(&ptr, coordinates.size());
  sdmap->nav_route_info.local_pts_num = coordinates.size();
  ConvertNavPathInfo(nav_info, &sdmap->nav_route_info);
  return true;
}

void ReleaseSDMap(sapSDMap* sdmap) {
  if (sdmap == nullptr) {
    return;
  }
  for (uint64_t i = 0; i < sdmap->links_num; ++i) {
    ReleaseSDMapLink(&sdmap->links[i]);
  }
  for (uint64_t i = 0; i < sdmap->start_link_nodes_num; ++i) {
    ReleaseSDMapLinkNode(&sdmap->start_link_nodes[i]);
  }
  for (uint64_t i = 0; i < sdmap->end_link_nodes_num; ++i) {
    ReleaseSDMapLinkNode(&sdmap->end_link_nodes[i]);
  }
  auto* buffer = reinterpret_cast<uint8_t*>(sdmap->links);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(sdmap->start_link_nodes);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(sdmap->end_link_nodes);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(sdmap->nav_route_info.link_raw_ids);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(sdmap->nav_route_info.points_llh);
  if (buffer == nullptr) buffer = reinterpret_cast<uint8_t*>(sdmap->nav_route_info.local_pts);
  // ConvertSDMap 使用一块连续 buffer 存放 links、nodes 和 nav_route_info
  // 数组，这里从首个非空字段恢复首地址后释放一次。
  delete[] buffer;
}

}  // namespace

bool ConvertSenseAutoSceneNaviMapToSap(
    SceneNaviMap::Reader msg, SapSDMapPtr* sdmap_output,
    SapRoadStructurePtr* road_structure_output,
    SapCrossInfoPtr* cross_info_output, SapNaviTopoPtr* navi_topo_output,
    uint64_t* timestamp_ns) {
  if (sdmap_output == nullptr || road_structure_output == nullptr ||
      cross_info_output == nullptr || navi_topo_output == nullptr ||
      timestamp_ns == nullptr) {
    return false;
  }

  auto sdmap = MakeSapInputPtr<sapSDMap>(new sapSDMap(), [](sapSDMap* value) {
    ReleaseSDMap(value);
  });
  std::memset(sdmap.get(), 0, sizeof(sapSDMap));
  // 一帧 SceneNaviMap 对应旧链路中 SDMap/LocalMap/NaviTopo 三个 convertor。
  // 这里一次转换产出四个 sap*，transport 再拆给对应 on_xxx 回调。
  if (!ConvertSDMap(msg.getSdMap(), sdmap.get())) {
    return false;
  }

  auto road_structure = MakeSapInputPtr<sapRoadStructure>(
      new sapRoadStructure(), [](sapRoadStructure* value) {
        ReleaseRoadStructure(value);
      });
  std::memset(road_structure.get(), 0, sizeof(sapRoadStructure));
  if (!ConvertRoadStructure(msg, road_structure.get())) {
    return false;
  }

  auto cross_info = MakeSapInputPtr<sapCrossInfo>(
      new sapCrossInfo(), [](sapCrossInfo* value) { ReleaseCrossInfo(value); });
  std::memset(cross_info.get(), 0, sizeof(sapCrossInfo));
  if (!ConvertCrossInfo(msg, cross_info.get())) {
    return false;
  }

  auto navi_topo = MakeSapInputPtr<sapNaviTopo>(
      new sapNaviTopo(), [](sapNaviTopo* value) { ReleaseNaviTopo(value); });
  std::memset(navi_topo.get(), 0, sizeof(sapNaviTopo));
  if (!ConvertNaviTopo(msg, navi_topo.get())) {
    return false;
  }

  *timestamp_ns = road_structure->timestamp_ns;
  *sdmap_output = sdmap;
  *road_structure_output = road_structure;
  *cross_info_output = cross_info;
  *navi_topo_output = navi_topo;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
