// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
// Purpose:
//   Preserve ObjectFrame -> sapPerceptionObjectFrame mapping while removing
//   RSCL/SWCFL Proxy, DataConvertor, and registration macro dependencies.

#include "transport/rscl/converters/object_frame_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "transport/rscl/converters/object_common.hpp"

namespace usharing_dlp_node {
namespace senseauto {

using Object = ::senseAD::msg::perception::Object;
using ObjectFrame = ::senseAD::msg::perception::ObjectFrame;

namespace {

using detail::ConvertBaseObject;

template <typename BboxReader>
void ConvertBbox(const BboxReader& source, sapBbox2D* target) {
  target->initialized = source.getInitialized();
  target->top_left_x = source.getTopLeftX();
  target->top_left_y = source.getTopLeftY();
  target->bottom_right_x = source.getBottomRightX();
  target->bottom_right_y = source.getBottomRightY();
  target->confidence = source.getConfidence();
  target->is_left_side_truncated = source.getIsLeftSideTruncated();
  target->is_right_side_truncated = source.getIsRightSideTruncated();
  target->is_top_side_truncated = source.getIsTopSideTruncated();
  target->is_bottom_side_truncated = source.getIsBottomSideTruncated();
}

void ConvertCameraBboxInfo(const Object::Reader& msg,
                           sapCameraBBox2DInfo* target) {
  const auto source = msg.getCameraBboxInfo();
  ConvertBbox(source.getRawDetectionBox(), &target->raw_detection_box);
  ConvertBbox(source.getTrackedBox(), &target->tracked_box);
}

void ConvertBboxMotionInfo(const Object::Reader& msg,
                           sapBBox2DMotionInfo* target) {
  const auto source = msg.getBboxMotionInfo();
  target->bottom_center_x_velocity = source.getBottomCenterXVelocity();
  target->bottom_center_y_velocity = source.getBottomCenterYVelocity();
  target->height_velocity = source.getHeightVelocity();
  target->width_velocity = source.getWidthVelocity();
  target->width_height_ratio_velocity = source.getWidthHeightRatioVelocity();
}


bool FillCameraAttribute(const Object::Reader& msg,
                         sapBaseCameraAttribute* attribute,
                         bool keep_direction) {
  const auto msg_attribute = msg.getAttribute();
  attribute->is_landmark_valid = msg_attribute.getIsLandmarkValid();
  const uint64_t landmark_num = msg_attribute.getLandmark().size();
  const uint64_t score_num = msg_attribute.getLandmarkScores().size();
  attribute->landmark_num = landmark_num;
  attribute->score_num = score_num;
  attribute->is_direction_valid =
      keep_direction ? msg_attribute.getIsDirectionValid() : false;
  if (keep_direction) {
    const auto viewport_direction = msg_attribute.getViewportDirection();
    attribute->viewport_direction.x = viewport_direction.getX();
    attribute->viewport_direction.y = viewport_direction.getY();
  } else {
    attribute->viewport_direction = {0.0f, 0.0f};
  }

  const std::size_t landmark_size = sizeof(sapVector2f) * landmark_num;
  const std::size_t score_size = sizeof(int8_t) * score_num;
  // landmark 和 score 共享一块连续 buffer；ReleaseConvertedObjectFrame
  // 会按同样规则只释放一次。
  auto* buffer =
      (landmark_size + score_size > 0)
          ? new uint8_t[landmark_size + score_size]
          : nullptr;

  auto* ptr = buffer;
  attribute->landmark =
      (landmark_num > 0) ? reinterpret_cast<sapVector2f*>(ptr) : nullptr;
  if (landmark_num > 0) {
    ptr += landmark_size;
  }
  attribute->landmark_scores =
      (score_num > 0) ? reinterpret_cast<int8_t*>(ptr) : nullptr;

  const auto landmarks = msg_attribute.getLandmark();
  for (uint64_t i = 0; i < landmark_num; ++i) {
    attribute->landmark[i].x = landmarks[i].getX();
    attribute->landmark[i].y = landmarks[i].getY();
  }

  const auto scores = msg_attribute.getLandmarkScores();
  for (uint64_t i = 0; i < score_num; ++i) {
    attribute->landmark_scores[i] = static_cast<int8_t>(scores[i]);
  }
  return true;
}

bool ConvertAutomobile(const Object::Reader& msg, sapAutomobile* vehicle) {
  if (vehicle == nullptr ||
      msg.getDerivedtype() != Object::DerivedType::CAMERA_VEHICLE) {
    return false;
  }
  if (!ConvertBaseObject(msg, &vehicle->base_info)) {
    return false;
  }

  const auto lane_assignments = msg.getOdRdInfo().getLaneAssignments();
  const uint64_t lane_num = lane_assignments.size();
  vehicle->lane_assignment_num = lane_num;

  const auto msg_attribute = msg.getAttribute();
  const uint64_t landmark_num = msg_attribute.getLandmark().size();
  const uint64_t score_num = msg_attribute.getLandmarkScores().size();
  const std::size_t landmark_size = sizeof(sapVector2f) * landmark_num;
  const std::size_t score_size = sizeof(int8_t) * score_num;
  const std::size_t lane_size = sizeof(int16_t) * lane_num;
  // 车辆目标还包含 lane_assignments，因此和 landmark/score 共同分配
  // 一块连续 buffer，保持旧 convertor 的内存布局。
  auto* buffer =
      (landmark_size + score_size + lane_size > 0)
          ? new uint8_t[landmark_size + score_size + lane_size]
          : nullptr;

  auto* ptr = buffer;
  vehicle->attribute.is_landmark_valid = msg_attribute.getIsLandmarkValid();
  vehicle->attribute.landmark_num = landmark_num;
  vehicle->attribute.score_num = score_num;
  vehicle->attribute.landmark =
      (landmark_num > 0) ? reinterpret_cast<sapVector2f*>(ptr) : nullptr;
  if (landmark_num > 0) {
    ptr += landmark_size;
  }
  vehicle->attribute.landmark_scores =
      (score_num > 0) ? reinterpret_cast<int8_t*>(ptr) : nullptr;
  if (score_num > 0) {
    ptr += score_size;
  }
  vehicle->lane_assignments =
      (lane_num > 0) ? reinterpret_cast<int16_t*>(ptr) : nullptr;
  vehicle->attribute.is_direction_valid = msg_attribute.getIsDirectionValid();
  const auto viewport_direction = msg_attribute.getViewportDirection();
  vehicle->attribute.viewport_direction.x = viewport_direction.getX();
  vehicle->attribute.viewport_direction.y = viewport_direction.getY();

  const auto landmarks = msg_attribute.getLandmark();
  for (uint64_t i = 0; i < landmark_num; ++i) {
    vehicle->attribute.landmark[i].x = landmarks[i].getX();
    vehicle->attribute.landmark[i].y = landmarks[i].getY();
  }
  const auto scores = msg_attribute.getLandmarkScores();
  for (uint64_t i = 0; i < score_num; ++i) {
    vehicle->attribute.landmark_scores[i] = static_cast<int8_t>(scores[i]);
  }
  for (uint64_t i = 0; i < lane_num; ++i) {
    vehicle->lane_assignments[i] = static_cast<int16_t>(lane_assignments[i]);
  }

  const auto depth_point = msg.getDepthPoint();
  vehicle->depth_point.nearest_point_x = depth_point.getNearestPointX();
  vehicle->depth_point.depth_point_x = depth_point.getDepthPointX();
  vehicle->depth_point.depth_point_y = depth_point.getDepthPointY();
  const auto front_point = depth_point.getLandmarkFrontPoint();
  vehicle->depth_point.landmark_front_point[0] = front_point.getX();
  vehicle->depth_point.landmark_front_point[1] = front_point.getY();
  const auto rear_point = depth_point.getLandmarkRearPoint();
  vehicle->depth_point.landmark_rear_point[0] = rear_point.getX();
  vehicle->depth_point.landmark_rear_point[1] = rear_point.getY();
  vehicle->depth_point.orientation = depth_point.getOrientation();
  vehicle->depth_point.label = static_cast<int16_t>(depth_point.getLabel());

  const auto light_status = msg.getAutomobileLightStatus();
  vehicle->automobile_light_status.left_turn =
      static_cast<int16_t>(light_status.getLeftTurn());
  vehicle->automobile_light_status.left_turn_confidence =
      light_status.getLeftTurnConfidence();
  vehicle->automobile_light_status.right_turn =
      static_cast<int16_t>(light_status.getRightTurn());
  vehicle->automobile_light_status.right_turn_confidence =
      light_status.getRightTurnConfidence();
  vehicle->automobile_light_status.hazard =
      static_cast<int16_t>(light_status.getHazard());
  vehicle->automobile_light_status.hazard_confidence =
      light_status.getHazardConfidence();
  vehicle->automobile_light_status.brake =
      static_cast<int16_t>(light_status.getBrake());
  vehicle->automobile_light_status.brake_confidence =
      light_status.getBrakeConfidence();

  ConvertCameraBboxInfo(msg, &vehicle->camera_bbox_info);
  ConvertBboxMotionInfo(msg, &vehicle->bbox_motion_info);
  vehicle->depth_type = static_cast<int16_t>(msg.getDebugInfo().getDepthType());
  return true;
}

bool ConvertNonAutomobile(const Object::Reader& msg,
                          sapNonAutomobileVehicle* non_vehicle) {
  if (non_vehicle == nullptr ||
      msg.getDerivedtype() != Object::DerivedType::CAMERA_NON_VEHICLE) {
    return false;
  }
  if (!ConvertBaseObject(msg, &non_vehicle->base_info)) {
    return false;
  }
  if (!FillCameraAttribute(msg, &non_vehicle->attribute, false)) {
    return false;
  }
  ConvertCameraBboxInfo(msg, &non_vehicle->camera_bbox_info);
  ConvertBboxMotionInfo(msg, &non_vehicle->bbox_motion_info);
  return true;
}

bool ConvertPedestrian(const Object::Reader& msg, sapPedestrian* pedestrian) {
  if (pedestrian == nullptr ||
      msg.getDerivedtype() != Object::DerivedType::CAMERA_PEDESTRIAN) {
    return false;
  }
  if (!ConvertBaseObject(msg, &pedestrian->base_info)) {
    return false;
  }
  if (!FillCameraAttribute(msg, &pedestrian->attribute, true)) {
    return false;
  }
  ConvertCameraBboxInfo(msg, &pedestrian->camera_bbox_info);
  ConvertBboxMotionInfo(msg, &pedestrian->bbox_motion_info);
  pedestrian->depth_type =
      static_cast<int16_t>(msg.getDebugInfo().getDepthType());
  return true;
}

bool ConvertObjectFrame(const ObjectFrame::Reader& msg,
                        sapPerceptionObjectFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  frame->sensor_id = static_cast<uint16_t>(msg.getSensorId());
  frame->timestamp_ns = msg.getFrameTimestampNs();
  frame->sensor_send_timestamp_ns = 0;

  const auto objects = msg.getPerceptionObjectList();
  frame->veh_num = 0;
  frame->non_veh_num = 0;
  frame->ped_num = 0;
  for (const auto& object : objects) {
    switch (object.getDerivedtype()) {
      case Object::DerivedType::CAMERA_VEHICLE:
        ++frame->veh_num;
        break;
      case Object::DerivedType::CAMERA_NON_VEHICLE:
        ++frame->non_veh_num;
        break;
      case Object::DerivedType::CAMERA_PEDESTRIAN:
        ++frame->ped_num;
        break;
      default:
        break;
    }
  }

  const std::size_t vehicle_size = sizeof(sapAutomobile) * frame->veh_num;
  const std::size_t non_vehicle_size =
      sizeof(sapNonAutomobileVehicle) * frame->non_veh_num;
  const std::size_t pedestrian_size = sizeof(sapPedestrian) * frame->ped_num;
  const std::size_t buffer_size =
      vehicle_size + non_vehicle_size + pedestrian_size;
  // vehicle/non_vehicle/pedestrian 三类对象放在一块连续 buffer 中，
  // SapObjectFramePtr 的 deleter 会按原始首地址释放。
  auto* buffer = (buffer_size > 0) ? new uint8_t[buffer_size] : nullptr;

  auto* ptr = buffer;
  frame->vehicle =
      (frame->veh_num > 0) ? reinterpret_cast<sapAutomobile*>(ptr) : nullptr;
  if (frame->veh_num > 0) {
    ptr += vehicle_size;
  }
  frame->non_vehicle =
      (frame->non_veh_num > 0)
          ? reinterpret_cast<sapNonAutomobileVehicle*>(ptr)
          : nullptr;
  if (frame->non_veh_num > 0) {
    ptr += non_vehicle_size;
  }
  frame->pedestrian =
      (frame->ped_num > 0) ? reinterpret_cast<sapPedestrian*>(ptr) : nullptr;

  for (std::size_t i = 0; i < frame->veh_num; ++i) {
    frame->vehicle[i] = {};
  }
  for (std::size_t i = 0; i < frame->non_veh_num; ++i) {
    frame->non_vehicle[i] = {};
  }
  for (std::size_t i = 0; i < frame->ped_num; ++i) {
    frame->pedestrian[i] = {};
  }

  std::size_t vehicle_index = 0;
  std::size_t non_vehicle_index = 0;
  std::size_t pedestrian_index = 0;
  // 第一次遍历只统计各类数量，第二次遍历再按 derived type 写入对应数组。
  for (const auto& object : objects) {
    switch (object.getDerivedtype()) {
      case Object::DerivedType::CAMERA_VEHICLE:
        if (!ConvertAutomobile(object, &frame->vehicle[vehicle_index++])) {
          return false;
        }
        break;
      case Object::DerivedType::CAMERA_NON_VEHICLE:
        if (!ConvertNonAutomobile(
                object, &frame->non_vehicle[non_vehicle_index++])) {
          return false;
        }
        break;
      case Object::DerivedType::CAMERA_PEDESTRIAN:
        if (!ConvertPedestrian(object,
                               &frame->pedestrian[pedestrian_index++])) {
          return false;
        }
        break;
      default:
        break;
    }
  }
  return true;
}

}  // namespace

bool ConvertSenseAutoObjectFrameToSap(ObjectFrame::Reader msg,
                                      SapObjectFramePtr* output) {
  if (output == nullptr) {
    return false;
  }

  auto frame = MakeObjectFrame();
  if (!ConvertObjectFrame(msg, frame.get())) {
    return false;
  }
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
