// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
// Purpose:
//   Preserve fusion ObjectFrame -> sapPerceptionFusionObjectFrame mapping
//   without RSCL/SWCFL convertor registration dependencies.

#include "transport/rscl/converters/fusion_object_frame_converter.hpp"

#include <cstdint>
#include <memory>

#include "transport/rscl/converters/converter_common.hpp"
#include "transport/rscl/converters/object_common.hpp"

namespace usharing_dlp_node {
namespace senseauto {

using Object = ::senseAD::msg::perception::Object;
using ObjectFrame = ::senseAD::msg::perception::ObjectFrame;

namespace {

using detail::ConvertBaseObject;
using detail::TimestampToNs;

bool ConvertFusionObject(const Object::Reader& msg_object,
                         sapFusionObject* fusion_object) {
  if (fusion_object == nullptr) {
    return false;
  }

  if (!ConvertBaseObject(msg_object, &fusion_object->base_object)) {
    return false;
  }

  fusion_object->base_object.maintenance_status =
      static_cast<sapMaintenanceStatus>(msg_object.getMaintenanceStatus());

  const auto motion_info = msg_object.getMotionInfo();
  const auto relative_velocity = motion_info.getRelativeVelocity();
  fusion_object->relative_velocity[0] = relative_velocity.getX();
  fusion_object->relative_velocity[1] = relative_velocity.getY();
  fusion_object->relative_velocity[2] = relative_velocity.getZ();

  const auto door_status = msg_object.getAutomobileDoorStatus();
  fusion_object->door_status.bboxValidity =
      static_cast<int64_t>(door_status.getBboxValidity());
  fusion_object->door_status.leftDoorStatus =
      static_cast<int64_t>(door_status.getLeftDoorStatus());
  fusion_object->door_status.rightDoorStatus =
      static_cast<int64_t>(door_status.getRightDoorStatus());
  fusion_object->door_status.rearDoorStatus =
      static_cast<int64_t>(door_status.getRearDoorStatus());

  const auto light_status = msg_object.getAutomobileLightStatus();
  fusion_object->vehicle_light_info.left_turn =
      static_cast<sapAttrBool>(light_status.getLeftTurn());
  fusion_object->vehicle_light_info.right_turn =
      static_cast<sapAttrBool>(light_status.getRightTurn());
  fusion_object->vehicle_light_info.hazard =
      static_cast<sapAttrBool>(light_status.getHazard());
  fusion_object->vehicle_light_info.brake =
      static_cast<sapAttrBool>(light_status.getBrake());
  fusion_object->vehicle_light_info.left_turn_confidence =
      light_status.getLeftTurnConfidence();
  fusion_object->vehicle_light_info.right_turn_confidence =
      light_status.getRightTurnConfidence();
  fusion_object->vehicle_light_info.hazard_confidence =
      light_status.getHazardConfidence();
  fusion_object->vehicle_light_info.brake_confidence =
      light_status.getBrakeConfidence();

  fusion_object->track_status =
      static_cast<sapTrackStatus>(msg_object.getTrackStatus());

  const auto matched_infos = msg_object.getMatchedSensorObjectInfo();
  // matched/history 信息是融合目标相对相机目标多出的部分，使用 SDK
  // ReleaseFusionObjectFrameMemory 释放。
  fusion_object->matched_sensor_object_info_num = matched_infos.size();
  fusion_object->matched_sensor_object_info =
      fusion_object->matched_sensor_object_info_num > 0
          ? new sapMatchedSensorObjectInfo
                [fusion_object->matched_sensor_object_info_num]()
          : nullptr;
  for (uint64_t i = 0; i < fusion_object->matched_sensor_object_info_num;
       ++i) {
    fusion_object->matched_sensor_object_info[i].frame_timestamp_ns =
        matched_infos[i].getFrameTimestampNs();
    fusion_object->matched_sensor_object_info[i].object_id =
        matched_infos[i].getObjectId();
  }

  const auto history_infos = msg_object.getHistInfo();
  fusion_object->associated_sensor_object_history_list_num =
      history_infos.size();
  fusion_object->associated_sensor_object_history_list =
      fusion_object->associated_sensor_object_history_list_num > 0
          ? new sapPerceptionLifeTimeHistory
                [fusion_object->associated_sensor_object_history_list_num]()
          : nullptr;
  for (uint64_t i = 0;
       i < fusion_object->associated_sensor_object_history_list_num; ++i) {
    auto& object_history =
        fusion_object->associated_sensor_object_history_list[i];
    const auto history = history_infos[i];
    object_history.sensor_id = static_cast<uint8_t>(history.getSensorId());
    object_history.track_id = history.getObstacleId();
    object_history.raw_sensor_capture_timestamp_ns =
        TimestampToNs(history.getRawSensorCaptureTimestamp());
    object_history.sensor_perception_capture_timestamp_ns =
        TimestampToNs(history.getSensorPerceptionCaptureTimestamp());
    object_history.sensor_perception_output_timestamp_ns =
        TimestampToNs(history.getSensorPerceptionOutputTimestamp());
    object_history.fusion_capture_timestamp_ns =
        TimestampToNs(history.getFusionCaptureTimestamp());
    object_history.fusion_output_timestamp_ns =
        TimestampToNs(history.getFusionOutputTimestamp());
  }

  return true;
}

bool ConvertFusionObjectFrame(const ObjectFrame::Reader& msg,
                              sapPerceptionFusionObjectFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  frame->sensor_id = static_cast<int8_t>(msg.getSensorId());
  frame->timestamp_ns = msg.getFrameTimestampNs();
  frame->sensor_send_timestamp_ns = msg.getHeader().getTime().getNanoSec();

  const auto objects = msg.getPerceptionObjectList();
  frame->fusion_objects_num = objects.size();
  frame->fusion_objects_list =
      frame->fusion_objects_num > 0
          ? new sapFusionObject[frame->fusion_objects_num]()
          : nullptr;

  for (uint64_t i = 0; i < frame->fusion_objects_num; ++i) {
    if (!ConvertFusionObject(objects[i], &frame->fusion_objects_list[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool ConvertSenseAutoFusionObjectFrameToSap(ObjectFrame::Reader msg,
                                            SapFusionObjectFramePtr* output) {
  if (output == nullptr) {
    return false;
  }

  auto* raw_frame = new sapPerceptionFusionObjectFrame();
  InitsapPerceptionFusionObjectFrame(raw_frame);
  // 融合目标 frame 的内部内存按 SDK helper 释放，因此 deleter 使用
  // ReleaseFusionObjectFrameMemory。
  auto frame = MakeSapInputPtr<sapPerceptionFusionObjectFrame>(
      raw_frame, [](sapPerceptionFusionObjectFrame* value) {
        ReleaseFusionObjectFrameMemory(value);
      });
  if (!ConvertFusionObjectFrame(msg, frame.get())) {
    return false;
  }
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
