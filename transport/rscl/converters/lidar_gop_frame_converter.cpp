// Ported from:
//   adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp
// Purpose:
//   Preserve 0434 Lidar GOP ObjectFrame -> sapPerceptionLidarGopFrame mapping
//   without SWCFL Proxy/DataConvertor dependencies.

#include "transport/rscl/converters/lidar_gop_frame_converter.hpp"

#include <cstddef>
#include <cstring>
#include <new>

namespace usharing_dlp_node {
namespace senseauto {

using Object = ::senseAD::msg::perception::Object;
using ObjectFrame = ::senseAD::msg::perception::ObjectFrame;

namespace {

bool ConvertLidarObstacle(const Object::Reader& msg,
                          uint64_t frame_timestamp_ns,
                          sapLidarObstacle* obstacle) {
  if (obstacle == nullptr) {
    return false;
  }

  obstacle->id = static_cast<uint32_t>(msg.getTrackId());
  obstacle->tracked_id = msg.getTrackId();
  obstacle->timestamp_ns = frame_timestamp_ns;
  obstacle->type = static_cast<sapLidarObjectType>(msg.getLabel());
  obstacle->confidence = msg.getTypeConfidence();
  obstacle->motion_state =
      static_cast<LidarObjectMotionState>(msg.getTrackStatus());

  const auto size_info = msg.getSizeInfo();
  obstacle->size.length = size_info.getSize().getX();
  obstacle->size.width = size_info.getSize().getY();
  obstacle->size.height = size_info.getSize().getZ();

  const auto motion_info = msg.getMotionInfo();
  obstacle->center.x = motion_info.getCenter().getX();
  obstacle->center.y = motion_info.getCenter().getY();
  obstacle->center.z =
      motion_info.getCenter().getZ() - obstacle->size.height / 2.0f;

  const auto direction_info = msg.getDirectionInfo();
  obstacle->yaw = direction_info.getDirectionYpr().getX();

  if (msg.hasSensorIdBox()) {
    const auto sensor_boxes = msg.getSensorIdBox();
    std::size_t box_index = 0;
    for (std::size_t i = 0; i < sensor_boxes.size() && box_index < 2; ++i) {
      const auto box = sensor_boxes[i];
      auto& target = obstacle->box_2d[box_index++];
      target.initialized = 1;
      target.top_left_x = box.getTopLeftX();
      target.top_left_y = box.getTopLeftY();
      target.bottom_right_x = box.getBottomRightX();
      target.bottom_right_y = box.getBottomRightY();
      target.is_left_side_truncated = box.getIsLeftSideTruncated();
      target.is_right_side_truncated = box.getIsRightSideTruncated();
      target.is_top_side_truncated = box.getIsTopSideTruncated();
      target.is_bottom_side_truncated = box.getIsBottomSideTruncated();
    }
  }

  return true;
}

bool ConvertLidarGopFrame(const ObjectFrame::Reader& msg,
                          sapPerceptionLidarGopFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  frame->sensor_id = static_cast<uint16_t>(msg.getSensorId());
  frame->timestamp_ns = msg.getFrameTimestampNs();
  frame->sensor_send_timestamp_ns = 0;
  frame->decoded_lidar_gop_num = 0;
  frame->decoded_lidar_object = nullptr;

  const auto objects = msg.getPerceptionObjectList();
  const auto object_num = static_cast<uint16_t>(objects.size());
  if (object_num == 0) {
    frame->lidar_gop_num = 0;
    frame->lidar_object = nullptr;
    return true;
  }

  frame->lidar_object = new (std::nothrow) sapLidarObstacle[object_num];
  if (frame->lidar_object == nullptr) {
    return false;
  }
  std::memset(frame->lidar_object, 0,
              sizeof(sapLidarObstacle) * object_num);

  uint16_t count = 0;
  for (uint16_t i = 0; i < object_num; ++i) {
    if (ConvertLidarObstacle(objects[i], frame->timestamp_ns,
                             &frame->lidar_object[count])) {
      ++count;
    }
  }
  frame->lidar_gop_num = count;
  return true;
}

}  // namespace

bool ConvertSenseAutoLidarGopFrameToSap(ObjectFrame::Reader msg,
                                        SapLidarGopFramePtr* output,
                                        uint64_t* timestamp_ns) {
  if (output == nullptr || timestamp_ns == nullptr) {
    return false;
  }

  auto frame = MakeLidarGopFrame();
  if (!ConvertLidarGopFrame(msg, frame.get())) {
    return false;
  }

  *timestamp_ns = frame->timestamp_ns;
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
