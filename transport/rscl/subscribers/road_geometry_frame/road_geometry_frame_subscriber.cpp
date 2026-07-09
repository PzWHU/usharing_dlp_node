// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to road geometry RSCL input and forward converted sap roads.

#include "transport/rscl/subscribers/road_geometry_frame/road_geometry_frame_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/road_geometry_frame_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void RoadGeometryFrameSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void RoadGeometryFrameSubscriber::SetCallback(
    std::function<void(SapRoadFramePtr)> callback) {
  callback_ = std::move(callback);
}

bool RoadGeometryFrameSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 注册 RoadGeometryFrame 订阅；字段映射保持在 converter 中。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::perception::RoadGeometryFrame>(
                 param_.topic, kRsclDefaultQueueSize,
                 &RoadGeometryFrameSubscriber::RoadGeometryFrameCallback, this) == 0;
}

void RoadGeometryFrameSubscriber::RoadGeometryFrameCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::perception::RoadGeometryFrame>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapRoadFramePtr output;
  if (senseauto::ConvertSenseAutoRoadGeometryFrameToSap(msg->Msg(),
                                                        &output)) {
    // output 已经是 sapPerceptionRoadFrame。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
