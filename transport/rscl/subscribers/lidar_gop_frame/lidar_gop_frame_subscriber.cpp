// Ported from:
//   adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to 0434 Lidar GOP input and forward converted sap DTOs.

#include "transport/rscl/subscribers/lidar_gop_frame/lidar_gop_frame_subscriber.hpp"

#include <utility>

#include "rscl_access.hpp"
#include "transport/rscl/converters/lidar_gop_frame_converter.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void LidarGopFrameSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void LidarGopFrameSubscriber::SetCallback(
    std::function<void(SapLidarGopFramePtr, uint64_t)> callback) {
  callback_ = std::move(callback);
}

bool LidarGopFrameSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::perception::ObjectFrame>(
                 param_.topic, kRsclDefaultQueueSize,
                 &LidarGopFrameSubscriber::LidarGopFrameCallback, this) == 0;
}

void LidarGopFrameSubscriber::LidarGopFrameCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapLidarGopFramePtr output;
  uint64_t timestamp_ns = 0;
  if (senseauto::ConvertSenseAutoLidarGopFrameToSap(
          msg->Msg(), &output, &timestamp_ns)) {
    callback_(std::move(output), timestamp_ns);
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
