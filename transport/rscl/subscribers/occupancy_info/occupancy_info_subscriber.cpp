// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_occ_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to occupancy RSCL input and forward converted sap occupancy data.

#include "transport/rscl/subscribers/occupancy_info/occupancy_info_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/occupancy_info_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void OccupancyInfoSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void OccupancyInfoSubscriber::SetCallback(
    std::function<void(SapOccFramePtr)> callback) {
  callback_ = std::move(callback);
}

bool OccupancyInfoSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // OCC 输入来自感知相机 topic，转换后作为 sapPerceptionOccFrame 推给 SDK。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::perception::OccupancyInfo>(
                 param_.topic, kRsclDefaultQueueSize,
                 &OccupancyInfoSubscriber::OccupancyInfoCallback, this) == 0;
}

void OccupancyInfoSubscriber::OccupancyInfoCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::perception::OccupancyInfo>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapOccFramePtr output;
  if (senseauto::ConvertSenseAutoOccupancyInfoToSap(msg->Msg(), &output)) {
    // output 的内部数组由 sap_raii 中的 deleter 释放。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
