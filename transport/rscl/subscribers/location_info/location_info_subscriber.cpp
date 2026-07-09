// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_global_localization_convertor.cpp
//   planning_bus/data_rscl/position_data/position_data.cpp
// Purpose:
//   Replace SWCFL DataConvertor/Proxy with planning_bus-style RSCL subscribe
//   callback plus sapLocationInfo internal DTO conversion.

#include "transport/rscl/subscribers/location_info/location_info_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/location_info_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void LocationInfoSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void LocationInfoSubscriber::SetCallback(
    std::function<void(SapLocationInfoPtr, uint64_t)> callback) {
  callback_ = std::move(callback);
}

bool LocationInfoSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 订阅 INS/global localization，对应 SDK 的 sapPushLocationInfo。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::sensor::Ins>(
                 param_.topic, kRsclDefaultQueueSize,
                 &LocationInfoSubscriber::LocationInfoCallback, this) == 0;
}

void LocationInfoSubscriber::LocationInfoCallback(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::Ins>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapLocationInfoPtr output;
  uint64_t timestamp_ns = 0;
  if (senseauto::ConvertSenseAutoInsToLocationInfo(msg->Msg(), &output,
                                                   &timestamp_ns)) {
    // timestamp_ns 从原始 header 中取出，和 sapLocationInfo 一起传递。
    callback_(std::move(output), timestamp_ns);
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
