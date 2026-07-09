// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_localization_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to local localization RSCL input and forward converted sap odom data.

#include "transport/rscl/subscribers/local_localization/local_localization_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/local_localization_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void LocalLocalizationSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void LocalLocalizationSubscriber::SetCallback(
    std::function<void(SapLocalLocalizationPtr)> callback) {
  callback_ = std::move(callback);
}

bool LocalLocalizationSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 订阅局部定位 OdomStateInfo，对应 SDK 的 sapPushLocalizationInfo。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::localization::OdomStateInfo>(
                 param_.topic, kRsclDefaultQueueSize,
                 &LocalLocalizationSubscriber::LocalLocalizationCallback, this) == 0;
}

void LocalLocalizationSubscriber::LocalLocalizationCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::localization::OdomStateInfo>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapLocalLocalizationPtr output;
  if (senseauto::ConvertSenseAutoOdomStateToLocalLocalization(msg->Msg(),
                                                              &output)) {
    // output 已经包含 measurement_time_ns，engine 会用它作为 push 时间。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
