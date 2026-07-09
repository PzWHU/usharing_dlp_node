// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_report_to_can_bus_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to vehicle report RSCL input and forward neutral VehicleInfo.

#include "transport/rscl/subscribers/vehicle_report/vehicle_report_subscriber.hpp"

#include <utility>

#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void VehicleReportSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void VehicleReportSubscriber::SetCallback(
    std::function<void(const VehicleInfo&, uint64_t)> callback) {
  callback_ = std::move(callback);
}

bool VehicleReportSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // VehicleReport 先转成 VehicleInfo，再由 VehicleStateMachine 生成
  // sapManualSignal；这里不直接接触 sap_camera SDK。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::vehicle::VehicleReport>(
                 param_.topic, kRsclDefaultQueueSize,
                 &VehicleReportSubscriber::VehicleReportCallback, this) == 0;
}

void VehicleReportSubscriber::VehicleReportCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::vehicle::VehicleReport>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  VehicleInfo vehicle_info;
  uint64_t timestamp_ns = 0;
  if (senseauto::ConvertSenseAutoVehicleReportToVehicleInfo(
          msg->Msg(), &conversion_state_, &vehicle_info, &timestamp_ns)) {
    // conversion_state_ 保留跨帧接管计数，vehicle_info 是本帧中立结构。
    callback_(vehicle_info, timestamp_ns);
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
