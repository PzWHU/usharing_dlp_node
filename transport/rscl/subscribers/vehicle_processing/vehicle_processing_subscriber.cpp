// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_processing_to_chassis_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to vehicle processing RSCL input and forward converted chassis state.

#include "transport/rscl/subscribers/vehicle_processing/vehicle_processing_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/vehicle_processing_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void VehicleProcessingSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void VehicleProcessingSubscriber::SetCallback(
    std::function<void(SapChassisStatePtr, uint64_t)> callback) {
  callback_ = std::move(callback);
}

bool VehicleProcessingSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // Vehicleprocessing 转成 sapChassisState 后既 push 给 SDK，也更新状态机。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::planning::Vehicleprocessing>(
                 param_.topic, kRsclDefaultQueueSize,
                 &VehicleProcessingSubscriber::VehicleProcessingCallback, this) ==
         0;
}

void VehicleProcessingSubscriber::VehicleProcessingCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::planning::Vehicleprocessing>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapChassisStatePtr output;
  uint64_t timestamp_ns = 0;
  if (senseauto::ConvertSenseAutoVehicleProcessingToChassisState(
          msg->Msg(), &output, &timestamp_ns)) {
    // timestamp_ns 从 RSCL message header 取出，保持和旧 convertor 一致。
    callback_(std::move(output), timestamp_ns);
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
