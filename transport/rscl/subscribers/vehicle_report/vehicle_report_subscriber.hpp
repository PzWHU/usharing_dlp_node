// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_report_to_can_bus_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL VehicleReport subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <cstdint>
#include <memory>

#include <ad_msg_idl/ad_vehicle/vehicle.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "transport/rscl/converters/vehicle_report_converter.hpp"
#include "planning_core/diagnostics.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// VehicleReport 的 RSCL 订阅器。输出不是 sap*，而是 VehicleInfo；
// VehicleStateMachine 会继续把它转换为 sapManualSignal。
class VehicleReportSubscriber {
 public:
  // 保存 topics_cfg 中 vehicle_report_cfg。
  void SetParam(const TopicConfig& param);
  // 注册 VehicleInfo 输出回调；timestamp_ns 取自原始消息 header。
  void SetCallback(std::function<void(const VehicleInfo&, uint64_t)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口。conversion_state_ 保存接管判断所需的跨帧计数。
  void VehicleReportCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::vehicle::VehicleReport>>& msg);

  TopicConfig param_;
  std::function<void(const VehicleInfo&, uint64_t)> callback_;
  senseauto::VehicleReportConversionState conversion_state_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
