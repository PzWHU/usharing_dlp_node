// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_processing_to_chassis_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL Vehicleprocessing subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <cstdint>
#include <memory>

#include <ad_msg_idl/ad_planning/planning.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// Vehicleprocessing 的 RSCL 订阅器，输出 sapChassisState。
class VehicleProcessingSubscriber {
 public:
  // 保存 topics_cfg 中 vehicle_processing_cfg。
  void SetParam(const TopicConfig& param);
  // 注册 chassis state 输出回调。
  void SetCallback(
      std::function<void(SapChassisStatePtr, uint64_t)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；转换结果会同时影响 SDK 输入和车辆状态机缓存。
  void VehicleProcessingCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::planning::Vehicleprocessing>>& msg);

  TopicConfig param_;
  std::function<void(SapChassisStatePtr, uint64_t)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
