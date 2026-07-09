// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_occ_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL OccupancyInfo subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <memory>

#include <ad_msg_idl/perception/perception.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// OccupancyInfo 的 RSCL 订阅器，输出 sapPerceptionOccFrame。
class OccupancyInfoSubscriber {
 public:
  // 保存 topics_cfg 中 occupancy_info_cfg。
  void SetParam(const TopicConfig& param);
  // 注册 OCC sap* 输出回调。
  void SetCallback(std::function<void(SapOccFramePtr)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；转换失败时直接丢弃本帧。
  void OccupancyInfoCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::perception::OccupancyInfo>>& msg);

  TopicConfig param_;
  std::function<void(SapOccFramePtr)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
