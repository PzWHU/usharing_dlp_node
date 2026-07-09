// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL RoadGeometryFrame subscriber boundary for Pilot DLP planning.
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

// RoadGeometryFrame 的 RSCL 订阅器，负责把道路几何消息转换成
// sapPerceptionRoadFrame 后交给 transport 上层。
class RoadGeometryFrameSubscriber {
 public:
  // 保存 topics_cfg 中 road_geometry_frame_cfg。
  void SetParam(const TopicConfig& param);
  // 注册 sapPerceptionRoadFrame 输出回调。
  void SetCallback(std::function<void(SapRoadFramePtr)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；字段映射由 road_geometry_frame_converter 完成。
  void RoadGeometryFrameCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::perception::RoadGeometryFrame>>& msg);

  TopicConfig param_;
  std::function<void(SapRoadFramePtr)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
