// Ported from:
//   adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL Lidar GOP ObjectFrame subscriber boundary for 0434 Pilot DLP.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <ad_msg_idl/perception/perception.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

class LidarGopFrameSubscriber {
 public:
  void SetParam(const TopicConfig& param);
  void SetCallback(
      std::function<void(SapLidarGopFramePtr, uint64_t)> callback);
  bool Init();

 private:
  void LidarGopFrameCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg);

  TopicConfig param_;
  std::function<void(SapLidarGopFramePtr, uint64_t)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
