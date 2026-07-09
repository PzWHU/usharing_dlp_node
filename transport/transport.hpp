// Ported from:
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
// Purpose:
//   Replace RSCL Receiver/Sender workshops with a middleware-neutral transport.
//
// 中文说明：
//   IPlanningTransport 是 core 层看到的通信抽象。它把所有输入都暴露为 sap*
//   或 core 内部小结构体，不把 RSCL/ROS 消息类型泄漏到 PlanningService。
//   新增 ROS 或其他中间件时，应实现同一接口并复用 PlanningService/SapPlanningEngine。
#pragma once

#include <functional>

#include "core/diagnostics.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {

class IPlanningTransport {
 public:
  virtual ~IPlanningTransport() = default;

  virtual bool Init() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;

  // 输入回调：transport 收到外部消息并转换为内部 DTO 后触发。
  // PlanningService 在 Init 中给这些 std::function 赋值，完成输入到 engine 的连接。
  std::function<void(SapObjectFramePtr)> on_object;
  std::function<void(SapRoadFramePtr)> on_road;
  std::function<void(SapFusionObjectFramePtr)> on_fusion_object;
  std::function<void(SapLidarGopFramePtr, uint64_t)> on_lidar_gop;
  std::function<void(SapOccFramePtr)> on_occ;
  std::function<void(SapLocalLocalizationPtr)> on_local_localization;
  std::function<void(SapLocationInfoPtr, uint64_t)> on_location_info;
  std::function<void(const VehicleInfo&, uint64_t)> on_vehicle_info;
  std::function<void(SapSDMapPtr, uint64_t)> on_sdmap;
  std::function<void(SapManualSignalPtr, uint64_t)> on_manual_signal;
  std::function<void(SapCrossInfoPtr, uint64_t)> on_cross_info;
  std::function<void(SapRoadStructurePtr, uint64_t)> on_road_structure;
  std::function<void(SapNaviTopoPtr, uint64_t)> on_navi_topo;
  std::function<void(SapChassisStatePtr, uint64_t)> on_chassis_state;
  std::function<void(SapAmapNaviInfoPtr, uint64_t)> on_amap_navi_info;

  // 输出发布：engine 通过 PlanningService 请求 transport 发布 SDK 输出结果。
  virtual void PublishPrediction(
      const sapPerceptionPlanningFrame& planning_frame) = 0;
  virtual void PublishPlanningDebug(
      const sapPerceptionPlanningDebugInfo& debug_info,
                                    const PerceptionPlanningInfo& info) = 0;
  virtual void PublishDiag(const FaultCodes& fault_codes) = 0;
};

}  // namespace usharing_dlp_node
