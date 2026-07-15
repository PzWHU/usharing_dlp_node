// Ported from:
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Middleware-neutral engine boundary around sap_camera planning SDK.
//
// 中文说明：
//   IPlanningEngine 是 planning_core 层看到的算法引擎抽象。输入统一使用 sap* DTO，
//   当前实现是 SapPlanningEngine，内部调用 sap_camera SDK 的 sapPush* API。
//   后续如果替换算法库，只需要实现该接口，PlanningService/transport 不需要变。
#pragma once

#include <functional>
#include <memory>

#include "planning_core/config.hpp"
#include "planning_core/diagnostics.hpp"
#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {

class IPlanningEngine {
 public:
  virtual ~IPlanningEngine() = default;

  virtual bool Init(const PlanningConfig& config) = 0;
  virtual void Stop() = 0;

  // 输入侧：transport/converter 已经完成中间件消息到 sap* 的转换。
  // 这些 PushXxx 调用只表达“把一帧内部 DTO 输入算法引擎”，不暴露 RSCL/ROS 类型。
  virtual bool PushObject(SapObjectFramePtr object_frame) = 0;
  virtual bool PushRoad(SapRoadFramePtr road_frame) = 0;
  virtual bool PushFusionObject(SapFusionObjectFramePtr fusion_frame) = 0;
  virtual bool PushLidarGop(SapLidarGopFramePtr lidar_gop_frame,
                            uint64_t timestamp_ns) = 0;
  virtual bool PushOcc(SapOccFramePtr occ_frame) = 0;
  virtual bool PushLocalLocalization(SapLocalLocalizationPtr local_pose) = 0;
  virtual bool PushLocationInfo(SapLocationInfoPtr location_info,
                                uint64_t timestamp_ns) = 0;
  virtual bool PushSDMap(SapSDMapPtr sdmap, uint64_t timestamp_ns) = 0;
  virtual bool PushManualSignal(SapManualSignalPtr manual_signal,
                                uint64_t timestamp_ns) = 0;
  virtual bool PushCrossInfo(SapCrossInfoPtr cross_info,
                             uint64_t timestamp_ns) = 0;
  virtual bool PushRoadStructure(SapRoadStructurePtr road_structure,
                                 uint64_t timestamp_ns) = 0;
  virtual bool PushNaviTopo(SapNaviTopoPtr navi_topo,
                            uint64_t timestamp_ns) = 0;
  virtual bool PushChassisState(SapChassisStatePtr chassis_state,
                                uint64_t timestamp_ns) = 0;
  virtual bool PushAmapNaviInfo(SapAmapNaviInfoPtr amap_navi_info,
                                uint64_t timestamp_ns) = 0;
  virtual void SetPlanningInfo(const PerceptionPlanningInfo& info) = 0;

  // 输出侧：算法引擎完成处理后主动通知 PlanningService。
  // PlanningService 再调用当前 transport 发布到 RSCL/ROS/其他中间件。
  std::function<void(SapPlanningFramePtr)> on_prediction;
  std::function<void(SapPlanningDebugInfoPtr, PerceptionPlanningInfo)> on_debug;
  std::function<void(const FaultCodes&)> on_diag;
};

}  // namespace usharing_dlp_node
