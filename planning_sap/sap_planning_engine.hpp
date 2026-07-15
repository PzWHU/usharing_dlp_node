// Ported from:
//   PerceptionCameraSdkWorkshop::InitSapCamera()
//   PerceptionCameraSdkWorkshop::OnDataArrived()
//   PerceptionCameraSdkWorkshop::PublishPlanning()
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Wrap sap_camera planning SDK without RSCL/SWCFL or ROS dependencies.
//
// 中文说明：
//   SapPlanningEngine 是 sap_camera SDK 适配层：
//   - Init 阶段设置 SDK 回调、创建/初始化/启动 sapCameraHandle；
//   - PushXxx 阶段把 sap* 输入送入 SDK 的 sapPush* API；
//   - SDK 完成处理后通过 PublishResultCallback 回调本类，再通过 sapGetPlanning*
//     获取输出并通知 PlanningService。
//   本层不处理 RSCL/ROS 消息，只管理 SDK 生命周期、输入 push 和输出 get。
#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>

#include "common/clock.hpp"
#include "common/logger.hpp"
#include "planning_core/planning_engine.hpp"

namespace usharing_dlp_node {

class SapPlanningEngine final : public IPlanningEngine {
 public:
  explicit SapPlanningEngine(ILogger* logger = nullptr, IClock* clock = nullptr);
  ~SapPlanningEngine() override;

  bool Init(const PlanningConfig& config) override;
  void Stop() override;

  bool PushObject(SapObjectFramePtr object_frame) override;
  bool PushRoad(SapRoadFramePtr road_frame) override;
  bool PushFusionObject(SapFusionObjectFramePtr fusion_frame) override;
  bool PushLidarGop(SapLidarGopFramePtr lidar_gop_frame,
                    uint64_t timestamp_ns) override;
  bool PushOcc(SapOccFramePtr occ_frame) override;
  bool PushLocalLocalization(SapLocalLocalizationPtr local_pose) override;
  bool PushLocationInfo(SapLocationInfoPtr location_info,
                        uint64_t timestamp_ns) override;
  bool PushSDMap(SapSDMapPtr sdmap, uint64_t timestamp_ns) override;
  bool PushManualSignal(SapManualSignalPtr manual_signal,
                        uint64_t timestamp_ns) override;
  bool PushCrossInfo(SapCrossInfoPtr cross_info, uint64_t timestamp_ns) override;
  bool PushRoadStructure(SapRoadStructurePtr road_structure,
                         uint64_t timestamp_ns) override;
  bool PushNaviTopo(SapNaviTopoPtr navi_topo, uint64_t timestamp_ns) override;
  bool PushChassisState(SapChassisStatePtr chassis_state,
                        uint64_t timestamp_ns) override;
  bool PushAmapNaviInfo(SapAmapNaviInfoPtr amap_navi_info,
                        uint64_t timestamp_ns) override;

  void SetPlanningInfo(const PerceptionPlanningInfo& info) override;

 private:
  // SDK 回调函数必须是普通函数指针，因此这里使用 static 函数，再通过
  // active_instance_ 找回当前引擎实例。
  static int ReleaseImageCallback(const uint8_t* const image, void* user_data);
  static int ReleaseLidarCallback(const uint8_t* const lidar_data_ptr,
                                  void* user_data);
  static int ReleaseSDMapCallback(const sapSDMap* const sdmap_data_ptr);
  static int PublishResultCallback(const sapPublishResultParam* const result);
  static int NotifyPipelineStatusCallback(const sapPipelineStatus* const status);

  // 将中立 PlanningConfig 转成 sapCameraParam，字段含义与 SDK 保持一致。
  bool FillSapCameraParam(const PlanningConfig& config);
  // SDK 处理完成后先进入该函数，根据 module_name 过滤只属于 planning 的结果。
  bool HandlePublishResult(const sapPublishResultParam& result);
  // 从 SDK 内部 frame/proxy 中取出 planning frame/debug info，并触发输出回调。
  bool PublishPlanning(const sapPublishResultParam& result);
  bool IsRunning(const char* scope) const;
  bool IsSuccess(int ret, const char* scope) const;
  void Log(LogLevel level, const std::string& scope,
           const std::string& message) const;

  static std::mutex active_mutex_;
  // sap_camera 的回调是进程级全局注册，当前实现只允许一个活动 engine。
  static SapPlanningEngine* active_instance_;

  ILogger* logger_{nullptr};
  IClock* clock_{nullptr};
  StderrLogger fallback_logger_;
  SystemClock fallback_clock_;
  sapCameraHandle camera_handle_{nullptr};
  sapCameraParam param_{};
  std::string planning_module_name_{"planning"};
  std::string planning_camera_name_{"nv_cameras"};
  PerceptionPlanningInfo planning_info_;
  std::atomic_bool running_{false};
  std::condition_variable active_callback_cv_;
  int active_callbacks_{0};
  bool stopping_callbacks_{false};
};

}  // namespace usharing_dlp_node
