// Ported from:
//   PerceptionCameraSdkWorkshop::InitSapCamera()
//   PerceptionCameraSdkWorkshop::DeInitSapCamera()
//   PerceptionCameraSdkWorkshop::OnDataArrived()
//   PerceptionCameraSdkWorkshop::PublishResults()
//   PerceptionCameraSdkWorkshop::PublishPlanning()
//   adapter/perception_camera/src/workshops/perception_camera/perception_camera_sdk_workshop.cpp
// Purpose:
//   Keep the sap_camera planning SDK lifecycle and push/get API behind a
//   middleware-neutral engine interface.

#include "sap/sap_planning_engine.hpp"

#include <algorithm>
#include <cstring>
#include <sstream>

namespace usharing_dlp_node {

namespace {
constexpr int kSapSuccess = 0;

#ifdef USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME
extern "C" int32_t sapPushLidarGopFrame(
    sapCameraHandle handle,
    const uint64_t timestamp_ns,
    sapPerceptionLidarGopFrame* const sap_lidar_gop_frame);
#endif

// sapCameraParam 里的路径/模块名都是定长 C 字符数组。这里统一做截断
// 和结尾 '\0' 保护，避免每个字段拷贝处重复处理边界。
void CopyString(const std::string& source, char* destination,
                std::size_t destination_size) {
  if (destination_size == 0) {
    return;
  }
  std::strncpy(destination, source.c_str(), destination_size - 1);
  destination[destination_size - 1] = '\0';
}
}  // namespace

std::mutex SapPlanningEngine::active_mutex_;
SapPlanningEngine* SapPlanningEngine::active_instance_ = nullptr;

SapPlanningEngine::SapPlanningEngine(ILogger* logger, IClock* clock)
    : logger_(logger ? logger : &fallback_logger_),
      clock_(clock ? clock : &fallback_clock_) {}

SapPlanningEngine::~SapPlanningEngine() {
  Stop();
}

bool SapPlanningEngine::Init(const PlanningConfig& config) {
  if (!FillSapCameraParam(config)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    // sapSetCallbackFunSet 注册的是进程级 C 回调，回调参数里没有 this。
    // 因此当前进程只允许一个 SapPlanningEngine 实例处于 active 状态。
    if (active_instance_ != nullptr && active_instance_ != this) {
      Log(LogLevel::kError, "SapPlanningEngine::Init",
          "sap_camera callbacks are process-global; another engine is active");
      return false;
    }
    active_instance_ = this;
  }

  // SDK -> node 的回调集合。输入链路由 sapPush* 进入 SDK；输出链路则
  // 由 PublishResultCallback 通知 node 再调用 sapGetPlanning* 拉取结果。
  sapCallBackFunSet callbacks{&SapPlanningEngine::ReleaseImageCallback,
                              &SapPlanningEngine::PublishResultCallback,
                              &SapPlanningEngine::NotifyPipelineStatusCallback,
                              &SapPlanningEngine::ReleaseLidarCallback,
                              &SapPlanningEngine::ReleaseSDMapCallback};

  if (!IsSuccess(sapSetCallbackFunSet(&callbacks),
                 "SapPlanningEngine::sapSetCallbackFunSet")) {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_instance_ = nullptr;
    return false;
  }

  camera_handle_ = sapCreate(&param_);
  if (camera_handle_ == nullptr) {
    Log(LogLevel::kError, "SapPlanningEngine::Init",
        "sapCreate returned nullptr");
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_instance_ = nullptr;
    return false;
  }

  if (!IsSuccess(sapInit(camera_handle_), "SapPlanningEngine::sapInit") ||
      !IsSuccess(sapStart(camera_handle_), "SapPlanningEngine::sapStart")) {
    Stop();
    return false;
  }

  running_ = true;
  Log(LogLevel::kInfo, "SapPlanningEngine::Init",
      "sap_camera planning SDK started");
  return true;
}

void SapPlanningEngine::Stop() {
  if (camera_handle_ != nullptr) {
    // 生命周期顺序保持和旧 PerceptionCameraSdkWorkshop::DeInitSapCamera()
    // 一致：stop -> deinit -> destroy。
    if (running_) {
      IsSuccess(sapStop(camera_handle_), "SapPlanningEngine::sapStop");
    }
    IsSuccess(sapDeinit(camera_handle_), "SapPlanningEngine::sapDeinit");
    IsSuccess(sapDestroy(&camera_handle_), "SapPlanningEngine::sapDestroy");
    camera_handle_ = nullptr;
  }
  running_ = false;

  std::lock_guard<std::mutex> lock(active_mutex_);
  if (active_instance_ == this) {
    active_instance_ = nullptr;
  }
}

bool SapPlanningEngine::PushObject(SapObjectFramePtr object_frame) {
  if (!IsRunning("SapPlanningEngine::PushObject") || !object_frame) {
    return false;
  }
  // sapPushObjectFrame 是 node 输入链路在本工程内的边界；后续排队、
  // 同步和算法处理都在 sap_camera SDK 内部完成。
  return IsSuccess(sapPushObjectFrame(camera_handle_,
                                      object_frame->timestamp_ns,
                                      object_frame.get()),
                   "SapPlanningEngine::sapPushObjectFrame");
}

bool SapPlanningEngine::PushRoad(SapRoadFramePtr road_frame) {
  if (!IsRunning("SapPlanningEngine::PushRoad") || !road_frame) {
    return false;
  }
  return IsSuccess(sapPushRoadFrame(camera_handle_, road_frame->timestamp_ns,
                                    road_frame.get()),
                   "SapPlanningEngine::sapPushRoadFrame");
}

bool SapPlanningEngine::PushFusionObject(SapFusionObjectFramePtr fusion_frame) {
  if (!IsRunning("SapPlanningEngine::PushFusionObject") || !fusion_frame) {
    return false;
  }
  return IsSuccess(sapPushFusionObjectFrame(camera_handle_,
                                            fusion_frame->timestamp_ns,
                                            fusion_frame.get()),
                   "SapPlanningEngine::sapPushFusionObjectFrame");
}

bool SapPlanningEngine::PushLidarGop(SapLidarGopFramePtr lidar_gop_frame,
                                     uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushLidarGop") || !lidar_gop_frame) {
    return false;
  }
#ifdef USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME
  return IsSuccess(sapPushLidarGopFrame(camera_handle_, timestamp_ns,
                                        lidar_gop_frame.get()),
                   "SapPlanningEngine::sapPushLidarGopFrame");
#else
  (void)timestamp_ns;
  Log(LogLevel::kError, "SapPlanningEngine::PushLidarGop",
      "sapPushLidarGopFrame is not declared by the bundled SDK; rebuild with "
      "USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME=ON and a matching 0434 "
      "sap_camera library to enable this input");
  return false;
#endif
}

bool SapPlanningEngine::PushOcc(SapOccFramePtr occ_frame) {
  if (!IsRunning("SapPlanningEngine::PushOcc") || !occ_frame) {
    return false;
  }
  return IsSuccess(sapPushOccInfoFrame(camera_handle_,
                                       occ_frame->timestamp_ns,
                                       occ_frame.get()),
                   "SapPlanningEngine::sapPushOccInfoFrame");
}

bool SapPlanningEngine::PushLocalLocalization(
    SapLocalLocalizationPtr local_pose) {
  if (!IsRunning("SapPlanningEngine::PushLocalLocalization") || !local_pose) {
    return false;
  }
  return IsSuccess(sapPushLocalizationInfo(camera_handle_,
                                           local_pose->measurement_time_ns,
                                           local_pose.get()),
                   "SapPlanningEngine::sapPushLocalizationInfo");
}

bool SapPlanningEngine::PushLocationInfo(SapLocationInfoPtr location_info,
                                         uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushLocationInfo") || !location_info) {
    return false;
  }
  return IsSuccess(sapPushLocationInfo(camera_handle_, timestamp_ns,
                                       location_info.get()),
                   "SapPlanningEngine::sapPushLocationInfo");
}

bool SapPlanningEngine::PushSDMap(SapSDMapPtr sdmap, uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushSDMap") || !sdmap) {
    return false;
  }
  return IsSuccess(sapPushSDMap(camera_handle_, timestamp_ns, sdmap.get()),
                   "SapPlanningEngine::sapPushSDMap");
}

bool SapPlanningEngine::PushManualSignal(SapManualSignalPtr manual_signal,
                                         uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushManualSignal") || !manual_signal) {
    return false;
  }
  // sapManualSignal 既有显式 timestamp 字段，sapPushManualSignal 也接收
  // timestamp 参数，两处保持一致，方便 SDK 内部按结构体或参数读取。
  manual_signal->timestamp = timestamp_ns;
  return IsSuccess(sapPushManualSignal(camera_handle_, timestamp_ns,
                                       manual_signal.get()),
                   "SapPlanningEngine::sapPushManualSignal");
}

bool SapPlanningEngine::PushCrossInfo(SapCrossInfoPtr cross_info,
                                      uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushCrossInfo") || !cross_info) {
    return false;
  }
  return IsSuccess(sapPushCrossInfo(camera_handle_, timestamp_ns,
                                    cross_info.get()),
                   "SapPlanningEngine::sapPushCrossInfo");
}

bool SapPlanningEngine::PushRoadStructure(SapRoadStructurePtr road_structure,
                                          uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushRoadStructure") || !road_structure) {
    return false;
  }
  return IsSuccess(
      sapPushRoadStructure(camera_handle_, timestamp_ns, road_structure.get()),
      "SapPlanningEngine::sapPushRoadStructure");
}

bool SapPlanningEngine::PushNaviTopo(SapNaviTopoPtr navi_topo,
                                     uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushNaviTopo") || !navi_topo) {
    return false;
  }
  return IsSuccess(sapPushNaviTopo(camera_handle_, timestamp_ns,
                                   navi_topo.get()),
                   "SapPlanningEngine::sapPushNaviTopo");
}

bool SapPlanningEngine::PushChassisState(SapChassisStatePtr chassis_state,
                                         uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushChassisState") || !chassis_state) {
    return false;
  }
  return IsSuccess(sapPushChassisState(camera_handle_, timestamp_ns,
                                       chassis_state.get()),
                   "SapPlanningEngine::sapPushChassisState");
}

bool SapPlanningEngine::PushAmapNaviInfo(SapAmapNaviInfoPtr amap_navi_info,
                                         uint64_t timestamp_ns) {
  if (!IsRunning("SapPlanningEngine::PushAmapNaviInfo") || !amap_navi_info) {
    return false;
  }
  return IsSuccess(sapPushAmapNaviInfo(camera_handle_, timestamp_ns,
                                       amap_navi_info.get()),
                   "SapPlanningEngine::sapPushAmapNaviInfo");
}

void SapPlanningEngine::SetPlanningInfo(const PerceptionPlanningInfo& info) {
  planning_info_ = info;
}

int SapPlanningEngine::ReleaseImageCallback(const uint8_t* const image,
                                            void* user_data) {
  (void)image;
  (void)user_data;
  return kSapSuccess;
}

int SapPlanningEngine::ReleaseLidarCallback(const uint8_t* const lidar_data_ptr,
                                            void* user_data) {
  (void)lidar_data_ptr;
  (void)user_data;
  return kSapSuccess;
}

int SapPlanningEngine::ReleaseSDMapCallback(
    const sapSDMap* const sdmap_data_ptr) {
  (void)sdmap_data_ptr;
  return kSapSuccess;
}

int SapPlanningEngine::PublishResultCallback(
    const sapPublishResultParam* const result) {
  if (result == nullptr) {
    return -1;
  }

  SapPlanningEngine* engine = nullptr;
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    // C 回调转回 C++ 对象方法。这里不持锁调用 HandlePublishResult，
    // 避免 SDK 回调处理过程中阻塞 Stop()/析构对 active_instance_ 的清理。
    engine = active_instance_;
  }
  if (engine == nullptr) {
    return -1;
  }
  return engine->HandlePublishResult(*result) ? kSapSuccess : -1;
}

int SapPlanningEngine::NotifyPipelineStatusCallback(
    const sapPipelineStatus* const status) {
  (void)status;
  return kSapSuccess;
}

bool SapPlanningEngine::FillSapCameraParam(const PlanningConfig& config) {
  std::memset(&param_, 0, sizeof(param_));

  // sapCameraParam 使用 SDK 头文件中的固定数组，超过上限会造成写越界。
  // 配置加载阶段不假设外部 json 一定合法，因此在写入前做保护。
  if (config.camera_names.size() > MAX_CAMERA_NUM ||
      config.pipelines.size() > MAX_PIPELINE_NUM) {
    Log(LogLevel::kError, "SapPlanningEngine::FillSapCameraParam",
        "camera or pipeline count exceeds sapCameraParam fixed array size");
    return false;
  }

  CopyString(config.resource_root_path, param_.senseauto_root_path, MAX_PATH);
  CopyString(config.sensor_config_folder, param_.sensor_config_folder,
             MAX_PATH);
  CopyString(config.sensor_config_root_path, param_.sensor_config_root_path,
             MAX_PATH);
  param_.opencv_thread_num = config.opencv_thread_num;
  param_.is_single_thread = config.single_thread ? 1 : 0;
  param_.is_replay_mode = config.replay_mode ? 1 : 0;
  param_.vpc_channel = config.vpc_channel;

  param_.camera_num = static_cast<int>(config.camera_names.size());
  for (std::size_t i = 0; i < config.camera_names.size(); ++i) {
    CopyString(config.camera_names[i], param_.camera_names[i], MAX_NAME);
    param_.cameras_fps[i] =
        (i < config.cameras_fps.size()) ? config.cameras_fps[i] : 0.0;
  }

  param_.pipeline_num = static_cast<int>(config.pipelines.size());
  param_.pub_module_num = static_cast<int>(config.pipelines.size());
  for (std::size_t i = 0; i < config.pipelines.size(); ++i) {
    const auto& pipeline = config.pipelines[i];
    CopyString(pipeline.pipeline_param_file, param_.pipeline_files[i],
               MAX_PATH);
    CopyString(pipeline.publisher_module_name, param_.pub_module_names[i],
               MAX_NAME);
    param_.pipeline_infer_ai_core_group_id[i] =
        pipeline.infer_ai_core_group_id;
  }

  if (!config.pipelines.empty()) {
    const auto& pipeline = config.pipelines.front();
    // Pilot DLP 当前只配置一个 planning pipeline。保留数组写入是为了
    // 对齐 sapCameraParam，planning_module_name_/camera_name_ 用于输出过滤。
    param_.pipeline_infer_dev_id = pipeline.infer_device_id;
    planning_module_name_ = pipeline.publisher_module_name;
    planning_camera_name_ = pipeline.camera_name;
  }
  return true;
}

bool SapPlanningEngine::HandlePublishResult(
    const sapPublishResultParam& result) {
  if (!result.is_pub_result) {
    return true;
  }
  // SDK 可能回调多个 module 的结果，DLP node 只处理配置中的 planning
  // module，等价于旧代码里 PublishResults() 的 module_name 分发。
  if (result.module_name == nullptr ||
      planning_module_name_ != std::string(result.module_name)) {
    return true;
  }
  return PublishPlanning(result);
}

bool SapPlanningEngine::PublishPlanning(
    const sapPublishResultParam& result) {
  if (!IsRunning("SapPlanningEngine::PublishPlanning")) {
    return false;
  }

  auto planning_frame = MakePlanningFrame();
  // SDK 输出内存由 sapGetPlanningFrame 填充，SapPlanningFramePtr 的
  // deleter 在 sap_types/sap_raii 中统一释放，避免业务层直接处理数组。
  const int planning_ret =
      sapGetPlanningFrame(camera_handle_, &result, planning_camera_name_.c_str(),
                          planning_frame.get());
  if (!IsSuccess(planning_ret, "SapPlanningEngine::sapGetPlanningFrame")) {
    return false;
  }

  if (on_prediction) {
    on_prediction(planning_frame);
  }

  auto debug_info = MakePlanningDebugInfo();
  // Debug 信息和 Prediction 分开拉取；PlanningService 会把当前车辆状态机
  // 维护的 PerceptionPlanningInfo 一并交给输出 converter。
  const int debug_ret =
      sapGetPlanningDebugInfo(camera_handle_, debug_info.get());
  if (!IsSuccess(debug_ret, "SapPlanningEngine::sapGetPlanningDebugInfo")) {
    return false;
  }

  if (on_debug) {
    on_debug(debug_info, planning_info_);
  }
  return true;
}

bool SapPlanningEngine::IsRunning(const char* scope) const {
  if (!running_ || camera_handle_ == nullptr) {
    Log(LogLevel::kError, scope, "sap_camera handle is not running");
    return false;
  }
  return true;
}

bool SapPlanningEngine::IsSuccess(int ret, const char* scope) const {
  if (ret == kSapSuccess) {
    return true;
  }
  std::ostringstream oss;
  oss << "sap_camera call failed, ret=" << ret;
  Log(LogLevel::kError, scope, oss.str());
  return false;
}

void SapPlanningEngine::Log(LogLevel level, const std::string& scope,
                            const std::string& message) const {
  if (logger_) {
    logger_->Log(level, scope, message);
  }
}

}  // namespace usharing_dlp_node
