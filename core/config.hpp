// Ported from:
//   adapter/perception_camera/src/workshops/perception_camera/perception_camera_sdk_workshop.cpp
//   adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml
//   from l29_sap_node-refs_tags_SenseAuto_thor_mp_v0.4.34
// Purpose:
//   Middleware-neutral configuration structures for Pilot DLP planning.
//
// 中文说明：
//   这里是传给 sap_camera SDK 的纯配置结构，不包含 RSCL/ROS 概念。
//   JsonConfigLoader、未来的 ROS 参数或其他配置系统都应该先转换成这些结构，
//   再交给 SapPlanningEngine 初始化 SDK。
#pragma once

#include <string>
#include <vector>

namespace usharing_dlp_node {

struct PlanningPipelineConfig {
  // SDK pipeline 输出类型。当前 DLP planning 固定使用 planning。
  std::string data_publisher_type{"planning"};
  // sapPublishResultParam::module_name 用于筛选 SDK 回调结果。
  std::string publisher_module_name{"planning"};
  // sapGetPlanningFrame 需要的 camera_name，旧链路固定为 nv_cameras。
  std::string camera_name{"nv_cameras"};
  // 相对于 resource_root_path 的 SDK pipeline prototxt。
  std::string pipeline_param_file{
      "./config/perception_camera/x86/vd_mapqr_dlp/"
      "camera_bev_v9_7_ep12_dlp.prototxt"};
  int infer_device_id{0};
  int infer_ai_core_group_id{0};
};

struct PlanningConfig {
  std::string node_name{"pilot_dlp"};
  // SDK 资源根目录，包含 config/models/licenses/vehicle 等。
  std::string resource_root_path{"./resource/"};
  std::string sensor_config_folder{"./vehicle/default"};
  std::string sensor_config_root_path{"./resource/"};
  std::vector<std::string> camera_names{
      "center_camera_fov120", "center_camera_fov30", "left_front_camera",
      "right_front_camera", "left_rear_camera", "right_rear_camera",
      "rear_camera"};
  std::vector<double> cameras_fps{20.0, 20.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  std::vector<PlanningPipelineConfig> pipelines{PlanningPipelineConfig{}};
  int opencv_thread_num{1};
  bool single_thread{false};
  bool replay_mode{false};
  int vpc_channel{0};
};

}  // namespace usharing_dlp_node
