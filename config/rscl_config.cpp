// Ported from:
//   planning_bus/app/config/config.cpp
// Purpose:
//   Read planning_bus-style json config files into middleware-neutral
//   PlanningConfig plus RSCL-only topic settings.

#include "config/rscl_config.hpp"

#include <cstddef>
#include <fstream>

#include <json/json.h>

#ifndef USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR
#define USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR "config/dlp_node/x86"
#endif

namespace usharing_dlp_node {
namespace {

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

bool FileExists(const std::string& path) {
  std::ifstream file(path);
  return file.good();
}

bool LoadJsonFile(const std::string& path, Json::Value* value,
                  std::string* error) {
  if (value == nullptr) {
    SetError(error, "internal error: null Json::Value for " + path);
    return false;
  }

  std::ifstream json_file(path);
  if (!json_file.is_open()) {
    SetError(error, "failed to open config file: " + path);
    return false;
  }

  Json::Reader reader;
  Json::Value parsed;
  if (!reader.parse(json_file, parsed)) {
    SetError(error, "failed to parse config file: " + path + ": " +
                        reader.getFormattedErrorMessages());
    return false;
  }

  *value = parsed;
  return true;
}

// 以下路径工具只处理普通文件路径，不引入 ROS package/share 目录查找。
// 这样同一套配置加载代码可以被 RSCL、ROS 或未来其他入口复用。
std::string TrimTrailingSlashes(const std::string& path) {
  std::string trimmed = path;
  while (trimmed.size() > 1 && trimmed.back() == '/') {
    trimmed.pop_back();
  }
  return trimmed;
}

std::string ParentDir(const std::string& path) {
  const std::string trimmed = TrimTrailingSlashes(path);
  const std::size_t slash = trimmed.find_last_of('/');
  if (slash == std::string::npos) {
    return ".";
  }
  if (slash == 0) {
    return "/";
  }
  return trimmed.substr(0, slash);
}

std::string JoinPath(const std::string& base, const std::string& path) {
  if (base.empty() || base == ".") {
    return path;
  }
  if (base.back() == '/') {
    return base + path;
  }
  return base + "/" + path;
}

bool IsDotRelative(const std::string& path) {
  return path == "." || path.rfind("./", 0) == 0;
}

std::string ResolveModulePath(const std::string& module_root,
                              const std::string& path) {
  if (!IsDotRelative(path)) {
    return path;
  }
  if (path == ".") {
    return module_root;
  }
  return JoinPath(module_root, path.substr(2));
}

std::string ModuleRootFromConfigDir(const std::string& config_dir) {
  const std::string trimmed = TrimTrailingSlashes(config_dir);
  const std::string marker = "resource/config/dlp_node";
  const std::size_t pos = trimmed.find(marker);
  if (pos != std::string::npos) {
    // 配置目录形如 resource/config/dlp_node/x86，SDK 资源路径中的
    // "./resource/..." 需要相对 usharing_dlp_node 模块根目录解析。
    std::string root = trimmed.substr(0, pos);
    if (!root.empty() && root.back() == '/') {
      root.pop_back();
    }
    return root.empty() ? "." : root;
  }
  return ParentDir(trimmed);
}

void ResolvePlanningResourceRoots(const std::string& config_dir,
                                  PlanningConfig* config) {
  if (config == nullptr) {
    return;
  }
  const std::string module_root = ModuleRootFromConfigDir(config_dir);
  // planning_cfg 保留了原 workshop 配置里的 "./resource/..." 写法；
  // 加载后立即转成可执行进程可直接使用的路径。
  config->resource_root_path =
      ResolveModulePath(module_root, config->resource_root_path);
  config->sensor_config_root_path =
      ResolveModulePath(module_root, config->sensor_config_root_path);
}

TopicConfig ReadTopic(const Json::Value& root, const std::string& key,
                      const TopicConfig& fallback) {
  if (!root.isMember(key)) {
    return fallback;
  }

  const auto& value = root[key];
  TopicConfig topic = fallback;
  topic.type = value.get("type", fallback.type).asInt();
  topic.topic = value.get("topic", fallback.topic).asString();
  topic.is_pub = value.get("isPub", fallback.is_pub).asBool();
  return topic;
}

void ReadPlanningConfig(const Json::Value& value, PlanningConfig* config) {
  if (config == nullptr) {
    return;
  }

  const auto& root =
      value.isMember("usharing_dlp_node") ? value["usharing_dlp_node"] : value;
  // planning_cfg 对应旧 workshop 中 PerceptionCameraSdk 配置块，只描述
  // sap_camera SDK 初始化参数，不包含任何 RSCL subscribe/publish 信息。
  config->node_name = root.get("node_name", config->node_name).asString();
  config->resource_root_path =
      root.get("resource_root_path", config->resource_root_path).asString();
  config->sensor_config_folder =
      root.get("sensor_config_folder", config->sensor_config_folder)
          .asString();
  config->sensor_config_root_path =
      root.get("sensor_config_root_path", config->sensor_config_root_path)
          .asString();
  config->opencv_thread_num =
      root.get("opencv_thread_num", config->opencv_thread_num).asInt();
  config->single_thread =
      root.get("single_thread", config->single_thread).asBool();
  config->replay_mode = root.get("replay_mode", config->replay_mode).asBool();
  config->vpc_channel = root.get("vpc_channel", config->vpc_channel).asInt();

  if (root.isMember("camera_names") && root["camera_names"].isArray()) {
    config->camera_names.clear();
    for (const auto& item : root["camera_names"]) {
      config->camera_names.push_back(item.asString());
    }
  }

  if (root.isMember("cameras_fps") && root["cameras_fps"].isArray()) {
    config->cameras_fps.clear();
    for (const auto& item : root["cameras_fps"]) {
      config->cameras_fps.push_back(item.asDouble());
    }
  }

  if (root.isMember("pipelines") && root["pipelines"].isArray()) {
    config->pipelines.clear();
    for (const auto& item : root["pipelines"]) {
      PlanningPipelineConfig pipeline;
      pipeline.data_publisher_type =
          item.get("data_publisher_type", pipeline.data_publisher_type)
              .asString();
      pipeline.publisher_module_name =
          item.get("publisher_module_name", pipeline.publisher_module_name)
              .asString();
      pipeline.camera_name =
          item.get("camera_name", pipeline.camera_name).asString();
      pipeline.pipeline_param_file =
          item.get("pipeline_param_file", pipeline.pipeline_param_file)
              .asString();
      pipeline.infer_device_id =
          item.get("infer_device_id", pipeline.infer_device_id).asInt();
      pipeline.infer_ai_core_group_id =
          item.get("infer_ai_core_group_id",
                   pipeline.infer_ai_core_group_id)
              .asInt();
      config->pipelines.push_back(pipeline);
    }
  }
}

bool ValidatePlanningConfig(const PlanningConfig& config, std::string* error) {
  if (config.camera_names.empty()) {
    SetError(error, "planning_cfg camera_names must not be empty");
    return false;
  }
  if (config.camera_names.size() != config.cameras_fps.size()) {
    SetError(error, "planning_cfg camera_names and cameras_fps size mismatch");
    return false;
  }
  if (config.pipelines.empty()) {
    SetError(error, "planning_cfg pipelines must not be empty");
    return false;
  }
  for (std::size_t i = 0; i < config.pipelines.size(); ++i) {
    if (config.pipelines[i].pipeline_param_file.empty()) {
      SetError(error, "planning_cfg pipeline_param_file must not be empty");
      return false;
    }
    if (config.pipelines[i].publisher_module_name.empty()) {
      SetError(error, "planning_cfg publisher_module_name must not be empty");
      return false;
    }
    if (config.pipelines[i].camera_name.empty()) {
      SetError(error, "planning_cfg camera_name must not be empty");
      return false;
    }
  }
  return true;
}

}  // namespace

std::string JsonConfigLoader::NormalizeConfigDir(
    const std::string& config_dir) {
  if (config_dir.empty()) {
    // 默认使用 x86 配置；交叉编译/部署 thor 时由启动参数显式传入
    // resource/config/dlp_node/thor。
    return std::string("./usharing_dlp_node/resource/") +
           USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR + "/";
  }
  if (config_dir.back() == '/') {
    return config_dir;
  }
  return config_dir + "/";
}

bool JsonConfigLoader::Load(const std::string& config_dir,
                            PlanningConfig* planning_config,
                            RsclPortConfig* rscl_config) const {
  return Load(config_dir, planning_config, rscl_config, nullptr);
}

bool JsonConfigLoader::Load(const std::string& config_dir,
                            PlanningConfig* planning_config,
                            RsclPortConfig* rscl_config,
                            std::string* error) const {
  if (planning_config == nullptr || rscl_config == nullptr) {
    SetError(error, "internal error: null output config pointer");
    return false;
  }

  const std::string dir = NormalizeConfigDir(config_dir);

  Json::Value sys_cfg;
  if (FileExists(dir + "sys_cfg") &&
      !LoadJsonFile(dir + "sys_cfg", &sys_cfg, error)) {
    return false;
  }
  if (!sys_cfg.isNull()) {
    // sys_cfg 只提供进程名和车型选择。字段名 ros_node_name 来自参考工程，
    // 这里不代表 core 依赖 ROS。
    const auto& root =
        sys_cfg.isMember("sys_cfg") ? sys_cfg["sys_cfg"] : sys_cfg;
    rscl_config->node_name =
        root.get("ros_node_name", rscl_config->node_name).asString();
    rscl_config->car_type =
        root.get("car_type", rscl_config->car_type).asString();
    planning_config->node_name = rscl_config->node_name;
  }

  Json::Value planning_cfg;
  if (FileExists(dir + "planning_cfg") &&
      !LoadJsonFile(dir + "planning_cfg", &planning_cfg, error)) {
    return false;
  }
  if (!planning_cfg.isNull()) {
    // planning_cfg 缺失时保留 PlanningConfig 默认值，便于本地最小化测试；
    // topics_cfg 是通信链路必须项，缺失则 Load 返回 false。
    ReadPlanningConfig(planning_cfg, planning_config);
    ResolvePlanningResourceRoots(dir, planning_config);
    if (!ValidatePlanningConfig(*planning_config, error)) {
      return false;
    }
  }

  Json::Value topics_cfg;
  if (!LoadJsonFile(dir + "topics_cfg", &topics_cfg, error)) {
    return false;
  }

  const Json::Value& topics =
      topics_cfg.isMember(rscl_config->car_type)
          ? topics_cfg[rscl_config->car_type]
          : topics_cfg;

  // topics_cfg 对应 RSCL transport 的输入/输出端口。fallback 保留正式
  // Pilot DLP topic，避免新增车型配置时遗漏字段导致空 topic。
  rscl_config->object_frame =
      ReadTopic(topics, "object_frame_cfg",
                TopicConfig{0, "/perception/nv_cameras/object", false});
  rscl_config->road_geometry_frame =
      ReadTopic(topics, "road_geometry_frame_cfg",
                TopicConfig{0, "/perception/nv_cameras/road_geometry",
                            false});
  rscl_config->fusion_object_frame =
      ReadTopic(topics, "fusion_object_frame_cfg",
                TopicConfig{0, "", false});
  rscl_config->lidar_gop_frame =
      ReadTopic(topics, "lidar_gop_frame_cfg",
                TopicConfig{0, "/perception/lidar_gop/object", false});
  rscl_config->occupancy_info =
      ReadTopic(topics, "occupancy_info_cfg",
                TopicConfig{0, "/perception/nv_cameras/occ_info", false});
  rscl_config->local_localization =
      ReadTopic(topics, "local_localization_cfg",
                TopicConfig{0, "/localization/odomstate_info", false});
  rscl_config->location_info =
      ReadTopic(topics, "location_info_cfg",
                TopicConfig{0, "", false});
  rscl_config->vehicle_report =
      ReadTopic(topics, "vehicle_report_cfg",
                TopicConfig{0, "/canbus/vehicle_report", false});
  rscl_config->vehicle_processing =
      ReadTopic(topics, "vehicle_processing_cfg",
                TopicConfig{0, "/mcu/vehicle_processing", false});
  rscl_config->scene_navi_map =
      ReadTopic(topics, "scene_navi_map_cfg",
                TopicConfig{0, "/maplesslm/scene_navi_map", false});
  rscl_config->amap_navi_info =
      ReadTopic(topics, "amap_navi_info_cfg",
                TopicConfig{0, "/amap/nav_realtime/info", false});
  rscl_config->prediction_pub =
      ReadTopic(topics, "prediction_pub_cfg",
                TopicConfig{0, "/prediction/objects_vd", true});
  rscl_config->planning_debug_pub =
      ReadTopic(topics, "planning_debug_pub_cfg",
                TopicConfig{0, "/UniAD/PlanningDebug", true});
  rscl_config->diag_pub =
      ReadTopic(topics, "dlp_diag_pub_cfg",
                TopicConfig{0, "", false});

  return true;
}

}  // namespace usharing_dlp_node
