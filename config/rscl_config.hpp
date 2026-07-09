// Ported from:
//   planning_bus/app/config/config.h
//   planning_bus/app/config/config.cpp
// Purpose:
//   JsonCpp-based configuration loader for the RSCL transport port. Only this
//   config boundary knows the planning_bus-style json file layout.
//
// 中文说明：
//   本文件定义 RSCL 版本的配置模型。配置文件采用 planning_bus 风格的
//   sys_cfg/planning_cfg/topics_cfg 三文件布局，但解析结果会转换为
//   middleware-neutral 的 PlanningConfig 和 RSCL 专用的 RsclPortConfig。
#pragma once

#include <string>

#include "core/config.hpp"

namespace usharing_dlp_node {

struct TopicConfig {
  // type 保留原 planning_bus 配置字段，当前主要用于兼容配置格式。
  int type{0};
  // RSCL topic 名称；发布和订阅统一使用该字段。
  std::string topic;
  // 对发布 topic 生效。订阅 topic 通常只看 topic 是否为空。
  bool is_pub{true};
};

struct RsclPortConfig {
  // RSCL 节点名，传给 CRsclAccess::init。
  std::string node_name{"pilot_dlp_usharing_dlp_node"};
  std::string car_type{"s31"};

  // 输入 topic：每个 subscriber 只负责一种外部消息到 sap*/内部结构的转换。
  TopicConfig object_frame;
  TopicConfig road_geometry_frame;
  TopicConfig fusion_object_frame;
  TopicConfig lidar_gop_frame;
  TopicConfig occupancy_info;
  TopicConfig local_localization;
  TopicConfig location_info;
  TopicConfig vehicle_report;
  TopicConfig vehicle_processing;
  TopicConfig scene_navi_map;
  TopicConfig amap_navi_info;
  // 输出 topic：SDK 处理完成后由 RsclPlanningTransport 发布。
  TopicConfig prediction_pub;
  TopicConfig planning_debug_pub;
  TopicConfig diag_pub;
};

class JsonConfigLoader {
 public:
  // 从 config_dir 读取 sys_cfg/planning_cfg/topics_cfg。
  // config_dir 可以是绝对路径，也可以是 resource/config/dlp_node/<platform>。
  bool Load(const std::string& config_dir, PlanningConfig* planning_config,
            RsclPortConfig* rscl_config) const;

 private:
  static std::string NormalizeConfigDir(const std::string& config_dir);
};

}  // namespace usharing_dlp_node
