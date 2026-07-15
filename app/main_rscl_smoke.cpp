// Ported from:
//   planning_bus/sys_main/sys_main.cpp
//   planning_bus/app/planning/planning.cpp
// Purpose:
//   RSCL bag smoke-test entry. It validates RSCL receive -> converter -> sap*
//   -> IPlanningEngine::PushXxx wiring without loading the sap_camera runtime.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <limits.h>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include "config/rscl_config.hpp"
#include "planning_core/planning_engine.hpp"
#include "planning_core/planning_service.hpp"
#include "rscl_access.hpp"
#include "rscl_planning_transport.hpp"

#ifndef USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR
#define USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR "config/dlp_node/x86"
#endif

namespace {

std::atomic_bool g_running{true};

void HandleSignal(int) {
  g_running = false;
}

bool FileExists(const std::string& path) {
  return access(path.c_str(), F_OK) == 0;
}

std::string ExecutableDir() {
  char exe_path[PATH_MAX] = {0};
  const ssize_t len = readlink("/proc/self/exe", exe_path,
                               sizeof(exe_path) - 1);
  if (len <= 0) {
    return ".";
  }
  exe_path[len] = '\0';
  std::string path(exe_path);
  const std::size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? "." : path.substr(0, slash);
}

std::string ReadArgValue(int argc, char** argv, const std::string& key) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == key) {
      return argv[i + 1];
    }
  }
  return "";
}

std::string ReadConfigDir(int argc, char** argv) {
  const std::string explicit_dir = ReadArgValue(argc, argv, "--config_dir");
  if (!explicit_dir.empty()) {
    return explicit_dir;
  }

  std::string default_config_subdir = USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR;
  const std::string platform = ReadArgValue(argc, argv, "--config_platform");
  if (!platform.empty()) {
    default_config_subdir = std::string("config/dlp_node/") + platform;
  }

  const std::string default_config_dir =
      std::string("./resource/") + default_config_subdir;
  if (FileExists(default_config_dir + "/topics_cfg")) {
    return default_config_dir;
  }
  const std::string exe_dir = ExecutableDir();
  const std::string build_tree_config_dir =
      exe_dir + "/../resource/" + default_config_subdir;
  if (FileExists(build_tree_config_dir + "/topics_cfg")) {
    return build_tree_config_dir;
  }
  const std::string installed_config_dir =
      exe_dir + "/resource/" + default_config_subdir;
  if (FileExists(installed_config_dir + "/topics_cfg")) {
    return installed_config_dir;
  }
  return default_config_dir;
}

int ReadRunSeconds(int argc, char** argv) {
  const std::string value = ReadArgValue(argc, argv, "--run_seconds");
  if (value.empty()) {
    return 20;
  }
  const int parsed = std::atoi(value.c_str());
  return parsed > 0 ? parsed : 20;
}

struct SmokeCounters {
  std::atomic<uint64_t> object{0};
  std::atomic<uint64_t> road{0};
  std::atomic<uint64_t> fusion_object{0};
  std::atomic<uint64_t> lidar_gop{0};
  std::atomic<uint64_t> occ{0};
  std::atomic<uint64_t> local_localization{0};
  std::atomic<uint64_t> location_info{0};
  std::atomic<uint64_t> sdmap{0};
  std::atomic<uint64_t> manual_signal{0};
  std::atomic<uint64_t> cross_info{0};
  std::atomic<uint64_t> road_structure{0};
  std::atomic<uint64_t> navi_topo{0};
  std::atomic<uint64_t> chassis_state{0};
  std::atomic<uint64_t> amap_navi_info{0};
  std::atomic<uint64_t> planning_info{0};
};

class CountingPlanningEngine final : public usharing_dlp_node::IPlanningEngine {
 public:
  explicit CountingPlanningEngine(SmokeCounters* counters)
      : counters_(counters) {}

  bool Init(const usharing_dlp_node::PlanningConfig& config) override {
    std::cerr << "[smoke] fake engine initialized: node=" << config.node_name
              << ", pipelines=" << config.pipelines.size() << std::endl;
    return true;
  }

  void Stop() override {
    std::cerr << "[smoke] fake engine stopped" << std::endl;
  }

  bool PushObject(usharing_dlp_node::SapObjectFramePtr object_frame) override {
    (void)object_frame;
    ++counters_->object;
    return true;
  }

  bool PushRoad(usharing_dlp_node::SapRoadFramePtr road_frame) override {
    (void)road_frame;
    ++counters_->road;
    return true;
  }

  bool PushFusionObject(
      usharing_dlp_node::SapFusionObjectFramePtr fusion_frame) override {
    (void)fusion_frame;
    ++counters_->fusion_object;
    return true;
  }

  bool PushLidarGop(usharing_dlp_node::SapLidarGopFramePtr lidar_gop_frame,
                    uint64_t timestamp_ns) override {
    (void)lidar_gop_frame;
    (void)timestamp_ns;
    ++counters_->lidar_gop;
    return true;
  }

  bool PushOcc(usharing_dlp_node::SapOccFramePtr occ_frame) override {
    (void)occ_frame;
    ++counters_->occ;
    return true;
  }

  bool PushLocalLocalization(
      usharing_dlp_node::SapLocalLocalizationPtr local_pose) override {
    (void)local_pose;
    ++counters_->local_localization;
    return true;
  }

  bool PushLocationInfo(usharing_dlp_node::SapLocationInfoPtr location_info,
                        uint64_t timestamp_ns) override {
    (void)location_info;
    (void)timestamp_ns;
    ++counters_->location_info;
    return true;
  }

  bool PushSDMap(usharing_dlp_node::SapSDMapPtr sdmap,
                 uint64_t timestamp_ns) override {
    (void)sdmap;
    (void)timestamp_ns;
    ++counters_->sdmap;
    return true;
  }

  bool PushManualSignal(usharing_dlp_node::SapManualSignalPtr manual_signal,
                        uint64_t timestamp_ns) override {
    (void)manual_signal;
    (void)timestamp_ns;
    ++counters_->manual_signal;
    return true;
  }

  bool PushCrossInfo(usharing_dlp_node::SapCrossInfoPtr cross_info,
                     uint64_t timestamp_ns) override {
    (void)cross_info;
    (void)timestamp_ns;
    ++counters_->cross_info;
    return true;
  }

  bool PushRoadStructure(
      usharing_dlp_node::SapRoadStructurePtr road_structure,
      uint64_t timestamp_ns) override {
    (void)road_structure;
    (void)timestamp_ns;
    ++counters_->road_structure;
    return true;
  }

  bool PushNaviTopo(usharing_dlp_node::SapNaviTopoPtr navi_topo,
                    uint64_t timestamp_ns) override {
    (void)navi_topo;
    (void)timestamp_ns;
    ++counters_->navi_topo;
    return true;
  }

  bool PushChassisState(usharing_dlp_node::SapChassisStatePtr chassis_state,
                        uint64_t timestamp_ns) override {
    (void)chassis_state;
    (void)timestamp_ns;
    ++counters_->chassis_state;
    return true;
  }

  bool PushAmapNaviInfo(usharing_dlp_node::SapAmapNaviInfoPtr amap_navi_info,
                        uint64_t timestamp_ns) override {
    (void)amap_navi_info;
    (void)timestamp_ns;
    ++counters_->amap_navi_info;
    return true;
  }

  void SetPlanningInfo(
      const usharing_dlp_node::PerceptionPlanningInfo& info) override {
    (void)info;
    ++counters_->planning_info;
  }

 private:
  SmokeCounters* counters_{nullptr};
};

void PrintCounters(const SmokeCounters& counters) {
  std::cout << "[smoke] push_object=" << counters.object.load() << '\n'
            << "[smoke] push_road=" << counters.road.load() << '\n'
            << "[smoke] push_fusion_object="
            << counters.fusion_object.load() << '\n'
            << "[smoke] push_lidar_gop=" << counters.lidar_gop.load() << '\n'
            << "[smoke] push_occ=" << counters.occ.load() << '\n'
            << "[smoke] push_local_localization="
            << counters.local_localization.load() << '\n'
            << "[smoke] push_location_info="
            << counters.location_info.load() << '\n'
            << "[smoke] push_sdmap=" << counters.sdmap.load() << '\n'
            << "[smoke] push_manual_signal="
            << counters.manual_signal.load() << '\n'
            << "[smoke] push_cross_info=" << counters.cross_info.load()
            << '\n'
            << "[smoke] push_road_structure="
            << counters.road_structure.load() << '\n'
            << "[smoke] push_navi_topo=" << counters.navi_topo.load()
            << '\n'
            << "[smoke] push_chassis_state="
            << counters.chassis_state.load() << '\n'
            << "[smoke] push_amap_navi_info="
            << counters.amap_navi_info.load() << '\n'
            << "[smoke] set_planning_info="
            << counters.planning_info.load() << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  usharing_dlp_node::PlanningConfig planning_config;
  usharing_dlp_node::RsclPortConfig rscl_config;
  usharing_dlp_node::JsonConfigLoader loader;
  std::string config_error;
  if (!loader.Load(ReadConfigDir(argc, argv), &planning_config,
                   &rscl_config, &config_error)) {
    std::cerr << "[smoke] failed to load config: " << config_error
              << std::endl;
    return 1;
  }

  if (CRsclAccess::Instance()->init(argc, argv, rscl_config.node_name) != 0) {
    std::cerr << "[smoke] failed to initialize RSCL" << std::endl;
    return 1;
  }

  int exit_code = 0;
  SmokeCounters counters;
  {
    usharing_dlp_node::PlanningService service;
    auto engine = std::make_unique<CountingPlanningEngine>(&counters);
    auto transport =
        std::make_unique<usharing_dlp_node::rscl::RsclPlanningTransport>(
            rscl_config);
    if (!service.Init(planning_config, std::move(engine),
                      std::move(transport))) {
      std::cerr << "[smoke] failed to initialize planning service" << std::endl;
      exit_code = 1;
    } else {
      service.Start();
      const auto deadline =
          std::chrono::steady_clock::now() +
          std::chrono::seconds(ReadRunSeconds(argc, argv));
      while (g_running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      service.Stop();
    }
  }

  CRsclAccess::Instance()->release();
  PrintCounters(counters);
  return exit_code;
}
