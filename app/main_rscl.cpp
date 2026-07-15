// Ported from:
//   planning_bus/sys_main/sys_main.cpp
//   planning_bus/app/planning/planning.cpp
// Purpose:
//   Standalone RSCL process entry for the migrated Pilot DLP planning port.
//   RSCL is initialized here and remains outside planning_core/planning_sap code.
//
// 中文说明：
//   这是 RSCL 版本可执行程序入口，只负责进程生命周期编排：
//   1. 读取 resource/config/dlp_node/<platform>/ 下的配置；
//   2. 初始化 RSCL runtime；
//   3. 创建 middleware-neutral 的 PlanningService、SapPlanningEngine 和
//      RsclPlanningTransport；
//   4. 让进程常驻，等待 RSCL runtime 在后台触发订阅回调。
//   注意：本文件是允许依赖 RSCL 的 app 边界，planning_core/planning_sap 层不应包含 RSCL。

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <iostream>
#include <limits.h>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include "planning_core/planning_service.hpp"
#include "config/rscl_config.hpp"
#include "planning_sap/sap_planning_engine.hpp"
#include "rscl_access.hpp"
#include "rscl_planning_transport.hpp"

#ifndef USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR
#define USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR "config/dlp_node/x86"
#endif

namespace {

std::atomic_bool g_running{true};

// 信号处理只修改原子标志位，避免在 signal handler 中执行复杂逻辑。
// 主线程会在循环中观察该标志并按顺序 Stop/Release。
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

std::string ReadConfigDir(int argc, char** argv) {
  // 最高优先级：调用方显式指定完整配置目录，便于调试或临时切换配置。
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config_dir") {
      return argv[i + 1];
    }
  }
  // 次优先级：只切换平台名。实际目录固定为 resource/config/dlp_node/<platform>，
  // 这样 x86/thor 的 sys_cfg、planning_cfg、topics_cfg 可以保持同构。
  std::string default_config_subdir = USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config_platform") {
      default_config_subdir = std::string("config/dlp_node/") + argv[i + 1];
    }
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

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  // 加载两类配置：
  // - PlanningConfig：给 sap_camera SDK 使用，描述资源目录、pipeline prototxt 等；
  // - RsclPortConfig：给 RSCL transport 使用，描述订阅/发布 topic。
  usharing_dlp_node::PlanningConfig planning_config;
  usharing_dlp_node::RsclPortConfig rscl_config;
  usharing_dlp_node::JsonConfigLoader loader;
  std::string config_error;
  if (!loader.Load(ReadConfigDir(argc, argv), &planning_config,
                   &rscl_config, &config_error)) {
    std::cerr << "failed to load usharing_dlp_node RSCL config: "
              << config_error << std::endl;
    return 1;
  }

  if (CRsclAccess::Instance()->init(argc, argv, rscl_config.node_name) != 0) {
    std::cerr << "failed to initialize RSCL" << std::endl;
    return 1;
  }

  int exit_code = 0;
  {
    // 依赖注入关系：
    // PlanningService 不关心底层通信是 RSCL/ROS，也不关心算法实现细节；
    // 当前可执行程序在 app 边界选择 RSCL transport 和 sap_camera SDK engine。
    usharing_dlp_node::PlanningService service;
    auto engine = std::make_unique<usharing_dlp_node::SapPlanningEngine>();
    auto transport =
        std::make_unique<usharing_dlp_node::rscl::RsclPlanningTransport>(
            rscl_config);
    if (!service.Init(planning_config, std::move(engine),
                      std::move(transport))) {
      std::cerr << "failed to initialize planning service" << std::endl;
      exit_code = 1;
    } else {
      service.Start();
      // RSCL 订阅回调由 RSCL runtime 内部线程触发。这里的循环只负责保持进程
      // 存活，并在收到 SIGINT/SIGTERM 后退出。
      while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      service.Stop();
    }
  }
  CRsclAccess::Instance()->release();
  return exit_code;
}
