// Ported from:
//   planning_bus/data_rscl/rscl_access/rscl_access.cpp
// Purpose:
//   Keep RSCL runtime lifecycle outside planning core and sap adapter code.

#include "rscl_access.hpp"

#include <ad_rscl/component/component_utils.h>
#include <ad_rscl/runtime.h>

CRsclAccess* CRsclAccess::Instance() {
  // 参考 planning_bus 的单例入口，集中持有 RSCL runtime、publisher 和
  // subscriber，避免 transport/core 中散落 runtime 生命周期管理。
  static CRsclAccess instance;
  return &instance;
}

CRsclAccess::CRsclAccess() {
  // 构造阶段只获取当前 runtime 指针；真正 Init 在 main 中显式调用，
  // 这样配置加载失败时不会启动 RSCL。
  runtime_ = senseAD::rscl::GetCurRuntime();
}

CRsclAccess::~CRsclAccess() {
  release();
}

int CRsclAccess::init(int argc, char** argv, const std::string& name) {
  runtime_ = senseAD::rscl::GetCurRuntime();
  if (runtime_ == nullptr) {
    return -1;
  }
  runtime_->Init(argc, argv);

  senseAD::rscl::component::ComponentConfig cfg;
  cfg.timer_conf.set_name(name);
  // Initialize 来自 BaseComponent，用于注册 component 名称和基础回调。
  // 本节点不依赖 SWCFL workshop，只使用 RSCL 原始 pub/sub 能力。
  Initialize(cfg);
  return 0;
}

int CRsclAccess::release() {
  // 先释放 subscriber/publisher 智能指针，再 Shutdown runtime，避免
  // runtime 销毁后仍有回调对象持有底层句柄。
  subscribers_.clear();
  publishers_.clear();

  if (runtime_ != nullptr) {
    // GetCurRuntime()/Runtime::Instance() 返回 RSCL 内部 shared_ptr 单例管理的
    // 对象，本封装只借用指针，不能 delete。
    runtime_->Shutdown();
    runtime_ = nullptr;
  }
  return 0;
}

bool CRsclAccess::OnProc() {
  // BaseComponent 定时处理入口。当前节点没有周期任务，全部输入由
  // subscribe 回调驱动，因此保持 true。
  return true;
}
