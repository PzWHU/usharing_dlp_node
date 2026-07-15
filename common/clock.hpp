// Ported from:
//   adapter/perception_camera/include/perception_camera_helper.hpp
// Purpose:
//   Replace RsclNowNs/RsclSteadyNowNs with middleware-neutral clock interfaces.
#pragma once

#include <chrono>
#include <cstdint>

namespace usharing_dlp_node {

// 中间件无关时间接口。planning_core/planning_sap 层只能依赖该接口或 SystemClock，
// 不能直接调用 ROS/RSCL 的时间 API。
class IClock {
 public:
  virtual ~IClock() = default;
  // 系统时间，通常用于写入对外消息 header/timestamp。
  virtual uint64_t NowNs() const = 0;
  // 单调时间，适合做耗时统计，避免系统时间回拨影响延迟计算。
  virtual uint64_t SteadyNowNs() const = 0;
};

// 默认实现仅依赖 C++ 标准库，保证 usharing_dlp_node 在无 ROS/RSCL 的
// 单元测试或 mock transport 中也能获取时间。
class SystemClock final : public IClock {
 public:
  uint64_t NowNs() const override {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  uint64_t SteadyNowNs() const override {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }
};

}  // namespace usharing_dlp_node
