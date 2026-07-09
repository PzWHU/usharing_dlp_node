// Ported from:
//   adapter/perception_camera/src/workshops/perception_camera/perception_camera_sdk_workshop.cpp
// Purpose:
//   Replace AD_SWCFL_* logging with a middleware-neutral logging facade.
#pragma once

#include <iostream>
#include <string>

namespace usharing_dlp_node {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

// 中间件无关日志接口。SapPlanningEngine 等核心代码只依赖 ILogger，
// 后续可以在 app/transport 层替换为 ROS logger、RSCL logger 或文件日志。
class ILogger {
 public:
  virtual ~ILogger() = default;
  virtual void Log(LogLevel level, const std::string& scope,
                   const std::string& message) = 0;
};

// 默认 stderr 实现用于命令行运行和本地调试，不绑定任何框架。
class StderrLogger final : public ILogger {
 public:
  void Log(LogLevel level, const std::string& scope,
           const std::string& message) override {
    std::cerr << "[" << ToString(level) << "][" << scope << "] " << message
              << std::endl;
  }

 private:
  static const char* ToString(LogLevel level) {
    switch (level) {
      case LogLevel::kDebug:
        return "DEBUG";
      case LogLevel::kInfo:
        return "INFO";
      case LogLevel::kWarn:
        return "WARN";
      case LogLevel::kError:
        return "ERROR";
    }
    return "UNKNOWN";
  }
};

}  // namespace usharing_dlp_node
