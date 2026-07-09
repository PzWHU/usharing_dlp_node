/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Liu ChenLong <liuchenlong@senseauto.com>
 */
#pragma once

#include <chrono>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <string>
#include <functional>

namespace senseAD {
namespace perception {

using Trace_Clock = std::chrono::steady_clock;

class Timer {
 public:
    explicit Timer(int tid = -1);
    virtual ~Timer() = default;
    uint64_t Start();
    uint64_t CountDuration() const;

 private:
    std::chrono::time_point<Trace_Clock> begin_time_;
    int tid_;
};

class ScopedFuncCall {
 public:
    ScopedFuncCall();
    explicit ScopedFuncCall(std::function <void()>&& at_exit);
    ScopedFuncCall(std::function<void()>&& at_start,
                   std::function<void()>&& at_exit);

    ~ScopedFuncCall();

 private:
    std::function<void()> at_exit_;
};

class TraceWriter {
 public:
    TraceWriter();
    static TraceWriter& Instance();

    ScopedFuncCall Range(const std::string& name);

    void Push(const std::string& name);
    void Pop(const std::string& name);

    inline void Enable() {
        enable_ = true;
        start_ts_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        Trace_Clock::now().time_since_epoch())
                        .count();
        if (!trace_file_.is_open()) {
            std::lock_guard<std::mutex> lock{file_mtx_};
            trace_file_.open("/tmp/logs/trace.json");
            trace_file_ << std::fixed << std::setprecision(3);
            trace_file_ << R"({"traceEvents":[)";
        }
    }
    inline void Disable() {
        enable_ = false;
        if (trace_file_.is_open()) {
            std::lock_guard<std::mutex> lock{file_mtx_};
            trace_file_ << R"(],"displayTimeUnit":"ns"})";
            trace_file_.close();
        }
    }
    inline bool IsEnabled() { return enable_; }
    std::string info_;

 private:
    int pid_ = -1;
    bool enable_ = false;
    uint64_t start_ts_ = 0;
    std::ofstream trace_file_{};
    std::mutex file_mtx_{};
};

}  // namespace perception
}  // namespace senseAD

#define TRACE_RANGE(name) \
    auto _profiler_ =     \
        senseAD::perception::TraceWriter::Instance().Range((#name));
#define TRACE_RANGE_STR(name) \
    auto _profiler_ =         \
        senseAD::perception::TraceWriter::Instance().Range((name));
#define TRACE_PUSH(name) \
    senseAD::perception::TraceWriter::Instance().Push(#name);
#define TRACE_POP(name) senseAD::perception::TraceWriter::Instance().Pop(#name);
#define TRACE_PUSH_STR(name) \
    senseAD::perception::TraceWriter::Instance().Push(name);
#define TRACE_POP_STR(name) \
    senseAD::perception::TraceWriter::Instance().Pop(name);
