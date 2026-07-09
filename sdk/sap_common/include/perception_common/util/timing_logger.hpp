/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Kent Nakajima <nakajima@sensetime.com>
 * GUO Zhichong <guozhichong@sensetime.com>
 * Liang Yu <liangyu@sensetime.com>
 */

#pragma once

#ifndef BUILD_ADAS

#include <fcntl.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <thread>  // NOLINT
#include <deque>

#ifdef USE_NVTX
#include "nvtx3/nvToolsExt.h"
#endif

#include "ad_time.hpp"  // NOLINT
#include "ad_log.hpp"  // NOLINT
#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

#define TIMING_LOGGER_WRITE_BUFFER_SIZE 128

class TimingLogger {
 public:
    /**
     * @brief init timinglogger
     * @param  file log file
     * @return flag to specify if succeed or fail
     */
    apStatus_t Init(std::string file);

    void Destroy();

    bool IsInited();

    FILE* Handle();

 private:
    FILE* file_ = nullptr;
};

class TimingLoggerGuard {
 public:
    /**
     * @brief start measure time
     * @param logger using logger to measure
     * @param tag  measure the time tag
     * @note logger need to inited.
     */
    explicit TimingLoggerGuard(TimingLogger* logger, const char* tag);

    /**
     * @brief end measure time and write to log file
     */
    ~TimingLoggerGuard();

 private:
    TimingLogger* logger_ = nullptr;
    const char* tag_ = nullptr;
    senseAD::Time start_;
    char buffer_[TIMING_LOGGER_WRITE_BUFFER_SIZE];
};

class TimingLoggerSharedGuard {
 public:
    /**
     * @brief Init
     * @param logger using logger to measure
     * @note logger need to inited.
     */
    explicit TimingLoggerSharedGuard(TimingLogger* logger);

    /**
     * @brief end measure time and write to log file
     */
    ~TimingLoggerSharedGuard();

    /**
     * @brief start measure time
     * @param tag  measure the time tag
     */
    void Start(const char* tag);

    /**
     * @brief start measure time
     * @param tag  measure the time tag
     */
    void End();

 private:
    TimingLogger* logger_ = nullptr;
    const char* tag_ = nullptr;
    char buffer_[TIMING_LOGGER_WRITE_BUFFER_SIZE];
    std::deque<senseAD::Time> starts_;
#ifdef USE_NVTX
    nvtxRangeId_t nvtx_id_;
#endif
};

void TimeSequenceLog(TimingLogger* logger, const char* tag);

TimingLogger* GetTimingLoggerContext();
TimingLogger* GetTimeSeqLoggerContext();

#define TIME_LOGGER_CONTEXT senseAD::perception::GetTimingLoggerContext()
#define SEQ_LOGGER_CONTEXT senseAD::perception::GetTimeSeqLoggerContext()

#define TIME_SEQ_LOG(handle, tag) TimeSequenceLog(handle, log)

}  // namespace perception
}  // namespace senseAD

#endif  // BUILD_ADAS
