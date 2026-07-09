/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * xuguanghui <xuguanghui@sensetime.com>
 */

#ifndef AUX_TIMER_UTILS_H  // NOLINT
#define AUX_TIMER_UTILS_H

#include <string>
#include <chrono>
#include <mutex>
#include <thread>
class Timer {
 public:
    /// @param name Name of timer to be started.
    /// @note A std::runtime_error exception will be thrown if a timer is
    /// started twice.
    __attribute__((visibility("default"))) static void Start(
        const std::string name);

    /// @brief  Stop the given timer.
    /// @param name Name of timer to be stopped.
    __attribute__((visibility("default"))) static void Stop(
        const std::string name);

    /// @brief Get the value of the given timer.
    /// @param name Name of timer to return value of.

    static std::chrono::microseconds Get(const std::string name);

    /// @brief  Prints the specified timer.  If it took longer than a minute to
    /// complete the timer will be displayed in days, hours, and minutes as
    /// well.
    /// @param name of timer in question
    __attribute__((visibility("default"))) static void PrintTimer(
        const std::string name);
    __attribute__((visibility("default"))) static void PrintAllTimer();
    static std::mutex _lock;
};

#endif  // NOLINT
