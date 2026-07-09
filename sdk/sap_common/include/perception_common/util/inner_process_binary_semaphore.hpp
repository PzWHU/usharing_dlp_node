/*
 * Copyright (C) 2019-20 by SenseTime Group Limited. All rights reserved.
 * Wang Jiadong <wangjiadong@senseauto.com>
 */

#pragma once

#include <string>
#include <mutex>

namespace senseAD {
namespace perception {

/**
 * @brief an inner-process replacement of ecl Semaphore
 */
class InnerProcessBinarySemaphore {
 public:
    InnerProcessBinarySemaphore() {}
    explicit InnerProcessBinarySemaphore(const std::string& string_id) {}

    virtual ~InnerProcessBinarySemaphore() {}

    void lock() { mtx_.lock(); }
    void unlock() { mtx_.unlock(); }
    bool trylock() { return mtx_.try_lock(); }

 private:
    std::mutex mtx_;
};

}  // namespace perception
}  // namespace senseAD
