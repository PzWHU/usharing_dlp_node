/*
 * Copyright (C) 2019-20 by SenseTime Group Limited. All rights reserved.
 * Wang Jiadong <wangjiadong@senseauto.com>
 */

#pragma once

#include <string>
#include <memory>

namespace senseAD {
namespace perception {

/**
 * @brief an inner-process replacement of ecl Shared Memory
 */
template <typename Storage>
class InnerProcessMemory {
 public:
    explicit InnerProcessMemory(const std::string& string_id,
                                const int& flag = O_RDWR) {
        storage = static_cast<Storage*>(malloc(shared_memory_size));
        memset(storage, 0, shared_memory_size);
    }
    ~InnerProcessMemory() {
        if (storage != nullptr) {
            free(storage);
        }
    }
    void SetMode(bool mode) {}
    Storage* data() { return storage; }

 private:
    InnerProcessMemory() {}
    const int shared_memory_size = sizeof(Storage);
    Storage* storage;
};

}  // namespace perception
}  // namespace senseAD
