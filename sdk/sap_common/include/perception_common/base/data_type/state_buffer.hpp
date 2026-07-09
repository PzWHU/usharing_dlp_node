/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <map>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Cholesky"
#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/state/KFStateVec.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"

namespace senseAD {
namespace perception {

template <class T>
class StateBuffer {
 public:
    typedef typename std::map<uint64_t, T>::iterator StateIterator;

 public:
    StateBuffer() : invalid_time(0) {
        T invalid_value;
        state_buffer_.insert(std::make_pair(invalid_time, invalid_value));
    }

    ~StateBuffer() = default;

 public:
    StateIterator GetIteratorClosestBefore(uint64_t timestamp_ns) {
        if (0ULL == timestamp_ns) {
            AP_LERROR(GetIteratorClosestBefore)
                << "the query timestmap is " << timestamp_ns;
            return state_buffer_.end();
        }

        if (state_buffer_.empty()) {
            AP_LWARN("StateBuffer") << "state buffer is empty";
            return state_buffer_.end();
        }
        typename StateBuffer<T>::StateIterator lower_iter =
            state_buffer_.lower_bound(timestamp_ns);

        lower_iter--;
        return lower_iter;
    }

    StateIterator GetIteratorClosestAfter(uint64_t timestamp_ns) {
        StateIterator upper_iter = state_buffer_.upper_bound(timestamp_ns);
        return upper_iter;
    }

    StateIterator GetIteratorClosest(uint64_t timestamp_ns) {
        typename StateBuffer<T>::StateIterator cur_iter =
            state_buffer_.find(timestamp_ns);
        if (cur_iter != state_buffer_.end()) return cur_iter;

        typename StateBuffer<T>::StateIterator lower_iter =
            GetIteratorClosestBefore(timestamp_ns);
        typename StateBuffer<T>::StateIterator upper_iter =
            GetIteratorClosestAfter(timestamp_ns);

        if (lower_iter == GetIteratorBeginBefore() ||
            lower_iter == state_buffer_.end())
            return upper_iter;

        if (upper_iter == state_buffer_.end()) return lower_iter;

        uint64_t lower_time_diff = timestamp_ns - lower_iter->first;
        uint64_t upper_time_diff = upper_iter->first - timestamp_ns;
        if (lower_time_diff > upper_time_diff)
            return upper_iter;
        else
            return lower_iter;
        return lower_iter;
    }

    StateIterator GetIteratorBegin() {
        if (state_buffer_.empty()) {
            AP_LDEBUG("StateBuffer") << "StateBuffer is empty";
            return state_buffer_.end();
        }
        auto iter = state_buffer_.begin();
        iter++;
        return iter;
    }

    StateIterator GetIteratorEnd() { return state_buffer_.end(); }

    StateIterator GetIteratorAtTime(uint64_t timestamp_ns) {
        StateIterator iter = state_buffer_.find(timestamp_ns);
        return iter;
    }

    StateIterator GetIteratorBeginBefore() {
        StateIterator iter = state_buffer_.begin();
        return iter;
    }

    StateIterator GetIteratorEndBefore() {
        if (state_buffer_.empty()) {
            AP_LWARN("StateBuffer") << "buffer is empty";
            return state_buffer_.end();
        }

        auto iter = state_buffer_.end();
        iter--;
        return iter;
    }

    apStatus_t ClearOldState(uint64_t timestamp_ns) {
        if (state_buffer_.size() < 2) {
            AP_LINFO("StateBuffer") << "buffer size is less than 2";
            return AP_SUCCESS;
        }
        auto iter = state_buffer_.begin();
        iter++;
        uint64_t time_last = GetIteratorEndBefore()->first;
        typename StateBuffer<T>::StateIterator iter_c =
            GetIteratorClosest(time_last - timestamp_ns);

        if (time_last - iter->first < timestamp_ns) {
            return AP_SUCCESS;
        }
        if (iter_c != iter) state_buffer_.erase(iter, iter_c);
        return AP_SUCCESS;
    }

    apStatus_t ClearStateOlderThanNS(uint64_t time_ns) {
        if (GetIteratorEndBefore() == state_buffer_.end()) {
            return AP_SUCCESS;
        }
        uint64_t time_latest = GetIteratorEndBefore()->first;
        if (time_ns > time_latest) {
            return AP_SUCCESS;
        }
        uint64_t duration = time_latest - time_ns;
        return ClearOldState(duration);
    }

    apStatus_t ClearState() {
        if (state_buffer_.empty()) return AP_SUCCESS;
        auto iter = state_buffer_.begin();
        iter++;
        state_buffer_.erase(iter, state_buffer_.end());
        return AP_SUCCESS;
    }

    apStatus_t Erase(const StateIterator& begin, const StateIterator& end) {
        state_buffer_.erase(begin, end);
        return AP_SUCCESS;
    }

    apStatus_t EraseByTimeNs(const uint64_t timestamp_ns) {
        if (state_buffer_.find(timestamp_ns) != state_buffer_.end()) {
            state_buffer_.erase(timestamp_ns);
        }
        return AP_SUCCESS;
    }

    StateIterator Insert(const uint64_t& timestamp_ns, const T& state_ptr) {
        std::pair<StateIterator, bool> iter_bool;
        iter_bool =
            state_buffer_.insert(std::make_pair(timestamp_ns, state_ptr));
        if (!iter_bool.second) {
            AP_LDEBUG(StateBuffer<T>)
                << " state buffer has contained the state at: " << timestamp_ns
                << " insert failed";
        }
        return iter_bool.first;
    }

    bool Empty() {
        int buffer_size = state_buffer_.size() - 1;
        if (buffer_size == 0)
            return true;
        else
            return false;
        return false;
    }

    int GetSize() { return state_buffer_.size() - 1; }

    std::map<uint64_t, T> GetStateBuffer() const { return state_buffer_; }

    uint64_t GetInvalidTime() const { return invalid_time; }

 public:
    uint64_t invalid_time;

 public:
    std::map<uint64_t, T> state_buffer_;
};
}  // namespace perception
}  // namespace senseAD
