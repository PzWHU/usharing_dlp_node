/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * xuguanghui <xuguanghui@sensetime.com>
 */
#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <limits>

class Exception {
 public:
    std::string error_info;
    explicit Exception(std::string s) : error_info(s) {}
    std::string what() { return error_info; }
};

template <typename T>
class circle_queue {
 public:
    typedef bool (*cmp)(T&, T&);
    const size_t npos = std::numeric_limits<size_t>::max();

 public:
    circle_queue() : start_(0), end_(0), max_size_(0) {
        data_.resize(max_size_ + 1);
    }
    explicit circle_queue(size_t size) : start_(0), end_(0), max_size_(size) {
        data_.resize(max_size_ + 1);
    }
    bool push(const T& data) {
        std::unique_lock<std::mutex> lock(rw_lock_);
        if (size() == max_size_) {
            std::cerr << "queue is full, auto pop front" << std::endl;
            // throw Exception("queue is full");
            pop();
            // return false;
        }

        data_[end_] = data;
        end_++;
        end_ %= data_.size();
        return true;
    }
    bool push(T&& data) {
        std::unique_lock<std::mutex> lock(rw_lock_);
        if (size() == max_size_) {
            std::cerr << "queue is full, auto pop front" << std::endl;
            // throw Exception("queue is full");
            // return false;
            pop();
        }

        data_[end_] = data;
        end_++;
        end_ %= data_.size();
        return true;
    }
    bool pop() {
        std::unique_lock<std::mutex> lock(rw_lock_);
        if (empty()) {
            // throw Exception("queue is full");
            return false;
        }
        start_++;
        start_ %= data_.size();
        return true;
    }
    /// 下标索引方式有可能导致返回数据不安全...
    T& front() {
        if (empty()) {
            AP_LERROR(__FUNCTION__)
                << "queue is full, can't call front" << std::endl;
            throw Exception("queue is empty, can not return front");
        }
        return data_[start_];
    }
    T& back() {
        if (empty()) {
            AP_LERROR(__FUNCTION__)
                << "queue is full, can't call back" << std::endl;
            throw Exception("queue is empty, can not return back");
        }
        return data_[(end_ + data_.size() - 1) % data_.size()];
    }
    T& operator[](size_t idx) {
        if (empty()) {
            AP_LERROR(__FUNCTION__) << "out of range" << std::endl;
            throw Exception("queue is empty, out of range");
        }
        if (idx >= size()) {
            AP_LERROR(__FUNCTION__) << "out of range" << std::endl;
            throw Exception("out of range");
        }
        return data_[(start_ + idx) % data_.size()];
    }

    size_t rfind(const T& data, cmp cmp_func) {
        for (size_t i = (*this).size() - 1; i >= 0; i--) {
            if (cmp_func(data, (*this)[i])) {
                return i;
            }
        }
        return npos;
    }
    size_t lfind(const T& data, cmp cmp_func) {
        for (size_t i = 0; i <= (*this).size() - 1; i++) {
            if (cmp_func(data, (*this)[i])) {
                return i;
            }
        }
        return npos;
    }
    bool expand(size_t size) {
        if (size <= max_size_) {
            return false;
        }

        data_.resize(size + 1);
        if (end_ < start_) {
            memcpy(data_.data() + max_size_ + 1, data_.data(),
                   end_ * sizeof(T));
            end_ += max_size_;
        }
        max_size_ = size;
        return true;
    }
    void clear() {
        start_ = 0;
        end_ = 0;
    }
    size_t size() { return (end_ + data_.size() - start_) % data_.size(); }
    bool empty() { return size() == 0; }
    bool full() { return size() == max_size_; }

 private:
    std::vector<T> data_;
    size_t start_, end_;
    size_t max_size_;
    std::mutex rw_lock_;
};
