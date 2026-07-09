/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Wangzewei1 <wangzewei1@sensetime.com>
 */

#pragma once

#include <list>
#include <memory>
#include <boost/bind/bind.hpp>
#include <boost/noncopyable.hpp>

template <typename T>
class shared_pool : boost::noncopyable {
 public:
    typedef shared_pool<T> pool_type;
    typedef std::shared_ptr<T> pointer;
    typedef std::shared_ptr<const T> const_pointer;
    typedef std::list<T *> list;

    explicit shared_pool(size_t initial_size = 32, size_t grow_by = 32) {
        _grow_by = grow_by;
        make(initial_size);
    }

    void init(size_t initial_size) { make(initial_size); }

    virtual ~shared_pool() { free_pool(); }

    size_t size() const { return _pool.size(); }

    void grow_by(size_t n) { _grow_by = n; }

    pointer clone(const pointer &src) {
        pointer p = create();
        *p = *src;
        return p;
    }

    pointer malloc() { return create(); }

    void free_pool() {
        for (typename list::iterator it = _pool.begin(); it != _pool.end();
             ++it)
            delete *it;
        _pool.clear();
    }

 private:
    void release(T *p) { _pool.push_back(p); }

    void make(size_t n) {
        for (unsigned i = 0; i < n; i++) {
            _pool.push_back(new T());
        }
    }

    pointer create() {
        if (_pool.empty()) {
            make(_grow_by);
            // next size will be twice as big as predecessor
            _grow_by *= 2;
        }

        T *p = _pool.back();
        _pool.pop_back();

        // custom "deleter" puts raw pointer back in pool
        return pointer(
            p, boost::bind(&pool_type::release, this, boost::placeholders::_1));
    }

    list _pool;
    size_t _grow_by;
};
