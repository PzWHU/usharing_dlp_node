/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 */
#pragma once

#include <memory>

namespace senseAD {
namespace perception {

#define ALWAYS_INLINE inline __attribute__((always_inline))
#define NO_INSTRUMENT __attribute__((no_instrument_function))
#define CONSTRUCTOR_FIRST __attribute__((constructor(101)))
#define DESTRUCTOR_FIRST __attribute__((destructor(101)))
#define CONSTRUCTOR __attribute__((constructor(102)))
#define DESTRUCTOR __attribute__((destructor(102)))

#define DISABLE_COPY_AND_ASSIGN(classname)            \
 private:                                             \
    classname &operator=(const classname &) = delete; \
    classname(const classname &) = delete

#define INSTANCE_CLASS(classname)           \
    template class classname<bool>;         \
    template class classname<uint8_t>;      \
    template class classname<int>;          \
    template class classname<unsigned int>; \
    template class classname<float>;        \
    template class classname<double>

#ifdef DISALLOW_COPY_AND_ASSIGN
#undef DISALLOW_COPY_AND_ASSIGN
#endif
// Macros for Singleton
#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
    classname(const classname &);           \
    classname &operator=(const classname &);

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
    classname();                                  \
    DISALLOW_COPY_AND_ASSIGN(classname);


#ifdef DECLARE_SINGLETON
#undef DECLARE_SINGLETON
#endif
#define DECLARE_SINGLETON(classname)          \
 public:                                      \
    static classname *instance() {            \
        static classname instance;            \
        return &instance;                     \
    }                                         \
    DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:

// Define Smart ptr
#define DEF_SMART_PTR(...)      \
    DEF_SHARED_PTR(__VA_ARGS__) \
    DEF_UNIQUE_PTR(__VA_ARGS__) \
    DEF_WEAK_PTR(__VA_ARGS__)

#define DEF_SHARED_PTR(...)                         \
    using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
    using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define DEF_UNIQUE_PTR(...)                         \
    using UniquePtr = std::unique_ptr<__VA_ARGS__>; \
    using ConstUniquePtr = std::unique_ptr<const __VA_ARGS__>;

#define DEF_WEAK_PTR(...)                       \
    using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
    using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

}  // namespace perception
}  // namespace senseAD
