/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * Chen Shengjie <chenshengjie@sensetime.com>
 */

#pragma once
typedef signed char int8_t;
// typedef signed short int16_t;
typedef signed int int32_t;
// typedef signed long  int64_t;
typedef unsigned char uint8_t;
// typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
// typedef unsigned long  uint64_t;
typedef float float32_t;
typedef double float64_t;
typedef long double float128_t;

namespace senseAD {
namespace perception {

#ifdef __aarch64__
typedef long long int senseStamp;  // NOLINT
#else
typedef long int senseStamp;  // NOLINT
#endif

/**
 * @brief struct for bgr means
 **/
typedef struct {
    float32_t blue;   // blue channel
    float32_t green;  // green channel
    float32_t red;    // red channel
} BgrValues_t;

typedef struct StructValidInfo {
    bool is_valid = false;
#ifdef WITH_SERIALIZATION_INTERFACE
    virtual ~StructValidInfo() = default;
#endif  // WITH_SERIALIZATION_INTERFACE
} ValidInfo;

}  // namespace perception
}  // namespace senseAD
