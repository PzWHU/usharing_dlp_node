/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * He Yuan <heyuan@sensetime.com>
 */

#pragma once

#include <iostream>

// #define HAF_LOG

#ifdef HAF_LOG
#include <unistd.h>
#include "ad_log/ad_log.hpp"
#endif


#ifdef HAF_LOG
#define AP_LTRACE(tag)      AD_LTRACE(tag)
#define AP_LDEBUG(tag)      AD_LDEBUG(tag)
#define AP_LINFO(tag)       AD_LINFO(tag)
#define AP_LERROR(tag)      AD_LERROR(tag)
#define AP_LWARN(tag)       AD_LWARN(tag)
#define AP_LFATAL(tag)      AD_LFATAL(tag)

#define AP_LWARN_EVERY(tag, n) AD_LWARN_EVERY(tag, n)
#define AP_LWARN_EVERY_SEC(tag, n) AD_LWARN_EVERY_SEC(tag, n)
#define AP_LERROR_EVERY_SEC(tag, n) AD_LERROR_EVERY_SEC(tag, n)

#define AP_FTRACE(tag)      AD_FTRACE(tag)
#define AP_FDEBUG(tag)      AD_FDEBUG(tag)
#define AP_FINFO(tag)       AD_FINFO(tag)
#define AP_FERROR(tag)      AD_FERROR(tag)
#define AP_FWARN(tag)       AD_FWARN(tag)
#define AP_FFATAL(tag)      AD_FFATAL(tag)
#else

#if defined(WITH_AD_COMMON) || defined(WITH_TDA)
#ifndef BUILD_ADAS
#include "ad_log/ad_log.hpp"
#endif
#endif

#ifdef BUILD_ADAS
#include "log.hpp"  // NOLINT
#endif  // BUILD_ADAS

#ifdef WITH_AD_COMMON
#define AP_LTRACE(tag) AD_LTRACE(tag)
#define AP_LDEBUG(tag) AD_LDEBUG(tag)
#define AP_LINFO(tag) AD_LINFO(tag)
#define AP_LERROR(tag) AD_LERROR(tag)
#define AP_LWARN(tag) AD_LWARN(tag)
#define AP_LFATAL(tag) AD_LFATAL(tag)

#define AP_LWARN_EVERY(tag, n) AD_LWARN_EVERY(tag, n)
#define AP_LERROR_EVERY_SEC(tag, n) AD_LERROR_EVERY_SEC(tag, n)
#endif  // WITH_AD_COMMON

#ifdef BUILD_ADAS
// define only use in adas for compile
#define FLOGS_TRACE(tag) \
    std::cout << std::endl << #tag << "::(not implemented!)"
#define FLOGS_DEBUG(tag) \
    std::cout << std::endl << #tag << "::(not implemented!)"
#define FLOGS_INFO(tag) std::cout << std::endl << #tag << "::(not implemented!)"
#define FLOGS_ERROR(tag) \
    std::cout << std::endl << #tag << "::(not implemented!)"
#define FLOGS_WARN(tag) std::cout << std::endl << #tag << "::(not implemented!)"
#define FLOGS_CRITICAL(tag) \
    std::cout << std::endl << #tag << "::(not implemented!)"

#define AD_LWARN_EVERY(tag, n) SD_LWARN(tag)
#define AD_LERROR_EVERY_SEC(tag, n) SD_LERROR(tag)
#ifdef WITH_QCOM
#define AD_LTRACE(tag) std::cout << std::endl << #tag << "::"
#define AD_LDEBUG(tag) std::cout << std::endl << #tag << "::"
#define AD_LINFO(tag) std::cout << std::endl << #tag << "::"
#define AD_LERROR(tag) std::cout << std::endl << #tag << "::"
#define AD_LWARN(tag) std::cout << std::endl << #tag << "::"
#define AD_LFATAL(tag) std::cout << std::endl << #tag << "::"
// define only use in adas for compile
#define AD_FTRACE(tag) FLOGS_TRACE(tag)
#define AD_FDEBUG(tag) FLOGS_DEBUG(tag)
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)

#elif defined(WITH_TDA) || defined(WITH_TDA)
#define AD_LTRACE(tag) std::cout << std::endl << "[" << #tag << "]: "
#define AD_LDEBUG(tag) std::cout << std::endl << "[" << #tag << "]: "
#define AD_LINFO(tag) std::cout << std::endl << "[" << #tag << "]: "
#define AD_LERROR(tag) std::cout << std::endl << "[" << #tag << "]: "
#define AD_LWARN(tag) std::cout << std::endl << "[" << #tag << "]: "
#define AD_LFATAL(tag) std::cout << std::endl << "[" << #tag << "]: "

// define only use in adas for compile
#define AD_FTRACE(tag) FLOGS_TRACE(tag)
#define AD_FDEBUG(tag) FLOGS_DEBUG(tag)
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)

#elif defined WITH_FPGA
#ifdef ENABLE_LOG
#define AD_LTRACE(tag) std::cout << std::endl << #tag << "::"
#define AD_LDEBUG(tag) std::cout << std::endl << #tag << "::"
#define AD_LINFO(tag) std::cout << std::endl << #tag << "::"
#define AD_LERROR(tag) std::cout << std::endl << #tag << "::"
#define AD_LWARN(tag) std::cout << std::endl << #tag << "::"
#define AD_LFATAL(tag) std::cout << std::endl << #tag << "::"
// define only use in adas for compile
#define AD_FTRACE(tag) FLOGS_TRACE(tag)
#define AD_FDEBUG(tag) FLOGS_DEBUG(tag)
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)

#else  // ENABLE_LOG
#define AD_LTRACE(tag) SD_LTRACE(tag)
#define AD_LDEBUG(tag) SD_LDEBUG(tag)
#define AD_LINFO(tag) SD_LINFO(tag)
#define AD_LERROR(tag) SD_LERROR(tag)
#define AD_LWARN(tag) SD_LWARN(tag)
#define AD_LFATAL(tag) SD_LFATAL(tag)
#define AD_FTRACE(tag) FLOGS_TRACE(tag)
#define AD_FDEBUG(tag) FLOGS_DEBUG(tag)
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)
#endif  // ENABLE_LOG
#else   // WITH_FPGA
#define AD_LTRACE(tag) SD_LTRACE(tag)
#define AD_LDEBUG(tag) SD_LDEBUG(tag)
#define AD_LINFO(tag) SD_LINFO(tag)
#define AD_LERROR(tag) SD_LERROR(tag)
#define AD_LWARN(tag) SD_LWARN(tag)
#define AD_LFATAL(tag) SD_LFATAL(tag)
#define AD_FTRACE(tag) FLOGS_TRACE(tag)
#define AD_FDEBUG(tag) FLOGS_DEBUG(tag)
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)
#endif  // WITH_FPGA
#endif  // BUILD_ADAS

#define AP_LTRACE(tag) AD_LTRACE(tag)
#define AP_LDEBUG(tag) AD_LDEBUG(tag)
#define AP_LINFO(tag) AD_LINFO(tag)
#define AP_LERROR(tag) AD_LERROR(tag)
#define AP_LWARN(tag) AD_LWARN(tag)
#define AP_LFATAL(tag) AD_LFATAL(tag)
#define AP_LWARN_EVERY(tag, n) AD_LWARN_EVERY(tag, n)
#define AP_LWARN_EVERY_SEC(tag, n) AD_LWARN_EVERY_SEC(tag, n)
#define AP_LERROR_EVERY_SEC(tag, n) AD_LERROR_EVERY_SEC(tag, n)

// with the prefix of AD_F  it will log into background log file
#define AP_FTRACE(tag) AD_FTRACE(tag)
#define AP_FDEBUG(tag) AD_FDEBUG(tag)
#define AP_FINFO(tag) AD_FINFO(tag)
#define AP_FERROR(tag) AD_FERROR(tag)
#define AP_FWARN(tag) AD_FWARN(tag)
#define AP_FFATAL(tag) AD_FFATAL(tag)

#endif
