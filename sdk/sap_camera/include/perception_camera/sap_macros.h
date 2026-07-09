/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

#define MAX_PATH 256
#define MAX_NAME 128
#define MAX_PIPELINE_NUM 20
#define MAX_CAMERA_NUM 11
#define MAX_LIDAR_NUM 3
#define MAX_OP_TIME_COST_NUM 20

#ifdef _WIN32

#ifdef SAP_DLL_BUILD
#define SAP_API__ __declspec(dllexport)
#else
#define SAP_API__ __declspec(dllimport)
#endif

#else
// only support gcc __GNUC__ >= 4 or clang
// need: defined (__GNUC__) && (__GNUC__ >= 4)
#define SAP_API__ __attribute__((visibility("default")))

#endif

#ifdef __cplusplus
#define SAP_API extern "C" SAP_API__
#else
#define SAP_API SAP_API__
#endif
