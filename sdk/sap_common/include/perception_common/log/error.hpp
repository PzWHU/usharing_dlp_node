/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * He Yuan <heyuan@sensetime.com>
 */

#pragma once

#include <string>
#include <cassert>  // assert

#ifdef USE_CUDA
#include <cuda_runtime.h>
#endif  // USE_CUDA

#include "perception_common/log/common_log.hpp"

// Tmp Version for Cmake & Make
namespace senseAD {
namespace perception {

typedef enum {
    AP_SUCCESS = 0,
    AP_INVALID_PARAM = -1,
    AP_INVALID_DESC = -2,
    // memory allocation failed
    AP_ALLOC_MEMORY_FAILED = -3,
    AP_INTERNAL_ERROR = -4,
    // doesn't support the image format
    AP_INVALID_PIXEL_FORMAT = -5,
    // the model file is not exist or has no read permission
    AP_FILE_NOT_FOUND = -6,
    // invalid model file format
    AP_INVALID_FILE_FORMAT = -7,
    AP_NOT_IMPLEMENTED = -8,
    AP_INVALID_BLOB_NAMES = -9,
    AP_INVALID_INPUT_BLOBS = -10,
    AP_SELECTOR_INIT_FAILD = -11,
    AP_TRANSFORMER_INIT_FAILD = -12,
    AP_CURVEFITTER_INIT_FAILD = -13,
    AP_INVALID_HEIGHT = -14,
    AP_INVALID_WIDTH = -15,
    AP_NULL_PTR = -16,
    AP_INVALID_SELECTOR_TYPE = -17,
    AP_INVALID_TRANSFORMER_TYPE = -18,
    AP_INVALID_CURVEFITTER_TYPE = -19,
    AP_HERMITE_CUBIC_SPLINE_TO_PP_ERROR = -20,
    AP_HERMITE_CUBIC_SPLINE_GET_KNOTS_ERROR = -21,
    AP_HERMITE_CUBIC_SPLINE_GET_REG_ERROR = -22,
    AP_HERMITE_CUBIC_SPLINE_MAT_L1_NORM_ERROR = -23,
    AP_SAMPLING_INTERVAL_OUT_OF_RANGE = -24,
    AP_BGR_MEANS_ERROR = -25,
    AP_BGR_MEANS_NOT_INIT = -26,
    AP_INPUT_BLOB_NOT_INIT = -27,
    AP_OUTPUT_BLOB_NOT_INIT = -28,
    AP_LOAD_MODEL_ERROR = -29,
    AP_LOAD_LICENSE_ERROR = -30,
    AP_LOAD_MODEL_PARAMETERS_JSON_ERROR = -31,
    AP_LOAD_MODEL_ENCRYPTION_MODEL_ERROR = -32,
    AP_GET_UUID_FAILED = -33,
    AP_PREDICTOR_INIT_FAILED = -34,
    AP_READ_RECTIFY_PARAM_ERROR = -35,
    AP_GET_HOMOGRAPHY_ERROR = -36,
    AP_CREATE_HANDLE_ERROR = -37,
    AP_TRACK_FAILED = -38,
    AP_CANBUS_ERROR = -39,
    AP_CANBUS_VEHICLE_ERROR = -40,
    AP_CANBUS_CAN_CLIENT_ERROR = -41,
    AP_END_OF_FILE = -42,
    AP_DW_INIT_FAILED = -43,
    AP_VIDEO_OPEN_ERROR = -44,
    AP_VIDEO_CLOSE_ERROR = -45,
    AP_CAMERA_OPEN_ERROR = -46,
    AP_CAMERA_CLOSE_ERROR = -47,
    AP_CAMERA_READ_ERROR = -48,
    AP_CAMERA_RECORD_START_ERROR = -49,
    AP_CAMERA_RECORD_STOP_ERROR = -50,
    AP_INS_ERROR = -51,
    AP_INS_PARSER_ERROR = -52,
    AP_GNSS_TRANSFORMER_ERROR = -53,
    AP_ROUTING_ERROR = -54,
    AP_ROUTING_LOCALIZATION_MISS = -55,
    AP_ROUTING_TARGET_REACH = -56,
    AP_CANBUS_TIME_OUT_ERROR = -57,
    AP_HDMAP_ERROR = -58,
    AP_CALIB_CONFIG_ERROR = -59,
    AP_TF_TREE_INIT_ERROR = -60,
    AP_LOCALIZATION_ERROR = -61,
    AP_TIME_OUT_ERROR = -62,
    AP_DATA_NO_UPDATE_ERROR = -63,
    AP_NO_INIT_ERROR = -64,
    AP_INVALID_VALUE = -65,
} apStatus_t;

#define AP_ASSERT(condition, message)                                    \
    do {                                                                 \
        AP_LERROR() << "ASSERT FAILED: " << #condition << " " << message \
                  << " @ " << __FILE__ << " (" << __LINE__ << ")"        \
                  << std::endl;                                          \
        assert(false);                                                   \
    } while (0)

#ifdef USE_CUDA
#define CUDA_CHECK(condition)                                  \
    do {                                                       \
        cudaError_t error = condition;                         \
        if (error != cudaSuccess) {                            \
            AP_ASSERT((condition), cudaGetErrorString(error)); \
        }                                                      \
    } while (0)
#endif

#define __GET_ARG4(arg1, arg2, arg3, arg4, ...) arg4

inline std::string SenseDriveGetStatusString(apStatus_t error) {
    switch (error) {
        case AP_SUCCESS:
            return "Success";
        case AP_INVALID_PARAM:
            return "Invalid arguments";
        case AP_INVALID_DESC:
            return "Invalid descriptor";
        case AP_ALLOC_MEMORY_FAILED:
            return "Failed memory allocation";
        case AP_INTERNAL_ERROR:
            return "Internal error";
        case AP_INVALID_PIXEL_FORMAT:
            return "Invalid image format";
        case AP_FILE_NOT_FOUND:
            return "File not found";
        case AP_INVALID_FILE_FORMAT:
            return "Invalid file format";
        case AP_NOT_IMPLEMENTED:
            return "Not implemented";
        case AP_INVALID_BLOB_NAMES:
            return "invalid blob names";
        case AP_INVALID_INPUT_BLOBS:
            return "invalid input blobs";
        case AP_SELECTOR_INIT_FAILD:
            return "selector init failed";
        case AP_TRANSFORMER_INIT_FAILD:
            return "transformer init failed";
        case AP_CURVEFITTER_INIT_FAILD:
            return "curve fitter init failed";
        case AP_INVALID_HEIGHT:
            return "invalid height";
        case AP_INVALID_WIDTH:
            return "invalid width";
        case AP_NULL_PTR:
            return "null pointer";
        case AP_INVALID_SELECTOR_TYPE:
            return "invalid selector type";
        case AP_INVALID_TRANSFORMER_TYPE:
            return "invalid transformer type";
        case AP_INVALID_CURVEFITTER_TYPE:
            return "invalid curve fitter type";
        case AP_HERMITE_CUBIC_SPLINE_TO_PP_ERROR:
            return "hermite cubic spline to pp failed";
        case AP_HERMITE_CUBIC_SPLINE_GET_KNOTS_ERROR:
            return "hermite cubic spline get knots failed";
        case AP_HERMITE_CUBIC_SPLINE_GET_REG_ERROR:
            return "hermite cubic spline get reg failed";
        case AP_HERMITE_CUBIC_SPLINE_MAT_L1_NORM_ERROR:
            return "hermite cubic spline mat l1 norm failed";
        case AP_SAMPLING_INTERVAL_OUT_OF_RANGE:
            return "sampling interval out of range";
        case AP_BGR_MEANS_ERROR:
            return "bgr means error, you should set bgr means correctly first";
        case AP_LOAD_MODEL_ERROR:
            return "load model failed, maybe model path invalid";
        case AP_LOAD_LICENSE_ERROR:
            return "add license failed";
        case AP_LOAD_MODEL_PARAMETERS_JSON_ERROR:
            return "load model parameters.json failed";
        case AP_LOAD_MODEL_ENCRYPTION_MODEL_ERROR:
            return "load model in encryption failed";
        case AP_GET_UUID_FAILED:
            return "get uuid failed";
        case AP_CANBUS_ERROR:
            return "canbus error";
        case AP_CANBUS_VEHICLE_ERROR:
            return "canbus vehicle error";
        case AP_CANBUS_CAN_CLIENT_ERROR:
            return "canbus can client error";
        case AP_END_OF_FILE:
            return "reach end of file";
        case AP_INS_ERROR:
            return "ins error";
        case AP_INS_PARSER_ERROR:
            return "ins parser error";
        case AP_GNSS_TRANSFORMER_ERROR:
            return "gnss transformer error";
        case AP_ROUTING_ERROR:
            return "routing error";
        case AP_ROUTING_LOCALIZATION_MISS:
            return "routing localization missed";
        case AP_ROUTING_TARGET_REACH:
            return "routing target reached";
        case AP_CANBUS_TIME_OUT_ERROR:
            return "canbus timeout error";
        case AP_HDMAP_ERROR:
            return "hdmap error";
        case AP_CALIB_CONFIG_ERROR:
            return "calib config error";
        case AP_TF_TREE_INIT_ERROR:
            return "tf tree init error";
        case AP_LOCALIZATION_ERROR:
            return "localization error";
        case AP_TIME_OUT_ERROR:
            return "time out error";
        case AP_DATA_NO_UPDATE_ERROR:
            return "data no update";
        case AP_NO_INIT_ERROR:
            return "no init error";
        default:
            return "";
    }
}

#ifdef HAF_LOG
#define PRINT_SENSEDRIVE_ERROR_MESSAGE(msg) AD_LERROR("{}")<<msg
#else
#define PRINT_SENSEDRIVE_ERROR_MESSAGE(msg) LOGP_ERROR("{}", msg)
#endif

/// 3 in 1 Checker
#define __GET_ARG4(arg1, arg2, arg3, arg4, ...) arg4
#ifdef AP_CHECK_SUCCESS
#undef AP_CHECK_SUCCESS
#endif  // AP_CHECK_SUCCESS
#define AP_CHECK_SUCCESS(err)                              \
    {                                                      \
        auto err_code = err;                               \
        if (senseAD::perception::AP_SUCCESS != err_code) { \
            PRINT_SENSEDRIVE_ERROR_MESSAGE(                \
                SenseDriveGetStatusString(err_code));      \
            return err_code;                               \
        }                                                  \
    }

#ifdef AP_CHECK_ERRORCODE
#undef AP_CHECK_ERRORCODE
#endif  // AP_CHECK_ERRORCODE
#define AP_CHECK_ERRORCODE(err, err_code)                                   \
    {                                                                       \
        if (err_code != err) {                                              \
            PRINT_SENSEDRIVE_ERROR_MESSAGE(SenseDriveGetStatusString(err)); \
            return err;                                                     \
        }                                                                   \
    }
#ifdef AP_CHECK_ERROR_WITH_MESSAGE
#undef AP_CHECK_ERROR_WITH_MESSAGE
#endif  // AP_CHECK_ERROR_WITH_MESSAGE
#define AP_CHECK_ERROR_WITH_MESSAGE(err, err_code, msg)                 \
    {                                                                   \
        auto err_c = err;                                               \
        if (err_code != err_c) {                                        \
            LOGP_ERROR("{} {}", SenseDriveGetStatusString(err_c), msg); \
            return err_c;                                               \
        }                                                               \
    }

#define AP_CHECK_ERROR(...)                                                  \
    __GET_ARG4(__VA_ARGS__, AP_CHECK_ERROR_WITH_MESSAGE, AP_CHECK_ERRORCODE, \
               AP_CHECK_SUCCESS)                                             \
    (__VA_ARGS__)  /// end 3 in 1 checker

#ifdef USE_CUDA
#define GPU_ERROR_CHECK(ans) \
    { GpuAssert((ans), __FILE__, __LINE__); }

inline void GpuAssert(cudaError_t code,
                      const char *file,
                      int line,
                      bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
                line);
        if (abort) exit(code);
    }
}

#define PRINT_SENSEDRIVE_CUDA_ERROR_MESSAGE(cuda_error_code) \
    LOGP_ERROR("cuda error {}", cuda_error_code)

#define AP_CHECK_CUDA_ERROR(err)                      \
    {                                                 \
        cudaError_t pst = err;                        \
        if (cudaSuccess != pst) {                     \
            PRINT_SENSEDRIVE_CUDA_ERROR_MESSAGE(pst); \
            return AP_INTERNAL_ERROR;                 \
        }                                             \
    }

#define AP_CHECK_CU_ERROR(err)                      \
    {                                               \
        CUresult pst = err;                         \
        if (CUDA_SUCCESS != pst) {                  \
            PRINT_SENSEDRIVE_CU_ERROR_MESSAGE(pst); \
            return AP_INTERNAL_ERROR;               \
        }                                           \
    }
#endif  // USE_CUDA

#define AP_ASSERTION_TRUE_WITHOUT_MESSAGE(res, err)                         \
    {                                                                       \
        apStatus_t pst = err;                                               \
        if (!res) {                                                         \
            PRINT_SENSEDRIVE_ERROR_MESSAGE(SenseDriveGetStatusString(pst)); \
            return pst;                                                     \
        }                                                                   \
    }

#define AP_ASSERTION_TRUE_WITH_MESSAGE(res, err, msg)                 \
    {                                                                 \
        apStatus_t pst = err;                                         \
        if (!res) {                                                   \
            LOGP_ERROR("{} {}", SenseDriveGetStatusString(pst), msg); \
            return pst;                                               \
        }                                                             \
    }

#define AP_ASSERTION_TRUE(res, err, ...)                                \
    __GET_ARG4(res, err, ##__VA_ARGS__, AP_ASSERTION_TRUE_WITH_MESSAGE, \
               AP_ASSERTION_TRUE_WITHOUT_MESSAGE)                       \
    (res, err, ##__VA_ARGS__)

}  // namespace perception
}  // namespace senseAD
