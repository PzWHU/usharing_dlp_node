/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>
#include "perception_camera/sap_macros.h"
#include "perception_camera/sap_radar_define.h"
#include "perception_camera/sap_image_info.h"
#include "perception_camera/data_type/sdmap/sdmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 相机pipeline操作句柄
 */
typedef void* sapCameraHandle;

#define MAX_PATH 256
#define MAX_NAME 128
#define MAX_PIPELINE_NUM 20
#define MAX_CAMERA_NUM 11
#define MAX_LIDAR_NUM 3
#define MAX_OP_TIME_COST_NUM 100

/**
 * @brief 相机参数
 * 相机的参数配置，配置了相机标定参数路径、相机名称，对应的算法Pipeline的名称、使用的设备等信息
 */
typedef struct {
    /** 系统运行根目录 */
    char senseauto_root_path[MAX_PATH];
    /** 相机标定配置目录 */
    char sensor_config_folder[MAX_PATH];
    /** 相机配置根目录 */
    char sensor_config_root_path[MAX_PATH];
    /** 算法Pipeline配置文件 */
    char pipeline_files[MAX_PIPELINE_NUM][MAX_PATH];
    /** 发布模块名称*/
    char pub_module_names[MAX_PIPELINE_NUM][MAX_NAME];
    /** 推理用的设备ID, 与设备平台相关，有些平台不用绑定设备，可忽略 */
    int pipeline_infer_dev_id;
    /** Pipeline数量 */
    int pipeline_num;
    /** 发布模块数量*/
    int pub_module_num;
    /** 相机名称 */
    char camera_names[MAX_CAMERA_NUM][MAX_NAME];
    double cameras_fps[MAX_CAMERA_NUM];
    /** 相机数量 */
    int camera_num;

    // control param
    int opencv_thread_num;
    int8_t is_single_thread;
    int pipeline_infer_ai_core_group_id[MAX_PIPELINE_NUM];
    /**
    * 图片处理通道号，
    * 通道号取值范围：[0, 255]，通道总数最多256。
    * 用户在使用VPC功能或VDEC功能（内部会使用VPC功能）时，都会占用VPC通道号
    * 通道号不能冲突
    */
    int vpc_channel;

    /** lidar名称 */
    char lidar_names[MAX_LIDAR_NUM][MAX_NAME];
    double lidars_fps[MAX_LIDAR_NUM];
    /** lidar数量 */
    int lidar_num;
    int8_t is_replay_mode;  // 0: normal, 1: replay
} sapCameraParam;

/**
 * @brief 输入的图像匹配情况
 *
 */
typedef struct {
    char module_names[MAX_PIPELINE_NUM][MAX_NAME];
    int module_num;
} sapModuleMatchStatus;

/**
 * @brief 输入的图像帧
 *
 */
typedef struct {
    /** 图像时间戳 */
    uint64_t timestamp_ns;
    /** 帧号，自定义，无绝对意义，一般是顺序号 */
    int64_t frame_no;
    /** 相机编号 */
    int8_t sensor_id;
    /** 相机名称 */
    char sensor_name[MAX_NAME];
    /** 相机图像数据 */
    sapRawImage image;
    /** 用户自定义地址，release_image 回调透传*/
    void* user_data;
} sapCameraFrame;

/**
 * @brief 初始化图形数据结构体
 * @param[in] image 图形数据结构体指针
 * @return 成功返回0, 否则返回错误码
 */
SAP_API int32_t initSapRawImage(sapRawImage* image);

/**
 * @brief 初始化sapCameraFrame结构体数据
 * @param[in] frame sapCameraFrame类型结构体指针
 * @return 成功返回0, 否则返回错误码
 */
SAP_API int32_t initSapCameraFrame(sapCameraFrame* frame);

/**
 * @brief 透传用户自定义数据
*/
typedef struct {
    /** 相机名称 */
    char sensor_name[MAX_NAME];
    /** 用户自定义数据，回调透传*/
    void* user_data;
} sapUserDataParam;

/**
 * @brief 内部结果信息，包括耗时等
 *
 */
typedef struct {
    /** Pipeline名称 */
    char* module_name;
    /** 帧号 */
    int64_t frame_no;
    /** Pipeline处理耗时 */
    double time_cost;
    /** Pipeline OP处理耗时,当前只提供推理OP耗时 */
    double op_time_cost_ms_arr[MAX_OP_TIME_COST_NUM];
    const char* op_name_arr[MAX_OP_TIME_COST_NUM];
    int valid_op_num;
    /** swc user data */
    void* inner_frame;
    int user_data_num;
    sapUserDataParam user_data_param[MAX_CAMERA_NUM];
    /** is_pub_result=0 means no perception results are published; a value
     * other than 0 means perception results are published.*/
    int8_t is_pub_result;
} sapPublishResultParam;

/**
 * @brief Pipeline执行状态
 *
 */
typedef enum { OP_FAILED = -1, OP_SUCCESS = 0 } sapEventStatus;

typedef enum {
    /* 读取前视FOV120相机标定参数 */
    CALIBRATION_PARAM_FOV120_READ_OP,
    /* 读取前视FOV30相机标定参数 */
    CALIBRATION_PARAM_FOV30_READ_OP,
    /* 读取左前相机标定外参参数 */
    CALIBRATION_PARAM_LF_CAM_READ_OP,
    /* 右到右前相机标定外参参数 */
    CALIBRATION_PARAM_RF_CAM_READ_OP,
    /* 读取左后相机标定外参参数 */
    CALIBRATION_PARAM_LR_CAM_READ_OP,
    /* 读取右后相机标定外参参数 */
    CALIBRATION_PARAM_RR_CAM_READ_OP,
    /* 读取后视相机标定外参参数 */
    CALIBRATION_PARAM_REAR_CAM_READ_OP,
    /* 多相机图像曝光时间戳匹配 */
    MULTI_CAM_TIMESTAMP_MATCH_OP,
    PIPELINE_RUNTIME_OP,
    PIPELINE_STUCK,
    OP_RUNTIME_ERROR,
    OP_INIT_ERROR,
    OP_RUN_TIMEOUT,
    OP_GET_GPU_LOCK_TIMEOUT,
    READ_NPY_END,
    OP_CODE_NUM
} sapOPCode;

typedef struct {
    sapOPCode op_code;
    sapEventStatus err_code;
    const char* module_name;
    uint64_t timestamp_ns;
} sapPipelineStatus;

/**
 * @brief 感知处理完成回调通知
 */
typedef struct {
    /** 通知一帧数据处理完成，可删除输入的图像数据 */
    int (*release_image)(const uint8_t* const image, void* user_data);
    /** 异步通知感知处理完成，可调用获取结果接口获取感知结果 */
    int (*publish_result)(const sapPublishResultParam* const result_param);
    /** Pipeline执行状态的回调 */
    int (*notify_pipeline_status)(
        const sapPipelineStatus* const pipeline_status);
    /** 通知一帧数据处理完成，可删除输入的lidar数据 */
    int (*release_lidar_frame)(const uint8_t* const lidar_data_ptr,
                               void* user_data);
    /** 通知一帧SDMap数据处理完成，可删除输入的SDMap数据*/
    int (*release_sdmap_data)(const sapSDMap* const sdmap_data_ptr);
} sapCallBackFunSet;

typedef enum sapSensorStatus_t {
    SENSOR_INVALID = 0,
    SENSOR_NORMAL = 1,
    SENSOR_OTHERS = 2
} sapSensorStatus;

typedef struct sapCameraMetaData_t {
    sapSensorStatus sensor_status;
    double sensor_to_ego_pose[4][4];
    uint16_t sensor_id;
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    uint64_t perception_capture_timestamp_ns;
    uint8_t is_used;
} sapCameraMetaData;

#ifdef __cplusplus
}
#endif
