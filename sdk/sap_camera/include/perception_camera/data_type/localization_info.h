/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdint.h>
#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3D点坐标
 *
 */
typedef struct sapPoint3D_t {
    /** 以原点正东为x方向的x坐标值，单位米 */
    double x;
    /** 以原点正北为Y方向的y坐标值，单位米 */
    double y;
    /** WGS-84坐标系椭球高度，单位米 */
    double z;  // Up from the WGS-84 ellipsoid, in meters.
} sapPoint3D_t;

/**
 * @brief 四元素?
 */
typedef struct Quaternion_t {
    /** 四元素虚部向量x分量，对应旋转轴描述 */
    double qx;
    /** 四元素虚部向量y分量，对应旋转轴描述 */
    double qy;
    /** 四元素虚部向量z分量，对应旋转轴描述 */
    double qz;
    /** 四元素实部，对应旋转角描述 */
    double qw;
} sapQuaternion_t;

/**
 * @brief 车辆位置姿态
 */
typedef struct StructSE3Pose {
    /** 当前车辆位姿的时间戳，单位纳秒 */
    uint64_t timestamp_ns;
    /** 位置坐标，单位米 */
    sapPoint3D_t position;
    /** 姿态 */
    sapQuaternion_t quat;
} sapSE3Pose;

/**
 * @brief 欧拉角表示的物体旋转角度
 */
typedef struct StructEulerAngles_t {
    /** 翻滚角，目标绕y轴转动角度，弧度单位 */
    double roll;
    /** 俯仰角，目标绕x轴转动角度，弧度单位 */
    double pitch;
    /** 偏航角，目标绕z轴转动角度，弧度单位 */
    double yaw;
} sapEulerAngles_t;

/**
 * @brief 局部定位信息
 * 载体为车体中心坐标系表示为前左上，局部坐标系为odometry参考坐标系（定义为第一帧车体中心坐标系）
 */
typedef struct StructLocalLocalizationInfo {
    /** 测量时间戳，单位纳秒 */
    uint64_t measurement_time_ns;
    /** 精度等级：0-无效，1-低精度，2-中、高精度 */
    uint32_t accuracy_level;
    /** 局部坐标系下的自车位姿，位置信息单位为m，姿态信息用四元素来表达 */
    sapSE3Pose local_pose;
    /** 角速度信息，表示在车体中心坐标系下，单位rad/s */
    sapPoint3D_t ego_car_angular_velocity;
    /** 线速度信息，表示在车体中心坐标系下，单位m/s */
    sapPoint3D_t ego_car_linear_velocity;
    /** 线加速度信息，表示在车体中心坐标系下，单位m/s^2 */
    sapPoint3D_t ego_car_linear_acceleration;
    /** 自车相对于地面旋转角四元素 */
    Quaternion_t ego_car_to_ground_rotation;
    /** 位置协方差，表示在车体中心坐标系下 */
    sapPoint3D_t local_translation_covariance;
    /** 姿态协方差，表示在车体中心坐标系 */
    sapPoint3D_t local_rotation_covariance;
    /** 位置标准差，表示在车体中心坐标系下，单位m */
    sapPoint3D_t position_std;
    /** 姿态信息，表示在局部坐标系下，单位弧度 */
    sapEulerAngles_t euler_angle;
    /** 姿态标准差，单位弧度，局部坐标系下欧拉角标准差 */
    sapPoint3D_t attitude_std;
} sapLocalLocalizationInfo;

/**
 * @brief 初始化局部定义信息结构体
 * @param[in] info 局部定位信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapLocalLocalizationInfo(sapLocalLocalizationInfo* info);

#ifdef __cplusplus
}
#endif
