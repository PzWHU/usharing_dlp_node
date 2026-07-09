/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * WangShaochang <wangshaochang@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ChassisState
 */
typedef struct {
    uint64_t timestamp;        // unit: ns
    sapVector2f position;      // unit, m; coord, ENU
    float speed;               // unit, m/s
    sapVector2f acceleration;  // unit, m/s2, coord ENU
    double heading;   // unit, rad; it is calculated based on ENU-x axis,
                      // and positive when rotated by counter-clockwise
                      // direction
    double yaw_rate;  // unit, rad/s
    double steering_wheel_angle;  // unit, rad
    double steering_torque;       // add the driver hand torque
    bool ebbbrkpedalapplied;      // detect whether brake flag
    uint64_t gear;
    bool lateral_engaged;  // auto driving system control in lateral
    bool longi_engaged;    // auto driving system control in longi
    bool throttle_override;
    bool steer_torque_exitauto_flag;  // true: exitautodrive

    // common_utils::FixedQueue<bool, 5>
    //     longi_engaged_queue;  // auto driving system control in longi

    bool lcc_engage;
    bool lcc_exit;

    bool btn_acc_onoff;
    bool btn_acc_cancel;
    bool btn_acc_resume;
    // headway set and mns/pls won't be acitvated at the same time
    bool btn_headway_set;
    bool btn_headway_pls;
    bool btn_headway_mns;
    bool btn_acc_speed_pls;
    bool btn_acc_speed_mns;
    bool btn_lcc_onoff;

    bool no_impl_turn_signal;
    uint64_t pre_msg_turn_signal;

    // pedal info
    float throttle_percent_command;
    float throttle_percent_actual;
    float brake_percent_command;
    float brake_percent_actual;

    // adas btn info
    bool ACC_btn;
    bool AEB_btn;
    bool LDP_btn;
    bool TJA_btn;
    bool LDW_btn;
    bool BSD_btn;
    bool LCA_btn;
    bool OSE_btn;
    bool FCTA_btn;
    bool RCTA_btn;
    bool ALC_btn;

    // door && trunk && hood
    bool DoorFL;
    bool DoorFR;
    bool DoorRL;
    bool DoorRR;
    bool DoorTrunk;
    bool DoorHood;
    // turn signal
    uint64_t turn_signal;

    double fornt_wheel_ang;  // frontWheelAng
    float set_speed;
    unsigned char set_time_gap;
    float steer_wheel_angle_rate;
    bool abs_active;
} sapChassisState;

#ifdef __cplusplus
}
#endif
