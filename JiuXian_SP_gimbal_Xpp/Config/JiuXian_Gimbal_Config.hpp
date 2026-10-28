//
// Created by ASUS on 2026/7/14.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_JIUXIAN_GIMBAL_CONFIG_HPP
#define JIUXIAN_SP_GIMBAL_XPP_JIUXIAN_GIMBAL_CONFIG_HPP

#include "JiuXuan_Status.hpp"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define ANGLE_TRANS(x, init)                    (x + init > 360.0f ? x + init - 360.0f : (x + init < 0.0f ? x + init + 360.0f : x + init))
#define ANGLE_TRANS_INV(x, init)                (x - init > 360.0f ? x - init - 360.0f : (x - init < 0.0f ? x - init + 360.0f : x - init))
#define ANGLE_TRANS_Absolute_to_Signed(x)       (x > 180.0f ? x - 360.0f : (x < -180.0f ? x + 360.0f : x))

#define friction_pulley_l(vel)   ( vel)
#define friction_pulley_r(vel)   (-vel)
#define pluck_vel_set(vel, dir)  (dir == 1? vel : (dir == -1 ? -vel : 0.0f))

#define CMD_MAX_VEL      2    // (m/s)
#define CMD_MAX_OMEGA    6.28 // (rad/s)

#define DATA_RESERVE_u8  0x00
#define DATA_RESERVE_f   0.0f

/* ============================================================================== */
#define LITTLE_YAW_INIT_ANGLE              166.244141f
#define PITCH_INIT_ANGLE                   179.511328f

#define LITTLE_YAW_WORKSPACE_RANGE          90.000000f
#define PITCH_WORKSPACE_RANGE               40.000000f

#define LITTLE_YAW_ANGLE_MIN      (-1.0f * LITTLE_YAW_WORKSPACE_RANGE / 2.0f)
#define LITTLE_YAW_ANGLE_MAX      ( 1.0f * LITTLE_YAW_WORKSPACE_RANGE / 2.0f)
#define PITCH_ANGLE_MIN           (-1.0f * PITCH_WORKSPACE_RANGE      / 2.0f)   //205
#define PITCH_ANGLE_MAX           ( 1.0f * PITCH_WORKSPACE_RANGE      / 2.0f)   //237
/* ============================================================================== */

typedef struct __attribute__((packed)){
    float    cmd_vel_x;
    float    cmd_vel_y;
    float    cmd_omega;
    float    set_big_yaw_controller_cmd;
    float    gimbal_yaw;
}tx_chassis_cmd_t;

typedef struct __attribute__((packed)){
    float    fdb_vel_x;
    float    fdb_vel_y;
    float    fdb_omega;
    float    big_yaw_angle;
    float    big_yaw_speed;
    float    big_yaw_current;
    float    big_yaw_init_abs_angle;
    float    relevant_yaw_angle;
}rx_chassis_cmd_t;

typedef struct __attribute__((packed)){
    tx_chassis_cmd_t tx_cmd;
    rx_chassis_cmd_t rx_cmd;

    float vel_x_cmd;
    float vel_y_cmd;
    float omega_cmd;

    uint8_t tx_cmd_msg[64];
    uint8_t rx_cmd_msg[64];
}Sentry_Chassis_Gimbal_Interact_Info_t;;

typedef struct __attribute__((packed)){
    JiuXuan_Menu_Info_t JiuXuan_Menu_Info;

    bool                IsSentryAttack;
    bool                IsAlive;
}JiuXuan_Status_Info_t;

typedef struct {
    uint64_t timestamp;
    bool     isTimeout;
}JiuXuan_Community_Tameout_Tick_t;

/* ================================================================================================ */
void JiuXuan_Menu_Selector(int16_t SA, int16_t SB, int16_t SC, int16_t SD, int16_t SE, bool SF, int16_t SG, bool SH);

#endif //JIUXIAN_SP_GIMBAL_XPP_JIUXIAN_GIMBAL_CONFIG_HPP