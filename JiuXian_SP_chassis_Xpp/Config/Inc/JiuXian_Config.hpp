//
// Created by ASUS on 2026/7/19.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_JIUXIAN_CONFIG_HPP
#define JIUXIAN_SP_CHASSIS_XPP_JIUXIAN_CONFIG_HPP

#include "main.h"
#include "math.h"
#include "stdbool.h"

#define IMU_TRANS(x, init)       (x + init > 180.0f ? x + init - 360.0f : (x + init < -180.0f ? x + init + 360.0f : x + init))

#define STEER_TRANS(x, init)     (x + init > 360.0f ? x + init - 360.0f : (x + init < 0.0f ? x + init + 360.0f : x + init))
#define STEER_TRANS_INV(x, init) (x - init > 360.0f ? x - init - 360.0f : (x - init < 0.0f ? x - init + 360.0f : x - init))

#define WHEEL_TRANS(x, id)       (id == 1 || id == 4 ? x : -x)

#define DATA_RESERVE_u8  0x00
#define DATA_RESERVE_f   0.0f

#define PID_METHOD            0
#define SMC_METHOD            1
#define USE_DMA_IDLE_RX       1
#define USE_DMA_DOUBLE_BUFFER 1
#define USE_IT_RX             0

/* =========================The Status of JiuXian======================= */
#define JiuXian_Length   0.369f
#define JiuXian_Width    0.369f

#define JiuXuan_Wheel_R  0.058f

#define STEER_ID1_INIT   242.446289f   //330.996094f
#define STEER_ID2_INIT   305.419922f
#define STEER_ID3_INIT   214.365234f
#define STEER_ID4_INIT    66.215820f

#define BIG_YAW_INIT     235.502930f

#define CMD_MAX_VEL   2    // (m/s)
#define CMD_MAX_OMEGA 62.8 // (rad/s)

/* ===================================================================== */
typedef struct __attribute__((packed)){
    float    fdb_vel_x;
    float    fdb_vel_y;
    float    fdb_omega;
    float    big_yaw_angle;
    float    big_yaw_speed;
    float    big_yaw_current;
    float    big_yaw_init_abs_angle;
    float    relevant_yaw_angle;
}tx_chassis_cmd_t;

typedef struct __attribute__((packed)){
    float    cmd_vel_x;
    float    cmd_vel_y;
    float    cmd_omega;
    float    set_big_yaw_controller_cmd;
    float    ginbal_yaw;
}rx_chassis_cmd_t;

typedef struct __attribute__((packed)){
    tx_chassis_cmd_t tx_cmd;
    rx_chassis_cmd_t rx_cmd;

    /* 大地坐标系 */
    float cmd_vel_x_;
    float cmd_vel_y_;

    /* =========================== */
    float cmd_omega_;
    /* =========================== */

    /* 底盘坐标系 */
    float chassis_vel_x_cmd;
    float chassis_vel_y_cmd;

    uint8_t tx_cmd_msg[64];
    uint8_t rx_cmd_msg[64];
}Sentry_Chassis_Gimbal_Interact_Info_t;;

/**
 * @brief 底盘配置参数
 */
class Chassis_Config {
        public:
        Chassis_Config(float long_side, float short_side);

        /**
         * @brief 设置底盘期望速度X
         * @param V_x
         * @return
         */
        bool Set_V_x(float V_x);

        /**
         * @brief 设置底盘期望速度Y
         * @param V_y
         * @return
         */
        bool Set_V_y(float V_y);

        /**
         * @brief 设置底盘期望角速度
         * @param Omega
         * @return
         */
        bool Set_Omega(float Omega);

        [[nodiscard]] float Get_V_x_Cmd() const {return this->v_set_x_;}
        [[nodiscard]] float Get_V_y_Cmd() const {return this->v_set_y_;}
        [[nodiscard]] float Get_Omega_Cmd() const {return this->omega_set_;}

        /**
         * @brief 底盘正解算
         * @return
         */
        bool Chassis_Kinematics_Calculation();

        /**
         * @brief 获取底盘轮毂电机速度
         * @param spd1
         * @param spd2
         * @param spd3
         * @param spd4
         * @return
         */
        bool Get_Wheel_Speed(float spd1, float spd2, float spd3, float spd4);

        /**
         * @brief 获得底盘舵向电机角度
         * @param ang1
         * @param ang2
         * @param ang3
         * @param ang4
         * @return
         */
        bool Get_Steer_Angle(float ang1, float ang2, float ang3, float ang4);

        /**
         * @brief 底盘逆解算
         * @return
         */
        bool Chassis_Inverse_Kinematics_Calculation();

        /**
         * @brief 清除底盘数据
         */
        void Clear();

        float v_x_ref_[4];
        float v_y_ref_[4];

        float steer_angle_ref_[4];
        float wheel_speed_ref_[4];

        float v_fdb_x_;
        float v_fdb_y_;
        float omega_fdb_;

        private:

        float long_side_;
        float short_side_;

        float l_;
        float alpha_;
        float beta_;

        /* =============================Set========================== */

        float v_set_x_;
        float v_set_y_;
        float omega_set_;

        /* =============================Fdb========================== */

        float v_x_fdb_[4];
        float v_y_fdb_[4];

        float steer_angle_fdb_[4];
        float wheel_speed_fdb_[4];

    };
/* ===================================================================== */

#endif //JIUXIAN_SP_CHASSIS_XPP_JIUXIAN_CONFIG_HPP