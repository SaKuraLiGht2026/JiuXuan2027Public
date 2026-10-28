//
// Created by ASUS on 2026/7/19.
//
#include "../Inc/JiuXian_Config.hpp"
#include "../../Algorithm/S_Curve/Inc/alg_s_curve.hpp"
#include "../../Algorithm/Ramp_Planner/Ramp.hpp"
#include "../../Bsp/RS485/Inc/bsp_rs485.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/HIPNUC/Inc/drv_HIPNUC_Hl12S2.h"
#include <arm_math.h>

/* System init state */
bool IsChassisSystemInit = false;

/* System and node tick */
uint64_t JiuXuan_System_Tick   = 0;
uint64_t imu_isr_Tick          = 0;
uint64_t can2_timeout_Tick     = 0;
uint16_t Gx_Tick               = 0;
uint64_t Sign_Tick             = 0; // TIM3

uint64_t ctrl_node_tick        = 0;
uint64_t debug_node_tick       = 0;
uint64_t imu_node_tick         = 0;
uint64_t init_node_tick        = 0;
uint64_t interaction_node_tick = 0;

/* Thread Frequency */
uint64_t ctrl_node_freq        = 0;
uint64_t debug_node_freq       = 0;
uint64_t imu_node_freq         = 0;
uint64_t init_node_freq        = 0;
uint64_t interaction_node_freq = 0;

/* FDCAN INIT FLAG */
bool ALL_FDCAN_INIT_OK = false;

/* FDCAN TX Flag */
bool isFDCAN1MSGTXOK   = false;
bool isFDCAN2MSGTXOK   = false;
bool isFDCAN3MSGTXOK   = false;

/* FDCAN RX Flag */
bool isFDCAN1FIFO0RXOK = false;
bool isFDCAN2FIFO0RXOK = false;
bool isFDCAN3FIFO0RXOK = false;

/* UART RX Flag */
bool isUASRT3DMARXOK   = false;
bool isUART7DMATXOK    = false;
bool isUART7DMARXOK    = false;
bool isUSART10DMARXOK  = false;

float Wheel_speed_radps[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float Steer_angle[4]       = {0.0f, 0.0f, 0.0f, 0.0f};

float big_yaw_current      = 0.0f;

float chassis_imu_init_angle_yaw = 0.0f;

Chassis_Config JiuXuan_Chassis(JiuXian_Length, JiuXian_Width);

SCurvePlanner  JiuXuan_Chassis_Speed_Curve(2.5f, 12.0f, 0.005f);

Ramp_Planner   JiuXuan_Chassis_Speed_x_Ramp(1.5f, 0.001f);
Ramp_Planner   JiuXuan_Chassis_Speed_y_Ramp(1.5f, 0.001f);
Ramp_Planner   JiuXuan_Chassis_Omega_Ramp(1.5f, 0.001f);

/* UART RX BUFFER */
__attribute__((section(".dma_buffer"))) uint8_t usart3_uart_txbuffer[256] = {0x00};
__attribute__((section(".dma_buffer"))) uint8_t uart8_uart_test_buffer[256] = {0x00};
__attribute__((section(".dma_buffer"))) uint8_t uart9_uart_test_buffer[256] = {0x00};
__attribute__((section(".dma_buffer"))) uint8_t uart10_rx_test_buffer[256]  = {0x00};

__attribute__((section(".dma_buffer"))) Struct_RS485_Manage_Object USART3_RS485_Manage_Object;

/* UART10 Double Buffer Private variables */
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   USART3_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   UART7_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   UART10_Manage_Object;

/* HIPNUC INFO */
HIPNUC_data_info_t HIPNUC_Hl12S2;

/*  */
Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;

/* Chassis Vel Calc */
/**
 * @brief 底盘解算
 */
Chassis_Config::Chassis_Config(float long_side, float short_side) :     long_side_(long_side),
                                                                        short_side_(short_side),
                                                                        v_set_x_(0.0f),
                                                                        v_set_y_(0.0f),
                                                                        omega_set_(0.0f)
{
    this->l_     = 0.5f * sqrtf(this->long_side_ * this->long_side_ + this->short_side_ * this->short_side_);

    this->alpha_ = atan2f(long_side_, short_side_);
    this->beta_  = M_PI_2 - this->alpha_;
}

void Chassis_Config::Clear() {
    this->v_set_x_   = 0.0f;
    this->v_set_y_   = 0.0f;
    this->omega_set_ = 0.0f;

    this->v_fdb_x_   = 0.0f;
    this->v_fdb_y_   = 0.0f;
    this->omega_fdb_ = 0.0f;
}

bool Chassis_Config::Set_V_x(float V_x) {
    this->v_set_x_ = V_x;
    if (fabs(V_x) < 0.01) this->v_set_x_ = 0.0f;
    return true;
}

bool Chassis_Config::Set_V_y(float V_y) {
    this->v_set_y_ = V_y;
    if (fabs(V_y) < 0.01) this->v_set_y_ = 0.0f;
    return true;
}

bool Chassis_Config::Set_Omega(float Omega) {
    this->omega_set_ = Omega;
    return true;
}

/* 底盘正解算 */
bool Chassis_Config::Chassis_Kinematics_Calculation() {
    this->v_x_ref_[0] = 1.0f * this->v_set_x_ + 0.0f * this->v_set_y_ + this->l_ * cosf(this->alpha_) * this->omega_set_;
    this->v_y_ref_[0] = 0.0f * this->v_set_x_ + 1.0f * this->v_set_y_ + this->l_ * sinf(this->alpha_) * this->omega_set_;
    this->v_x_ref_[1] = 1.0f * this->v_set_x_ + 0.0f * this->v_set_y_ - this->l_ * cosf(this->alpha_) * this->omega_set_;
    this->v_y_ref_[1] = 0.0f * this->v_set_x_ + 1.0f * this->v_set_y_ + this->l_ * sinf(this->alpha_) * this->omega_set_;
    this->v_x_ref_[2] = 1.0f * this->v_set_x_ + 0.0f * this->v_set_y_ - this->l_ * cosf(this->alpha_) * this->omega_set_;
    this->v_y_ref_[2] = 0.0f * this->v_set_x_ + 1.0f * this->v_set_y_ - this->l_ * sinf(this->alpha_) * this->omega_set_;
    this->v_x_ref_[3] = 1.0f * this->v_set_x_ + 0.0f * this->v_set_y_ + this->l_ * cosf(this->alpha_) * this->omega_set_;
    this->v_y_ref_[3] = 0.0f * this->v_set_x_ + 1.0f * this->v_set_y_ - this->l_ * sinf(this->alpha_) * this->omega_set_;

    this->wheel_speed_ref_[0] = sqrtf(this->v_x_ref_[0] * this->v_x_ref_[0] + this->v_y_ref_[0] * this->v_y_ref_[0]);
    this->wheel_speed_ref_[1] = sqrtf(this->v_x_ref_[1] * this->v_x_ref_[1] + this->v_y_ref_[1] * this->v_y_ref_[1]);
    this->wheel_speed_ref_[2] = sqrtf(this->v_x_ref_[2] * this->v_x_ref_[2] + this->v_y_ref_[2] * this->v_y_ref_[2]);
    this->wheel_speed_ref_[3] = sqrtf(this->v_x_ref_[3] * this->v_x_ref_[3] + this->v_y_ref_[3] * this->v_y_ref_[3]);

    this->steer_angle_ref_[0] = atan2f(this->v_y_ref_[0], this->v_x_ref_[0]);
    this->steer_angle_ref_[1] = atan2f(this->v_y_ref_[1], this->v_x_ref_[1]);
    this->steer_angle_ref_[2] = atan2f(this->v_y_ref_[2], this->v_x_ref_[2]);
    this->steer_angle_ref_[3] = atan2f(this->v_y_ref_[3], this->v_x_ref_[3]);

    if (this->v_x_ref_[0] == 0.0f && this->v_y_ref_[0] == 0.0f) this->steer_angle_ref_[0] = 0.0f;
    if (this->v_x_ref_[1] == 0.0f && this->v_y_ref_[1] == 0.0f) this->steer_angle_ref_[1] = 0.0f;
    if (this->v_x_ref_[2] == 0.0f && this->v_y_ref_[2] == 0.0f) this->steer_angle_ref_[2] = 0.0f;
    if (this->v_x_ref_[3] == 0.0f && this->v_y_ref_[3] == 0.0f) this->steer_angle_ref_[3] = 0.0f;

    return true;
}
/* ================================================================================================================================== */

bool Chassis_Config::Get_Steer_Angle(float ang1, float ang2, float ang3, float ang4) {
    this->steer_angle_fdb_[0] = ang1;
    this->steer_angle_fdb_[1] = ang2;
    this->steer_angle_fdb_[2] = ang3;
    this->steer_angle_fdb_[3] = ang4;

    return true;
}

bool Chassis_Config::Get_Wheel_Speed(float spd1, float spd2, float spd3, float spd4) {
    this->wheel_speed_fdb_[0] = spd1 * JiuXuan_Wheel_R;
    this->wheel_speed_fdb_[1] = spd2 * JiuXuan_Wheel_R;
    this->wheel_speed_fdb_[2] = spd3 * JiuXuan_Wheel_R;
    this->wheel_speed_fdb_[3] = spd4 * JiuXuan_Wheel_R;

    return true;
}

/* 底盘逆解算 */
/* 底盘逆解算（根据正解算严格修正） */
bool Chassis_Config::Chassis_Inverse_Kinematics_Calculation() {
    // 1. 将每个轮子的速度和角度转换为底盘坐标系下的速度分量（保留原有硬件接口宏）
    this->v_x_fdb_[0] = WHEEL_TRANS(this->wheel_speed_fdb_[0], 1) * cosf(STEER_TRANS_INV(this->steer_angle_fdb_[0], STEER_ID1_INIT) * PI / 180.0f);
    this->v_y_fdb_[0] = WHEEL_TRANS(this->wheel_speed_fdb_[0], 1) * sinf(STEER_TRANS_INV(this->steer_angle_fdb_[0], STEER_ID1_INIT) * PI / 180.0f);

    this->v_x_fdb_[1] = WHEEL_TRANS(this->wheel_speed_fdb_[1], 2) * cosf(STEER_TRANS_INV(this->steer_angle_fdb_[1], STEER_ID2_INIT) * PI / 180.0f);
    this->v_y_fdb_[1] = WHEEL_TRANS(this->wheel_speed_fdb_[1], 2) * sinf(STEER_TRANS_INV(this->steer_angle_fdb_[1], STEER_ID2_INIT) * PI / 180.0f);

    this->v_x_fdb_[2] = WHEEL_TRANS(this->wheel_speed_fdb_[2], 3) * cosf(STEER_TRANS_INV(this->steer_angle_fdb_[2], STEER_ID3_INIT) * PI / 180.0f);
    this->v_y_fdb_[2] = WHEEL_TRANS(this->wheel_speed_fdb_[2], 3) * sinf(STEER_TRANS_INV(this->steer_angle_fdb_[2], STEER_ID3_INIT) * PI / 180.0f);

    this->v_x_fdb_[3] = WHEEL_TRANS(this->wheel_speed_fdb_[3], 4) * cosf(STEER_TRANS_INV(this->steer_angle_fdb_[3], STEER_ID4_INIT) * PI / 180.0f);
    this->v_y_fdb_[3] = WHEEL_TRANS(this->wheel_speed_fdb_[3], 4) * sinf(STEER_TRANS_INV(this->steer_angle_fdb_[3], STEER_ID4_INIT) * PI / 180.0f);

    // 2. 底盘速度正逆解匹配：直接平均法（与正解算 vx_i = vx + ... 严格对应）
    this->v_fdb_x_ = 0.25f * (this->v_x_fdb_[0] + this->v_x_fdb_[1] + this->v_x_fdb_[2] + this->v_x_fdb_[3]);
    this->v_fdb_y_ = 0.25f * (this->v_y_fdb_[0] + this->v_y_fdb_[1] + this->v_y_fdb_[2] + this->v_y_fdb_[3]);

    // 3. 角速度逆解算：根据正解算矩阵求逆推导
    // omega = [cos(a)*(vx0 - vx1 + vx3 - vx2) + sin(a)*(vy0 + vy1 - vy2 - vy3)] / (4*l)
    this->omega_fdb_ = ( cosf(this->alpha_) * (this->v_x_fdb_[0] - this->v_x_fdb_[1] + this->v_x_fdb_[3] - this->v_x_fdb_[2])
                       + sinf(this->alpha_) * (this->v_y_fdb_[0] + this->v_y_fdb_[1] - this->v_y_fdb_[2] - this->v_y_fdb_[3]) )
                     / (4.0f * this->l_);

    return true;
}

