//
// Created by ASUS on 2026/7/14.
//
#include "JiuXian_Gimbal_Config.hpp"
#include "JiuXuan_Status.hpp"
#include "../../Algorithm/MathUnit/Inc/MathUnit.hpp"
#include "../../Algorithm/S_Curve/Inc/alg_s_curve.hpp"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/MiniPC/Inc/Drv_MiniPC.h"
#include "../../Drv/HIPNUC/Hl14S3/Inc/drv_HIPNUC_Hl14S3.h"
#include "../../PreApp/Signal_Generator/Inc/Signal_Generator.hpp"
#include "../../Algorithm/Ramp_Planner/Ramp.hpp"

bool IsUseRMRefereeSystem = false;

bool IsInit            = false;
bool isFDCAN1FIFO0RXOK = false;
bool isFDCAN2FIFO0RXOK = false;
bool isFDCAN3FIFO0RXOK = false;

bool isUART7DMATXOK    = false;
bool isUART9DMATXOK    = false;

bool isPluckPIDCalcOK  = false;

bool isPluckErr        = false;

/* UART10 Double Buffer Private variables */
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object  USART1_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   UART5_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   UART8_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object   UART9_Manage_Object;
__attribute__((section(".dma_buffer"))) Drv_UART_Manage_Object USART10_Manage_Object;

/* MINIPC INFO */
__attribute__((section(".dma_buffer"))) MiniPC_Msg_Info_t        minipc_msg_info;

/* HIPNUC INFO */
HIPNUC_data_info_t                                               HIPNUC_Hl14S3;

/* Gimbal IMU Calibration Flag */
bool isPitchInfoRXOK           = false;
bool isLittleYawInfoRXOK       = false;
bool isBigYawInfoRXOK          = false;
bool isIMUInfoRXOK             = false;

bool GimbalIMUCalibrationStart = false;
bool isGimbalIMUCalibrationOK  = false;

float gimbal_imu_init_yaw      = 0.0f;
float gimbal_imu_init_pitch    = 0.0f;
float gimbal_imu_init_roll     = 0.0f;
/* =========================== */

uint64_t JiuXuan_System_Tick     = 0;
uint64_t Px_Tick                 = 0;
uint64_t Pluck_Err_Tick          = 0;
uint64_t Pluck_Recover_Tick      = 0;
uint64_t Standard_link_node_tick = 0;

uint64_t Standard_link_node_freq = 0;

/* JiuXian float Value begin */
float set_Pluck_speed    = 0.0f;
float set_Pluck_angle    = 0.0f;
float set_friction_speed = 0.0f;

float pitch_angle      = 0.0f;
float yaw_angle        = 0.0f;
float little_yaw_angle = 0.0f;
float big_yaw_angle    = 0.0f;

float gimbal_navy_yaw  = 0.0f;

float set_yaw_speed    = 0.0f;
/* Jiuxian float Value end */

JiuXuan_Status_Info_t                 JiuXuan_Status_Info;
Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;

JiuXuan_Community_Tameout_Tick_t      VTM_Timeout_Tick = {0, false};

differential                          gimbal_pitch_omega_differential(0.0f);
differential                          gimbal_yaw_omega_differential(0.0f);
differential                          gimbal_yaw_torque_differential(0.0f);

YawSimple                             JiuXuan_Yaw_Simple_Smoother(0.001, 200, 500, -4095, 4095);

YawSmoother                           JiuXuan_Yaw_Smoother(400, 1000, 3600);
Ramp_Planner                          JiuXuan_Yaw_Ramp(0.000001, 0.001);
VelocityRampPlanner                   JiuXuan_Yaw_Ramp2(1500, 2500, 0.001);

/* ================================================================================================================================================================== */
/**
 * @brief 信号发生器变量
 */
void JiuXuan_Semi_Auto_Fire_Start_Callback() {
    set_Pluck_angle += 360.0f / 7.0f;
    set_Pluck_angle  = (set_Pluck_angle > 360.0f ? set_Pluck_angle - 360.0f : set_Pluck_angle);
}

Pulse_Generator                       JiuXuan_Fire_Semi_Auto_Trigger(1000, JiuXuan_Semi_Auto_Fire_Start_Callback);

void RM_Client_Fire_Mode_Switch() {
    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientFricON) {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientFricON = false;
    }else {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientFricON = true;
    }
}

Pulse_Generator                       JiuXuan_RM_Client_Fire_Switch(10, RM_Client_Fire_Mode_Switch);

void RM_Client_Gyro_Switch() {
    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientGyroON) {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientGyroON = false;
    }else {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientGyroON = true;
    }
}

Pulse_Generator                       JiuXuan_RM_Client_Gyro_Switch(2, RM_Client_Gyro_Switch);

void RM_Client_Semi_Switch() {
    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientSemiON) {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientSemiON = false;
    }else {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientSemiON = true;
    }
}

Pulse_Generator                       JiuXuan_RM_Client_Semi_Switch(2, RM_Client_Semi_Switch);


Triangle_Generator                    JiuXuan_Sentry_Scan_Mode_Little_Yaw(
                                                                            true,                             // start_rising: 从上升开始
                                                                            3.0f / (20.0f * M_PI),            // up_gain_:   每步上升量
                                                                            3.0f / (20.0f * M_PI),            // down_gain_: 每步下降量
                                                                            LITTLE_YAW_WORKSPACE_RANGE / 2.0f,                            // top_
                                                                           -LITTLE_YAW_WORKSPACE_RANGE / 2.0f,                            // bottom_
                                                                            0.0f                              // offset
                                                                         );

Triangle_Generator                    JiuXuan_Sentry_Scan_Mode_Pitch     (
                                                                            true,                             // start_rising: 从上升开始
                                                                            0.1f,                             // up_gain_:   每步上升量
                                                                            0.1f,                             // down_gain_: 每步下降量
                                                                            PITCH_WORKSPACE_RANGE / 2.0f,                            // top_
                                                                           -PITCH_WORKSPACE_RANGE / 2.0f,                            // bottom_
                                                                            0.0f                              // offset
                                                                         );
/* ================================================================================================================================================================== */
/**
 *
 * @param SA
 * @param SB
 * @param SC
 * @param SD
 * @param SE
 * @param SF
 * @param SG
 * @param SH
 * @param RS
 */
void JiuXuan_Menu_Selector(int16_t SA,
                           int16_t SB,
                           int16_t SC,
                           int16_t SD,
                           int16_t SE,
                           bool    SF,
                           int16_t SG,
                           bool    SH
                           )
{
    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_voice      = (SE == 0 ? No_Voice : (SE == 1 ? IDLE_Voice : War_Voice));
    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_voice_mode = (SG == 0 ? Waiting  : (SG == 1 ? Soldier    : Idol     ));

    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_work_mode  = (SA == 0 ? Work_Mode_Default : (SA == 1 ? Free : War));

    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop      = (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_work_mode == Work_Mode_Default ?  Troop_Default :
                                                                                                                                                 (SB == 0 ? Troop_Default : (SB == 1 ? Infantry : Sentry)));

    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode = (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop != Infantry ? NULL_Ctrl :
                                                                                                                                           (SC == 0 ? NULL_Ctrl : (SC == 1 ? JiuXuan_Remote_Ctrl : JiuXuan_RM_Client_Ctrl)));

    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop != Troop_Default && JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode != NULL_Ctrl) {
        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode == JiuXuan_Remote_Ctrl) {
            JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch = (SF == true ? Friction_ON : Friction_OFF);
        }else {
            JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch = Friction_OFF;
        }
    }else {
        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch = Friction_OFF;
    }

    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode   = (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch == Friction_OFF ? Disable_Fire : (SD == 0    ? Disable_Fire : (SD == 1 ? Semi_Auto : Full_Auto)));
    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_switch = (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch == Friction_OFF ? Fire_OFF     : (SH == true ? Fire_ON      : Fire_OFF));
}