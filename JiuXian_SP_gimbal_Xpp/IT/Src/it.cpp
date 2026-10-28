//
// Created by ASUS on 2026/7/14.
//
#include "../Inc/it.h"
#include "../../PreApp/WS2812/Inc/Indicator.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
#include "fdcan.h"
#include "usart.h"
#include "../../Drv/HIPNUC/Hl14S3/Inc/drv_HIPNUC_Hl14S3.h"
#include "../../Drv/DJI/Inc/drv_DJI.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/MiniPC/Inc/Drv_MiniPC.h"
#include "../../Algorithm/MathUnit/Inc/MathUnit.hpp"
#include "../../Algorithm/LADRC/Inc/LADRC.hpp"
#include "../../Algorithm/PID/Inc/PID.hpp"
#include "../../Algorithm/SMC/Inc/alg_smc.hpp"
#include "../../Algorithm/ComplianceControl/Inc/ComplianceControl.hpp"
#include "../../Algorithm/S_Curve/Inc/alg_s_curve.hpp"
#include "../../Algorithm/Ramp_Planner/Ramp.hpp"
#include "../../PreApp/Signal_Generator/Inc/Signal_Generator.hpp"

uint32_t TIM3_Tick = 0;

float    big_yaw_fdb_angle      = 0.0f;
float    big_yaw_fdb_angle_last = 0.0f;
float    big_yaw_fdb_angle_lpf  = 0.0f;

float    little_yaw_angle_last  = 0.0f;

extern float                    set_Pluck_speed;
extern float                    set_Pluck_angle;
extern float                    set_friction_speed;

extern float                    pitch_angle;
extern float                    yaw_angle;
extern float                    little_yaw_angle;
extern float                    big_yaw_angle;
extern float                    set_yaw_speed;

extern float                    gimbal_navy_yaw;

extern float                    gimbal_imu_init_yaw;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern Drv_UART_Manage_Object   USART1_Manage_Object;
extern Drv_UART_Manage_Object   UART5_Manage_Object;
extern Drv_UART_Manage_Object   UART8_Manage_Object;
extern Drv_UART_Manage_Object   UART9_Manage_Object;
extern Drv_UART_Manage_Object   USART10_Manage_Object;

extern DJI_Msg_INFO_t           DJI_M2006_P36_G1;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G2;
extern DJI_Msg_INFO_t           DJI_GM6020_G2;

extern HIPNUC_data_info_t       HIPNUC_Hl14S3;

extern JiuXuan_Community_Tameout_Tick_t VTM_Timeout_Tick;

extern bool                     isFDCAN1FIFO0RXOK;
extern bool                     isFDCAN2FIFO0RXOK;
extern bool                     isFDCAN3FIFO0RXOK;

extern bool                     isUART7DMATXOK;
extern bool                     isUART9DMATXOK;

extern bool                     isPluckPIDCalcOK;

extern bool                     isPluckErr;

extern bool                     IsInit;

extern bool                     isPitchInfoRXOK;
extern bool                     isLittleYawInfoRXOK;
extern bool                     isBigYawInfoRXOK;
extern bool                     isIMUInfoRXOK;

extern uint64_t                 JiuXuan_System_Tick;
extern uint64_t                 Px_Tick;
extern uint64_t                 Pluck_Err_Tick;
extern uint64_t                 Pluck_Recover_Tick;
extern uint64_t                 Standard_link_node_tick;

extern uint64_t                 Standard_link_node_freq;

/* Ammo */
extern PIDController            PluckPID;
extern PIDController            PluckSemiPID;
extern PIDController            FricLPID;
extern PIDController            FricRPID;

/* Gimbal */
extern PIDController            JiuXuan_Pitch_Pos_PID;
extern PIDController            JiuXuan_PITCH_Spd_PID;

extern SMC                      JiuXuan_Little_Yaw_Smc;
extern SMC                      JiuXuan_Big_Yaw_Smc;

extern PIDController            JiuXuan_Big_Yaw_Pos_PID;
extern PIDController            JiuXuan_Big_Yaw_Spd_PID;
extern PIDController            JiuXuan_Big_Yaw_Tor_PID;

extern LADRC                    JiuXuan_Big_Yaw_LADRC;
extern ComplianceControl        JiuXuan_Big_Yaw_Compliance;
/**
 * @brief 数学计算
 */
extern differential             gimbal_pitch_omega_differential;
extern differential             gimbal_yaw_omega_differential;
extern differential             gimbal_yaw_torque_differential;

extern YawSimple                JiuXuan_Yaw_Simple_Smoother;
extern YawSmoother              JiuXuan_Yaw_Smoother;
extern Ramp_Planner             JiuXuan_Yaw_Ramp;
extern VelocityRampPlanner      JiuXuan_Yaw_Ramp2;

/**
 * @brief 信号发生器部分
 */
extern Pulse_Generator          JiuXuan_Fire_Semi_Auto_Trigger;
extern Pulse_Generator          JiuXuan_RM_Client_Fire_Switch;
extern Pulse_Generator          JiuXuan_RM_Client_Gyro_Switch;
extern Pulse_Generator          JiuXuan_RM_Client_Semi_Switch;

extern Triangle_Generator       JiuXuan_Sentry_Scan_Mode_Little_Yaw;
extern Triangle_Generator       JiuXuan_Sentry_Scan_Mode_Pitch;

extern DMA_HandleTypeDef        hdma_usart1_rx;
extern DMA_HandleTypeDef        hdma_uart5_rx;
extern DMA_HandleTypeDef        hdma_uart8_rx;
extern DMA_HandleTypeDef        hdma_uart9_rx;
extern DMA_HandleTypeDef        hdma_usart10_rx;

extern JiuXuan_Status_Info_t    JiuXuan_Status_Info;

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;

/**
 * @brief :
 */
void TIM1_Period_Callback(void) {
    JiuXuan_System_Tick++;

    USART1_Manage_Object.Rx_Timeout_Count++;
    if (USART1_Manage_Object.Rx_Timeout_Count > 1000) {
        USART1_Manage_Object.Rx_Timeout_Count = 1000;
        USART1_Manage_Object.UART_Timeout = true;
    }

    UART5_Manage_Object.Rx_Timeout_Count++;
    if (UART5_Manage_Object.Rx_Timeout_Count > 1200) {
        UART5_Manage_Object.Rx_Timeout_Count = 1200;
        UART5_Manage_Object.UART_Timeout = true;
    }

    USART10_Manage_Object.Rx_Timeout_Count++;
    if (USART10_Manage_Object.Rx_Timeout_Count > 1000) {
        USART10_Manage_Object.Rx_Timeout_Count = 1000;
        USART10_Manage_Object.UART_Timeout = true;
    }

    VTM_Timeout_Tick.timestamp++;
    if (VTM_Timeout_Tick.timestamp > 2000) {
        VTM_Timeout_Tick.isTimeout = true;
    }

    if (IsInit) {
        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode == Full_Auto) {
            set_Pluck_angle = DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos;
            if (PluckPID.update(set_Pluck_speed, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel)) {
                isPluckPIDCalcOK = true;
            }
        }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode == Semi_Auto){
            if (PluckSemiPID.update(set_Pluck_angle, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos)) {
                if (PluckPID.update(PluckSemiPID.getOutput(), DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel)) {
                    isPluckPIDCalcOK = true;
                }
            }
        }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode == JiuXuan_RM_Client_Ctrl) {
            if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientSemiON) {
                if (PluckSemiPID.update(set_Pluck_angle, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos)) {
                    if (PluckPID.update(PluckSemiPID.getOutput(), DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel)) {
                        isPluckPIDCalcOK = true;
                    }
                }
            }else {
                set_Pluck_angle = DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos;
                if (PluckPID.update(set_Pluck_speed, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel)) {
                    isPluckPIDCalcOK = true;
                }
            }
        }else {
            set_Pluck_angle = DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos;
            if (PluckPID.update(0, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel)) {
                isPluckPIDCalcOK = true;
            }
        }

        JiuXuan_Yaw_Ramp2.setTarget(set_yaw_speed);
        yaw_angle        += JiuXuan_Yaw_Ramp2.update() * 0.001f;

        // JiuXuan_Yaw_Simple_Smoother.update(set_yaw_speed, ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)));

        JiuXuan_Yaw_Smoother.update(yaw_angle, 0.001f);


        yaw_angle        = (JiuXuan_Status_Info.IsSentryAttack ? yaw_angle : JiuXuan_Yaw_Smoother.getSmoothPosition());
        yaw_angle        = (yaw_angle > 360.0f ? yaw_angle - 360.0f : (yaw_angle < 0 ? yaw_angle + 360.0f : yaw_angle));

        // yaw_angle        = (JiuXuan_Yaw_Angle_SCurve.get_angle() > 360.0f ? JiuXuan_Yaw_Angle_SCurve.get_angle() - 360.0f : (JiuXuan_Yaw_Angle_SCurve.get_angle() < 0 ? JiuXuan_Yaw_Angle_SCurve.get_angle() + 360.0f : JiuXuan_Yaw_Angle_SCurve.get_angle()));

        big_yaw_angle    = yaw_angle;

        gimbal_pitch_omega_differential.differential_calc(pitch_angle, 0.001);
        gimbal_yaw_omega_differential.differential_calc(yaw_angle, 0.001);
        gimbal_yaw_torque_differential.differential_calc(gimbal_yaw_omega_differential.getOutput() * M_PI / 180.0f, 0.001);

        big_yaw_fdb_angle = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle + (Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180.0f / M_PI);
        big_yaw_fdb_angle_lpf = 0.2 * big_yaw_fdb_angle + 0.8 * big_yaw_fdb_angle_last;

        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Infantry) {
            little_yaw_angle = ANGLE_TRANS_Absolute_to_Signed(yaw_angle) - ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle));
            little_yaw_angle = (little_yaw_angle > 180.0f ? little_yaw_angle - 360.0f : (little_yaw_angle < -180.0f ? little_yaw_angle + 360.0f : little_yaw_angle));
            little_yaw_angle = (little_yaw_angle > LITTLE_YAW_ANGLE_MAX ? LITTLE_YAW_ANGLE_MAX : (little_yaw_angle < LITTLE_YAW_ANGLE_MIN ? LITTLE_YAW_ANGLE_MIN : little_yaw_angle));
        }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Sentry) {
            if (JiuXuan_Status_Info.IsSentryAttack) {
                little_yaw_angle = ANGLE_TRANS_Absolute_to_Signed(yaw_angle) - ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle));
                little_yaw_angle = (little_yaw_angle > 180.0f ? little_yaw_angle - 360.0f : (little_yaw_angle < -180.0f ? little_yaw_angle + 360.0f : little_yaw_angle));
                little_yaw_angle = (little_yaw_angle > LITTLE_YAW_ANGLE_MAX ? LITTLE_YAW_ANGLE_MAX : (little_yaw_angle < LITTLE_YAW_ANGLE_MIN ? LITTLE_YAW_ANGLE_MIN : little_yaw_angle));
            }else {
                JiuXuan_Sentry_Scan_Mode_Little_Yaw.update();
                JiuXuan_Sentry_Scan_Mode_Pitch.update();
                // little_yaw_angle = (LITTLE_YAW_WORKSPACE_RANGE / 2.0f) * sinf(static_cast<float>(JiuXuan_System_Tick) / 400.0f);
                little_yaw_angle = JiuXuan_Sentry_Scan_Mode_Little_Yaw.Get_Triangle_Generator_Output();
                little_yaw_angle = little_yaw_angle > LITTLE_YAW_ANGLE_MAX ? LITTLE_YAW_ANGLE_MAX : little_yaw_angle < LITTLE_YAW_ANGLE_MIN ? LITTLE_YAW_ANGLE_MIN : little_yaw_angle;

                pitch_angle      = JiuXuan_Sentry_Scan_Mode_Pitch.Get_Triangle_Generator_Output();
            }
        }else {
            little_yaw_angle = ANGLE_TRANS_Absolute_to_Signed(yaw_angle) - ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle));
            little_yaw_angle = (little_yaw_angle > 180.0f ? little_yaw_angle - 360.0f : (little_yaw_angle < -180.0f ? little_yaw_angle + 360.0f : little_yaw_angle));
            little_yaw_angle = (little_yaw_angle > LITTLE_YAW_ANGLE_MAX ? LITTLE_YAW_ANGLE_MAX : (little_yaw_angle < LITTLE_YAW_ANGLE_MIN ? LITTLE_YAW_ANGLE_MIN : little_yaw_angle));
        }

        gimbal_navy_yaw  = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE));

        pitch_angle      = pitch_angle > PITCH_ANGLE_MAX ? PITCH_ANGLE_MAX : pitch_angle < PITCH_ANGLE_MIN ? PITCH_ANGLE_MIN : pitch_angle;

        /**
         * Controller Algorithm
         */
        FricLPID.update(friction_pulley_l(set_friction_speed), DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_vel);
        FricRPID.update(friction_pulley_r(set_friction_speed), DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_vel);

        JiuXuan_Pitch_Pos_PID.update(ANGLE_TRANS(pitch_angle, PITCH_INIT_ANGLE), DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos);
        JiuXuan_PITCH_Spd_PID.update(JiuXuan_Pitch_Pos_PID.getOutput() + 0.05 * (gimbal_pitch_omega_differential.getOutput() * M_PI / 180.0f), DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel);

        // Jiuxuan_PITCH_SMC.target = ANGLE_TRANS(pitch_angle, PITCH_INIT_ANGLE);
        // Jiuxuan_PITCH_SMC.SMC_Update(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos, DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel);

        // JiuXuan_Little_Yaw_Smc.target = 0.1 * ANGLE_TRANS(little_yaw_angle, LITTLE_YAW_INIT_ANGLE) + 0.9 * little_yaw_angle_last;
        JiuXuan_Little_Yaw_Smc.target = ANGLE_TRANS(0, LITTLE_YAW_INIT_ANGLE);
        JiuXuan_Little_Yaw_Smc.SMC_Update(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel);

        little_yaw_angle_last         = JiuXuan_Little_Yaw_Smc.target;

        // JiuXuan_Big_Yaw_Pos_PID.update(ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle), Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle + (Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180.0f / M_PI));
        JiuXuan_Big_Yaw_Pos_PID.update(ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle), big_yaw_fdb_angle_lpf);
        // JiuXuan_Big_Yaw_Pos_PID.update(ANGLE_TRANS_Absolute_to_Signed(big_yaw_angle), ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS(HIPNUC_Hl14S3.data_frame.yaw, gimbal_imu_init_yaw) + 180.0f));
        // ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS(HIPNUC_Hl14S3.data_frame.yaw, gimbal_imu_init_yaw) + 180.0f)
        JiuXuan_Big_Yaw_Spd_PID.update(JiuXuan_Big_Yaw_Pos_PID.getOutput() - 0.5 * (Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega), Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed);
        JiuXuan_Big_Yaw_Tor_PID.update(JiuXuan_Big_Yaw_Spd_PID.getOutput(), Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_current);

        // JiuXuan_Big_Yaw_Spd_PID.update(set_yaw_speed - Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed);

        // float target_spd = JiuXuan_Yaw_Ramp.update(set_yaw_speed);
        JiuXuan_Big_Yaw_Smc.target = (JiuXuan_Yaw_Ramp2.update() + 0.0 * Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_omega) * 0.001f;
        JiuXuan_Big_Yaw_Smc.C      = 0.2 + (fabs(JiuXuan_Big_Yaw_Smc.gainErr()) > 0.01 ? fabs(JiuXuan_Big_Yaw_Smc.gainErr()) * 0.1 : 0.0f);
        JiuXuan_Big_Yaw_Smc.SMC_Update(-(ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf - 0.0 * Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega), Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed + 1.25 * Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega);
        // JiuXuan_Big_Yaw_Smc.SMC_Update(-(big_yaw_angle - HIPNUC_Hl14S3.data_frame.yaw), Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed - 0 * Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega);

        // JiuXuan_Big_Yaw_Smc.SMC_Update(big_yaw_fdb_angle_lpf, HIPNUC_Hl14S3.data_frame.gyro_z);

        JiuXuan_Big_Yaw_LADRC.update(ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle), big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed);

        JiuXuan_Big_Yaw_Compliance.update(ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle), big_yaw_fdb_angle_lpf, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega);

        big_yaw_fdb_angle_last = big_yaw_fdb_angle;
    }
}

/**
 * @brief :
 */
void TIM3_State_Update_Callback(void) {
    if (IsInit) {
        TIM3_Tick++;

        JiuXuan_Fire_Semi_Auto_Trigger.Pulse_Rising_Edge();
        JiuXuan_RM_Client_Fire_Switch.Pulse_Rising_Edge();
        JiuXuan_RM_Client_Gyro_Switch.Pulse_Rising_Edge();
        JiuXuan_RM_Client_Semi_Switch.Pulse_Rising_Edge();

        // Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_Spd_PID.getOutput() + 0.000 * static_cast<float>(gimbal_yaw_torque_differential.getOutput());
        Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_controller_cmd = JiuXuan_Big_Yaw_Smc.u;
        // Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_LADRC.get_control_output();

        // Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_Tor_PID.getOutput();

        // float err = ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf > 180.0f ? ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf - 360.0f : ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf < -180.0f ? ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf + 360.0f : ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) - big_yaw_fdb_angle_lpf;

        // if (fabs(err) > 50.0f) {
        //     Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_Compliance.getComplianceTorque();
        // }else if (fabs(err) > 30.0f) {
        //     Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_Smc.u;
        // }else {
        //     Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = JiuXuan_Big_Yaw_Spd_PID.getOutput() + 0.01 * Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_current;
        // }

        // Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.set_big_yaw_torq = 0.5 * JiuXuan_Big_Yaw_Compliance.getComplianceTorque() + 0.5 * JiuXuan_Big_Yaw_Smc.u + 0.00 * JiuXuan_Big_Yaw_LADRC.get_control_output();

        Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.gimbal_yaw       = ANGLE_TRANS_Absolute_to_Signed(yaw_angle);
        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Sentry) Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.gimbal_yaw = ANGLE_TRANS_Absolute_to_Signed(0.0f);

        if (TIM3_Tick > 10) {
            TIM3_Tick = 0;

            Normal_Breathing();
        }

        if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick %  50 == 0) {
            HAL_GPIO_TogglePin(Sign_LED_GPIO_Port, Sign_LED_Pin);
        }

        if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 200 == 0) {
            Px_Tick++;
            if (Px_Tick > 3) Px_Tick = 0;
        }

        if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 1000 == 0) {
            Standard_link_node_freq = Standard_link_node_tick;

            Standard_link_node_tick = 0;
        }

        switch (Px_Tick) {
            case 1 : {
                HAL_GPIO_WritePin(P1_GPIO_Port, P1_Pin, GPIO_PIN_SET);
                break;
            }
            case 2 : {
                HAL_GPIO_WritePin(P2_GPIO_Port, P2_Pin, GPIO_PIN_SET);
                break;
            }
            case 3 : {
                HAL_GPIO_WritePin(P3_GPIO_Port, P3_Pin, GPIO_PIN_SET);
                break;
            }
            default : {
                HAL_GPIO_WritePin(P1_GPIO_Port, P1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(P2_GPIO_Port, P2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(P3_GPIO_Port, P3_Pin, GPIO_PIN_RESET);
                break;
            }
        }
    }
}

extern "C" {
    /**
     * @brief HAL库CAN接收FIFO0中断
     *
     * @param hfdcan CAN编号
     */
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer) == HAL_OK){
                if (CAN1_Manage_Object.Rx_Header.Identifier == 0x20B) isPitchInfoRXOK     = true;
                if (CAN1_Manage_Object.Rx_Header.Identifier == 0x20A) isLittleYawInfoRXOK = true;

                DJI_GM6020_FeedBack_Msg_UnpackingCallBack(CAN1_Manage_Object.Rx_Buffer, 8, CAN1_Manage_Object.Rx_Header.Identifier);

                isFDCAN1FIFO0RXOK = true;
            }
        }
        else if(hfdcan->Instance == FDCAN2)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.RX_Buffer_FDCAN) == HAL_OK){
                memcpy(&Sentry_Chassis_Gimbal_Interact_Info.rx_cmd, CAN2_Manage_Object.RX_Buffer_FDCAN, sizeof(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd));
                isBigYawInfoRXOK  = true;

                isFDCAN2FIFO0RXOK = true;
            }
        }
        else if(hfdcan->Instance == FDCAN3)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer) == HAL_OK){
                if (CAN3_Manage_Object.Rx_Header.Identifier == 0x201) DJI_M2006_P36_FeedBack_Msg_UpackingCallBack(CAN3_Manage_Object.Rx_Buffer, 8, CAN3_Manage_Object.Rx_Header.Identifier);
                else                                                  DJI_M3508_P19_FeedBack_Msg_UpackingCallBack(CAN3_Manage_Object.Rx_Buffer, 8, CAN3_Manage_Object.Rx_Header.Identifier);

                isFDCAN3FIFO0RXOK = true;
            }
        }
    }

    /**
     * @brief HAL库CAN错误中断
     *
     * @param hfdcan CAN编号
     * @param ErrorStatusITs 错误状态
     */
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
    {
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
        {
            isFDCAN1FIFO0RXOK = false;

            isFDCAN3FIFO0RXOK = false;
            // CAN总线离线, 重新启动CAN
            HAL_FDCAN_Stop(hfdcan);
            HAL_FDCAN_Start(hfdcan);
        }
    }

    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
        if (huart->Instance == USART1) {
            USART1_Manage_Object.Rx_Timeout_Count = 0;
            USART1_Manage_Object.UART_Timeout = false;

            USART1_Manage_Object.Rx_Buffer_Already = USART1_Manage_Object.Rx_Buffer_Active;
            if (USART1_Manage_Object.Rx_Buffer_Active == USART1_Manage_Object.Rx_Buffer_A) {
                USART1_Manage_Object.Rx_Buffer_Active = USART1_Manage_Object.Rx_Buffer_B;
            }else {
                USART1_Manage_Object.Rx_Buffer_Active = USART1_Manage_Object.Rx_Buffer_A;
            }

            USART1_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

            if (USART1_Manage_Object.Callback_Function != nullptr) {
                USART1_Manage_Object.Callback_Function(USART1_Manage_Object.Rx_Buffer_Already, Size);
            }
        }
        else if (huart->Instance == UART5) {
            UART5_Manage_Object.Rx_Timeout_Count = 0;
            UART5_Manage_Object.UART_Timeout = false;

            UART5_Manage_Object.Rx_Buffer_Already = UART5_Manage_Object.Rx_Buffer_Active;
            if (UART5_Manage_Object.Rx_Buffer_Active == UART5_Manage_Object.Rx_Buffer_A) {
                UART5_Manage_Object.Rx_Buffer_Active = UART5_Manage_Object.Rx_Buffer_B;
            }else {
                UART5_Manage_Object.Rx_Buffer_Active = UART5_Manage_Object.Rx_Buffer_A;
            }

            UART5_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

            if (UART5_Manage_Object.Callback_Function != nullptr) {
                UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer_Already, Size);
            }
        }
        else if (huart->Instance == UART8) {
            UART8_Manage_Object.Rx_Buffer_Already = UART8_Manage_Object.Rx_Buffer_Active;
            if (UART8_Manage_Object.Rx_Buffer_Active == UART8_Manage_Object.Rx_Buffer_A) {
                UART8_Manage_Object.Rx_Buffer_Active = UART8_Manage_Object.Rx_Buffer_B;
            }else {
                UART8_Manage_Object.Rx_Buffer_Active = UART8_Manage_Object.Rx_Buffer_A;
            }

            UART8_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;
            VTM_Timeout_Tick.timestamp        = 0;
            VTM_Timeout_Tick.isTimeout        = false;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart8, UART8_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

            if (UART8_Manage_Object.Callback_Function != nullptr) {
                UART8_Manage_Object.Callback_Function(UART8_Manage_Object.Rx_Buffer_Already, Size);
            }
        }
        else if (huart->Instance == UART9) {
            UART9_Manage_Object.Rx_Buffer_Already = UART9_Manage_Object.Rx_Buffer_Active;
            if (UART9_Manage_Object.Rx_Buffer_Active == UART9_Manage_Object.Rx_Buffer_A) {
                UART9_Manage_Object.Rx_Buffer_Active = UART9_Manage_Object.Rx_Buffer_B;
            }else {
                UART9_Manage_Object.Rx_Buffer_Active = UART9_Manage_Object.Rx_Buffer_A;
            }

            UART9_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART9_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);

            if (UART9_Manage_Object.Callback_Function != nullptr) {
                UART9_Manage_Object.Callback_Function(UART9_Manage_Object.Rx_Buffer_Already, Size);
            }
        }
        else if (huart->Instance == USART10) {
            USART10_Manage_Object.Rx_Timeout_Count = 0;
            USART10_Manage_Object.UART_Timeout = false;

            isIMUInfoRXOK                      = true;

            USART10_Manage_Object.Rx_Buffer_Already = USART10_Manage_Object.Rx_Buffer_Active;
            if (USART10_Manage_Object.Rx_Buffer_Active == USART10_Manage_Object.Rx_Buffer_A) {
                USART10_Manage_Object.Rx_Buffer_Active = USART10_Manage_Object.Rx_Buffer_B;
            }else {
                USART10_Manage_Object.Rx_Buffer_Active = USART10_Manage_Object.Rx_Buffer_A;
            }

            USART10_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, USART10_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);

            if (USART10_Manage_Object.Callback_Function != nullptr) {
                USART10_Manage_Object.Callback_Function(USART10_Manage_Object.Rx_Buffer_Already, Size);
            }
        }
    }

    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
        if (huart->Instance == UART7) {
            isUART7DMATXOK = false;
        }else if (huart->Instance == UART8) {

        }else if (huart->Instance == UART9) {
            isUART9DMATXOK = false;
        }
    }
}
