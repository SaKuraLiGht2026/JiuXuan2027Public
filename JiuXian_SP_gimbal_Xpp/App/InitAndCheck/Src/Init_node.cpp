//
// Created by ASUS on 2026/7/14.
//
#include "../Inc/Init_node.h"
#include "../../PreApp/Vofa/Inc/app_vofa.hpp"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
#include "../../Drv/DJI/Inc/Drv_DJI.h"
#include "../../Drv/HIPNUC/Hl14S3/Inc/drv_HIPNUC_Hl14S3.h"
#include "../../Algorithm/PID/Inc/PID.hpp"
#include "../../Algorithm/SMC/Inc/alg_smc.hpp"
#include "../../Drv/RoboMaster_Referee_System/Standard_Link/Inc/drv_robomaster_standard_link.h"
#include "../../Drv/RoboMaster_Referee_System/VTM_Link/Inc/drv_robomaster_vtm_link.h"
#include "../../PreApp/Signal_Generator/Inc/Signal_Generator.hpp"
#include "usart.h"

extern bool                                           IsInit;
extern bool                                           isUART7DMATXOK;

extern float                                          pitch_angle;
extern float                                          yaw_angle;
extern float                                          little_yaw_angle;
extern float                                          big_yaw_angle;
extern float                                          set_yaw_speed;

extern float                                          big_yaw_fdb_angle_lpf;

extern DJI_Msg_INFO_t                                 DJI_GM6020_G2;

extern HIPNUC_data_info_t                             HIPNUC_Hl14S3;

extern RoboMAster_Referee_System_Standard_Link_Info_t RM_Referee_Standard_Link_Info;
extern RoboMaster_Referee_System_VTM_Link_Info_t      RM_Referee_VTM_Link_Info;

extern Sentry_Chassis_Gimbal_Interact_Info_t          Sentry_Chassis_Gimbal_Interact_Info;

extern Pulse_Generator                                JiuXuan_Fire_Semi_Auto_Trigger;

extern PIDController                                  JiuXuan_Big_Yaw_Pos_PID;
extern PIDController                                  JiuXuan_Big_Yaw_Spd_PID;
extern PIDController                                  JiuXuan_Big_Yaw_Tor_PID;

extern SMC                      JiuXuan_Big_Yaw_Smc;

extern float                                          gimbal_imu_init_yaw;
extern float                                          gimbal_imu_init_pitch;

VOFA::VOFA_Justfloat_Manager                          RoboMaster_Debug_Vofa_JuatFloat(&huart7);

extern "C" void InitNcheckTask(void *argument)
{
    /* USER CODE BEGIN InitNcheckTask */
    fdcan_classic_filter_init(&hfdcan1);
    fdcan_filter_init(&hfdcan2);
    fdcan_classic_filter_init(&hfdcan3);
    DJI_INIT();

    float Vofa_Data[20] = {0.0f};

    HAL_GPIO_WritePin(POWER_OUT1_GPIO_Port  , POWER_OUT1_Pin  , GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_OUT2_GPIO_Port  , POWER_OUT2_Pin  , GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_OUT_5V_GPIO_Port, POWER_OUT_5V_Pin, GPIO_PIN_SET);

    IsInit = true;
    /* Infinite loop */
    for(;;)
    {
        if (IsInit) {
            RoboMaster_Debug_Vofa_JuatFloat.Set_Tx_Buffer_Addr(0x30000000);

            Vofa_Data[0]  = set_yaw_speed;
            Vofa_Data[1]  = big_yaw_angle;
            Vofa_Data[2]  = little_yaw_angle;
            Vofa_Data[3]  = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI);
            Vofa_Data[4]  = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)) > 180.0f ? ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)) - 360.0f : ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)) < -180.0f? ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)) + 360.0f : ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE));
            Vofa_Data[5]  = JiuXuan_Big_Yaw_Pos_PID.getError();
            Vofa_Data[6]  = DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos;
            Vofa_Data[7]  = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) + ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE));
            Vofa_Data[8]  = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE));
            Vofa_Data[9]  = JiuXuan_Big_Yaw_Pos_PID.getOutput();
            Vofa_Data[10] = JiuXuan_Big_Yaw_Spd_PID.getOutput();
            Vofa_Data[11] = JiuXuan_Big_Yaw_Tor_PID.getOutput();
            Vofa_Data[12] = ANGLE_TRANS(big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle);
            Vofa_Data[13] = big_yaw_fdb_angle_lpf;
            Vofa_Data[14] = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_speed;
            Vofa_Data[15] = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_current;
            Vofa_Data[16] = 0;
            Vofa_Data[17] = 0;
            Vofa_Data[18] = 0;
            Vofa_Data[19] = 0;

            if (isUART7DMATXOK == false) {
                isUART7DMATXOK = true;
                RoboMaster_Debug_Vofa_JuatFloat.Write_data(Vofa_Data, VOFA_SHOW_NUM_20);
            }
        }
        // taskYIELD();
        osDelay(1);
    }
    /* USER CODE END InitNcheckTask */
}
