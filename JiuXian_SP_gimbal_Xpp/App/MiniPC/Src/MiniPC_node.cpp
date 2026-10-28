//
// Created by ASUS on 2026/8/6.
//
#include "../Inc/MiniPC_node.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Drv/DJI/Inc/Drv_DJI.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/MiniPC/Inc/Drv_MiniPC.h"
#include "../../Drv/HIPNUC/Hl14S3/Inc/drv_HIPNUC_Hl14S3.h"
#include "../../Drv/RoboMaster_Referee_System/Standard_Link/Inc/drv_robomaster_standard_link.h"
#include <usart.h>

bool                            IsMiniPCConnectTimeout = false;

uint64_t                        minipc_connect_tick = 0;

extern bool                     IsInit;
extern bool                     isUART9DMATXOK;

extern float                    gimbal_imu_init_yaw;
extern float                    gimbal_imu_init_pitch;

extern uint64_t                 JiuXuan_System_Tick;

extern DJI_Msg_INFO_t           DJI_GM6020_G2;

extern DMA_HandleTypeDef        hdma_uart9_rx;

extern Drv_UART_Manage_Object   UART9_Manage_Object;

extern HIPNUC_data_info_t       HIPNUC_Hl14S3;

extern MiniPC_Msg_Info_t        minipc_msg_info;

extern Sentry_Chassis_Gimbal_Interact_Info_t          Sentry_Chassis_Gimbal_Interact_Info;

extern RoboMAster_Referee_System_Standard_Link_Info_t RM_Referee_Standard_Link_Info;

void MiniPC_Data_Receiver_CpltCallback(uint8_t *Buffer, uint16_t Length) {
    Rx_Info_Data_Process(&minipc_msg_info, Buffer, Length);

    minipc_connect_tick = 0;
    IsMiniPCConnectTimeout = false;
}

extern "C" void MiniPC_node(void *argument)
{
    /* USER CODE BEGIN MiniPC_node */
    MiniPC_Data_init(&minipc_msg_info);
    UART_Init(&UART9_Manage_Object, &huart9, MiniPC_Data_Receiver_CpltCallback);

    __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
    /* Infinite loop */
    for(;;)
    {
        if(IsInit) {
            UART_Receive_DMA_Repeat(&UART9_Manage_Object);
            __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);

            minipc_connect_tick++;
            if (minipc_connect_tick > 1000) {
                minipc_connect_tick = 1000;
                IsMiniPCConnectTimeout = true;
            }

            if (IsMiniPCConnectTimeout) {
                memset(&minipc_msg_info.Rx_Info, 0, sizeof(__MiniPC_Rx_Info_t));  // Clear the received data structure
            }

            if (isUART9DMATXOK == false) {
                isUART9DMATXOK = true;
                Set_Data_MiniPC(&minipc_msg_info, &huart9,
                                 Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_vel_x,
                                 Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_vel_y,
                                 Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.fdb_omega,
                                 ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle) + Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI),
                                 static_cast<float>(RM_Referee_Standard_Link_Info.robot_status.current_HP) / static_cast<float>(RM_Referee_Standard_Link_Info.robot_status.maximum_HP) < 0.1f ? true : false,
                                 RM_Referee_Standard_Link_Info.projectile_allowance.projectile_allowance_17mm < 20 ? true : false,
                                 RM_Referee_Standard_Link_Info.Robot_Faction == RED ? 0x01 : 0x00,
                                 ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) +
                                        Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI                                                                                                   +
                                        ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)),
                                 ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos, PITCH_INIT_ANGLE)),
                                 ANGLE_TRANS_Absolute_to_Signed(HIPNUC_Hl14S3.data_frame.roll + 180.0f),
                                 RM_Referee_Standard_Link_Info.shoot_data.initial_speed,
                                 JiuXuan_System_Tick
                                 );
            }
        }
        osDelay(1);
    }
    /* USER CODE END MiniPC_node */
}
