//
// Created by ASUS on 2026/7/30.
//
#include "../Inc/RoboMaster_standard_link_node.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/RoboMaster_Referee_System/Referee_Info_Define/Inc/RoboMaster_Referee_Info_Define.h"
#include "../../Drv/RoboMaster_Referee_System/Standard_Link/Inc/drv_robomaster_standard_link.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include <usart.h>

extern bool                     IsUseRMRefereeSystem;

extern Drv_UART_Manage_Object   USART1_Manage_Object;

extern DMA_HandleTypeDef        hdma_usart1_rx;

extern uint64_t                 Standard_link_node_tick;

extern JiuXuan_Status_Info_t                          JiuXuan_Status_Info;
extern Sentry_Chassis_Gimbal_Interact_Info_t          Sentry_Chassis_Gimbal_Interact_Info;
extern RoboMAster_Referee_System_Standard_Link_Info_t RM_Referee_Standard_Link_Info;

void Standard_Link_Data_Process_Callback(uint8_t *Buffer, uint16_t Length) {
    RoboMaster_Standard_Link_Rx_Data_Process(Buffer, Length);
}

extern "C" void RoboMaster_Standard_Link_node(void *argument)
{
    /* USER CODE BEGIN RoboMaster_Standard_Link_node */
    UART_Init(&USART1_Manage_Object, &huart1, Standard_Link_Data_Process_Callback);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    Standard_Link_Init();

    /* Infinite loop */
    for(;;)
    {
        Standard_link_node_tick++;

        if (USART1_Manage_Object.UART_Timeout) {
            UART_Receive_DMA_Repeat(&USART1_Manage_Object);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

            IsUseRMRefereeSystem = false;
        }else {
            IsUseRMRefereeSystem = true;
        }

        if (IsUseRMRefereeSystem) {
            JiuXuan_Status_Info.IsAlive = RM_Referee_Standard_Link_Info.robot_status.current_HP > 0 ? true : false;
        }else {
            JiuXuan_Status_Info.IsAlive = true;
        }

        // interaction_figure_t fig = {0};
        //
        // fig.figure_name[0] = 'T';
        // fig.figure_name[1] = 'E';
        // fig.figure_name[2] = 'S';
        //
        // fig.operate_tpye = 1;   // 新增
        // fig.figure_tpye  = 1;   // 直线
        // fig.layer        = 0;
        // fig.color        = 2;   // 蓝色
        // fig.width        = 3;
        //
        // fig.start_x = 400;
        // fig.start_y = 400;
        // fig.details_d = 800;   // end_x
        // fig.details_e = 800;   // end_y
        //
        // RoboMaster_Interaction_UI_Design_One_Graphic(&fig);

        osDelay(1);
    }
    /* USER CODE END RoboMaster_Standard_Link_node */
}