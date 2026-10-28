//
// Created by ASUS on 2026/7/30.
//
#include "../Inc/RoboMaster_vtm_link_node.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/RoboMaster_Referee_System/Referee_Info_Define/Inc/RoboMaster_Referee_Info_Define.h"
#include <usart.h>

#include "../../../../Drv/RoboMaster_Referee_System/VTM_Link/Inc/drv_robomaster_vtm_link.h"

extern Drv_UART_Manage_Object           UART8_Manage_Object;
extern JiuXuan_Community_Tameout_Tick_t VTM_Timeout_Tick;

extern DMA_HandleTypeDef                hdma_uart8_rx;

void VTM_Link_Data_Process_Callback(uint8_t *Buffer, uint16_t Length) {
    RoboMaster_VTM_Link_Rx_Data_Process(Buffer, Length);
}

extern "C" void RoboMaster_VTM_Link_node(void *argument)
{
    /* USER CODE BEGIN RoboMaster_VTM_Link_node */
    VTM_Link_Init();

    UART_Init(&UART8_Manage_Object, &huart8, VTM_Link_Data_Process_Callback);

    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

#if VTM_VT02_Ver_20241225

    // __HAL_UART_DISABLE(&huart8);
    // huart8.Init.BaudRate = 115200;
    // HAL_UART_Init(&huart8);
    // __HAL_UART_ENABLE(&huart8);

#endif

    /* Infinite loop */
    for(;;)
    {
        if (VTM_Timeout_Tick.isTimeout) {
            VTM_Link_Clear();
        }
        osDelay(1);
    }
    /* USER CODE END RoboMaster_VTM_Link_node */
}
