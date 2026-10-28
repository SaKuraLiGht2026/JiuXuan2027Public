//
// Created by ASUS on 2026/7/22.
//
#include <usart.h>
#include <dma.h>
#include "../Inc/\InitAndState_node.h"
#include "../../../Config/Inc/JiuXian_Config.hpp"

extern DMA_HandleTypeDef      hdma_uart8_rx;
extern DMA_HandleTypeDef      hdma_uart9_rx;

extern bool                   IsChassisSystemInit;

extern uint8_t                uart8_uart_test_buffer[256];
extern uint8_t                uart9_uart_test_buffer[256];

extern uint64_t               init_node_tick;

extern "C" void InitAndStateUpdate(void *argument)
{
    /* USER CODE BEGIN InitAndStateUpdate */

    /* Open User Power 24V/5V */
    HAL_GPIO_WritePin(POWER_OUT1_GPIO_Port  , POWER_OUT1_Pin  , GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_OUT2_GPIO_Port  , POWER_OUT2_Pin  , GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_OUT_5V_GPIO_Port, POWER_OUT_5V_Pin, GPIO_PIN_SET);

    /* Open UART Rx */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_uart_test_buffer, 256);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_uart_test_buffer, 256);
    __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);

    /* Initialization Complete! */
    IsChassisSystemInit = true;
    /* Infinite loop */
    for(;;)
    {
        init_node_tick++;

        if(IsChassisSystemInit) {

        }
        // osDelay(1);
        taskYIELD();
    }
    /* USER CODE END InitAndStateUpdate */
}
