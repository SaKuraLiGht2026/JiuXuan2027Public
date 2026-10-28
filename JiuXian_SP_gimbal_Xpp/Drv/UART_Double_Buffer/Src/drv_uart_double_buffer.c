//
// Created by ASUS on 2026/7/20.
//
#include "../Inc/drv_uart_double_buffer.h"

DRV_UART_Manage_StatusTypedef UART_Init(Drv_UART_Manage_Object *UART_Manage_Object, UART_HandleTypeDef *huart, UART_Callback Callback_Function) {
    if (UART_Manage_Object != NULL) {
        UART_Manage_Object->Rx_Timeout_Count = 0;
        UART_Manage_Object->UART_Timeout = false;

        UART_Manage_Object->UART_Handler = huart;
        UART_Manage_Object->Callback_Function = Callback_Function;

        UART_Manage_Object->Rx_Buffer_Active = UART_Manage_Object->Rx_Buffer_A;
        HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->UART_Handler, UART_Manage_Object->Rx_Buffer_Active, UART_BUFFER_SIZE);
        return DRV_UART_OK;
    }else {
        return DRV_UART_ERR;
    }
}

DRV_UART_Manage_StatusTypedef UART_Reinit(Drv_UART_Manage_Object *UART_Manage_Object, UART_HandleTypeDef *huart) {
    if (UART_Manage_Object != NULL) {
        UART_Manage_Object->Rx_Timeout_Count = 0;
        UART_Manage_Object->UART_Timeout = false;

        UART_Manage_Object->UART_Handler = huart;
        UART_Manage_Object->Rx_Buffer_Active = UART_Manage_Object->Rx_Buffer_A;
        HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->UART_Handler, UART_Manage_Object->Rx_Buffer_Active, UART_BUFFER_SIZE);
        return DRV_UART_OK;
    } else {
        return DRV_UART_ERR;
    }
}

DRV_UART_Manage_StatusTypedef UART_Receive_DMA_Repeat(const Drv_UART_Manage_Object *UART_Manage_Object) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->UART_Handler, UART_Manage_Object->Rx_Buffer_Active, UART_BUFFER_SIZE) == HAL_OK) {
        return DRV_UART_OK;
    }else {
        return DRV_UART_ERR;
    }
}
