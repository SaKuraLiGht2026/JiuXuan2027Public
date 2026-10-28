//
// Created by ASUS on 2026/7/20.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_DRV_UART_DOUBLE_BUFFER_H
#define JIUXIAN_SP_CHASSIS_XPP_DRV_UART_DOUBLE_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <usart.h>
#include <string.h>

#define UART_BUFFER_SIZE 512

/**
 * @brief UART通信接收回调函数数据类型
 */
typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

    typedef enum {
        DRV_UART_OK       = 0x00U,
        DRV_UART_ERR      = 0x01U,
        DRV_UART_BUSY     = 0x02U,
        DRV_UART_TIMEOUT  = 0x03U
    }DRV_UART_Manage_StatusTypedef;

    typedef struct {
        UART_HandleTypeDef *UART_Handler;
        UART_Callback Callback_Function;

        uint8_t Rx_Buffer_A[UART_BUFFER_SIZE];
        uint8_t Rx_Buffer_B[UART_BUFFER_SIZE];

        uint8_t *Rx_Buffer_Active;
        uint8_t *Rx_Buffer_Already;

        // 接收时间戳
        uint64_t Rx_Time_Stamp;
    }Drv_UART_Manage_Object;

    DRV_UART_Manage_StatusTypedef UART_Init(Drv_UART_Manage_Object *UART_Manage_Object, UART_HandleTypeDef *huart, UART_Callback Callback_Function);

    DRV_UART_Manage_StatusTypedef UART_Reinit(Drv_UART_Manage_Object *UART_Manage_Object, UART_HandleTypeDef *huart);

    DRV_UART_Manage_StatusTypedef UART_Receive_DMA_Repeat(const Drv_UART_Manage_Object *UART_Manage_Object);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_CHASSIS_XPP_DRV_UART_DOUBLE_BUFFER_H