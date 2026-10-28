//
// Created by ASUS on 2026/10/2.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_BSP_RS485_H
#define JIUXIAN_SP_CHASSIS_XPP_BSP_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdbool.h"
#include "usart.h"
#include "dma.h"

typedef enum {
    TX     = 0x00U,
    TX_IT  = 0x01U,
    TX_DMA = 0x02U
}RS485_TX_MODE;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *DE_GPIO_Port;
    uint16_t            DE_Pin;

    RS485_TX_MODE       tx_mode;
    bool                is_tx;
    bool                auto_switch_DE;
}Struct_RS485_Manage_Object;

/* ---------- 接口 ---------- */
void BSP_RS485_Init(Struct_RS485_Manage_Object *rs485_manage_object, UART_HandleTypeDef *huart, GPIO_TypeDef *DE_GPIO_Port, uint16_t DE_Pin, RS485_TX_MODE tx_mode, bool auto_switch_DE);   /* 置为接收态 */
void BSP_RS485_Send(Struct_RS485_Manage_Object *rs485_manage_object, const uint8_t *data, uint16_t len);                                                               /* 发送任意数据 */

void RS485_TX(Struct_RS485_Manage_Object *rs485_manage_object);
void RS485_RX(Struct_RS485_Manage_Object *rs485_manage_object);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_CHASSIS_XPP_BSP_RS485_H