//
// Created by ASUS on 2026/10/2.
//
#include "../Inc/bsp_rs485.h"
#include "string.h"

void BSP_RS485_Init(Struct_RS485_Manage_Object *rs485_manage_object, UART_HandleTypeDef *huart, GPIO_TypeDef *DE_GPIO_Port, uint16_t DE_Pin, RS485_TX_MODE tx_mode, bool auto_switch_DE)
{
    rs485_manage_object->huart = huart;
    rs485_manage_object->DE_GPIO_Port = DE_GPIO_Port;
    rs485_manage_object->DE_Pin = DE_Pin;
    rs485_manage_object->tx_mode = tx_mode;
    rs485_manage_object->is_tx = false;
    rs485_manage_object->auto_switch_DE = auto_switch_DE;
}

void BSP_RS485_Send(Struct_RS485_Manage_Object *rs485_manage_object, const uint8_t *data, uint16_t len)
{
    if (rs485_manage_object->tx_mode == TX) {
        if (!rs485_manage_object->auto_switch_DE) {
            RS485_TX(rs485_manage_object);
        }
        if (HAL_UART_Transmit(rs485_manage_object->huart, (uint8_t *)data, len, HAL_MAX_DELAY) != HAL_OK) {
            // Handle transmission error
            rs485_manage_object->is_tx = false;
            return;
        }else {
            rs485_manage_object->is_tx = true;
        }
    } else if (rs485_manage_object->tx_mode == TX_IT) {
        if (!rs485_manage_object->auto_switch_DE) {
            RS485_TX(rs485_manage_object);
        }
        if (HAL_UART_Transmit_IT(rs485_manage_object->huart, (uint8_t *)data, len) != HAL_OK) {
            // Handle transmission error
            rs485_manage_object->is_tx = false;
            return;
        }else {
            rs485_manage_object->is_tx = true;
        }
    } else if (rs485_manage_object->tx_mode == TX_DMA) {
        if (!rs485_manage_object->auto_switch_DE) {
            RS485_TX(rs485_manage_object);
        }
        if (HAL_UART_Transmit_DMA(rs485_manage_object->huart, (uint8_t *)data, len) != HAL_OK) {
            // Handle transmission error
            rs485_manage_object->is_tx = false;
            return;
        }else {
            rs485_manage_object->is_tx = true;
        }
    } else {
        return;
    }
}

void RS485_TX(Struct_RS485_Manage_Object *rs485_manage_object)
{
    if (rs485_manage_object->DE_GPIO_Port != NULL && rs485_manage_object->DE_Pin != -1) {
        HAL_GPIO_WritePin(rs485_manage_object->DE_GPIO_Port, rs485_manage_object->DE_Pin, GPIO_PIN_SET); // Set DE pin high for transmission
    }else {
        return;
    }
}

void RS485_RX(Struct_RS485_Manage_Object *rs485_manage_object)
{
    if (rs485_manage_object->DE_GPIO_Port != NULL && rs485_manage_object->DE_Pin != -1) {
        HAL_GPIO_WritePin(rs485_manage_object->DE_GPIO_Port, rs485_manage_object->DE_Pin, GPIO_PIN_RESET); // Set DE pin low for reception
    }else {
        return;
    }
}
