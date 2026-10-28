//
// Created by ASUS on 2026/7/14.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_BSP_FDCAN_H
#define JIUXIAN_SP_GIMBAL_XPP_BSP_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <main.h>
#include <fdcan.h>
#include <string.h>

    extern uint8_t CAN1_0x1fe_Tx_Data[];
    extern uint8_t CAN1_0x1ff_Tx_Data[];
    extern uint8_t CAN1_0x200_Tx_Data[];
    extern uint8_t CAN1_0x2fe_Tx_Data[];
    extern uint8_t CAN1_0x2ff_Tx_Data[];
    extern uint8_t CAN1_0x3fe_Tx_Data[];
    extern uint8_t CAN1_0x4fe_Tx_Data[];

    extern uint8_t CAN2_0x1fe_Tx_Data[];
    extern uint8_t CAN2_0x1ff_Tx_Data[];
    extern uint8_t CAN2_0x200_Tx_Data[];
    extern uint8_t CAN2_0x2fe_Tx_Data[];
    extern uint8_t CAN2_0x2ff_Tx_Data[];
    extern uint8_t CAN2_0x3fe_Tx_Data[];
    extern uint8_t CAN2_0x4fe_Tx_Data[];

    extern uint8_t CAN3_0x1fe_Tx_Data[];
    extern uint8_t CAN3_0x1ff_Tx_Data[];
    extern uint8_t CAN3_0x200_Tx_Data[];
    extern uint8_t CAN3_0x2fe_Tx_Data[];
    extern uint8_t CAN3_0x2ff_Tx_Data[];
    extern uint8_t CAN3_0x3fe_Tx_Data[];
    extern uint8_t CAN3_0x4fe_Tx_Data[];

    typedef struct
    {
        FDCAN_HandleTypeDef *CAN_Handler;

        FDCAN_RxHeaderTypeDef Rx_Header;
        uint8_t Rx_Buffer[8];
        uint8_t RX_Buffer_FDCAN[64];
    }Struct_CAN_Manage_Object;

    void fdcan_classic_filter_init(FDCAN_HandleTypeDef *hfdcan);
    void fdcan_filter_init(FDCAN_HandleTypeDef *hfdcan);

    uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

    uint8_t FDCAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_BSP_FDCAN_H