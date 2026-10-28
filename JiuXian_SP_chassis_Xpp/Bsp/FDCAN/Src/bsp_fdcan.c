//
// Created by ASUS on 2026/4/4.
//
/**
 *
 */
#include "../Inc/bsp_fdcan.h"

Struct_CAN_Manage_Object CAN1_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN2_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN3_Manage_Object = {NULL};

// CAN1
uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

// CAN2
uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

// CAN3
uint8_t CAN3_0x1fe_Tx_Data[8];
uint8_t CAN3_0x1ff_Tx_Data[8];
uint8_t CAN3_0x200_Tx_Data[8];
uint8_t CAN3_0x2fe_Tx_Data[8];
uint8_t CAN3_0x2ff_Tx_Data[8];
uint8_t CAN3_0x3fe_Tx_Data[8];
uint8_t CAN3_0x4fe_Tx_Data[8];

void fdcan_classic_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef can_filter_init_structure;

    can_filter_init_structure.IdType = FDCAN_STANDARD_ID;
    can_filter_init_structure.FilterIndex = 0;
    can_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
    can_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_init_structure.FilterID1 = 0x00000000;
    can_filter_init_structure.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &can_filter_init_structure);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(hfdcan);
}

void fdcan_filter_init(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef FDCAN_FilterConfig;

    FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN_FilterConfig.FilterIndex = 0;
    FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_FilterConfig.FilterID1 = 0x00000000;
    FDCAN_FilterConfig.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_EnableTxDelayCompensation(hfdcan);//开启FDCAN的发送延迟补偿

    HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 5, 5);//设置补偿时间 参数2和参数3都为TimeSeg1的值

    HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = Length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data));
}

/**
 * @brief 发送数据帧(FDCAN Fifo/Queue版本)
 *
 * @param hfdcan
 * @param ID
 * @param Data
 * @param Length
 * @return
 */
uint8_t FDCAN_Transmit_Data_with_TxFifoQ(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    if      (Length ==  0) tx_header.DataLength = FDCAN_DLC_BYTES_0 ;
    else if (Length ==  1) tx_header.DataLength = FDCAN_DLC_BYTES_1 ;
    else if (Length ==  2) tx_header.DataLength = FDCAN_DLC_BYTES_2 ;
    else if (Length ==  3) tx_header.DataLength = FDCAN_DLC_BYTES_3 ;
    else if (Length ==  4) tx_header.DataLength = FDCAN_DLC_BYTES_4 ;
    else if (Length ==  5) tx_header.DataLength = FDCAN_DLC_BYTES_5 ;
    else if (Length ==  6) tx_header.DataLength = FDCAN_DLC_BYTES_6 ;
    else if (Length ==  7) tx_header.DataLength = FDCAN_DLC_BYTES_7 ;
    else if (Length ==  8) tx_header.DataLength = FDCAN_DLC_BYTES_8 ;
    else if (Length == 16) tx_header.DataLength = FDCAN_DLC_BYTES_16;
    else if (Length == 20) tx_header.DataLength = FDCAN_DLC_BYTES_20;
    else if (Length == 24) tx_header.DataLength = FDCAN_DLC_BYTES_24;
    else if (Length == 32) tx_header.DataLength = FDCAN_DLC_BYTES_32;
    else if (Length == 48) tx_header.DataLength = FDCAN_DLC_BYTES_48;
    else if (Length == 64) tx_header.DataLength = FDCAN_DLC_BYTES_64;
    else return HAL_ERROR;

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 发送数据帧(FDCAN Buffer版本)
 *
 * @param hfdcan
 * @param ID
 * @param Data
 * @param Length
 * @return
 */
uint8_t FDCAN_Transmit_Data_with_TxBuffer(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    if      (Length ==  0) tx_header.DataLength = FDCAN_DLC_BYTES_0 ;
    else if (Length ==  1) tx_header.DataLength = FDCAN_DLC_BYTES_1 ;
    else if (Length ==  2) tx_header.DataLength = FDCAN_DLC_BYTES_2 ;
    else if (Length ==  3) tx_header.DataLength = FDCAN_DLC_BYTES_3 ;
    else if (Length ==  4) tx_header.DataLength = FDCAN_DLC_BYTES_4 ;
    else if (Length ==  5) tx_header.DataLength = FDCAN_DLC_BYTES_5 ;
    else if (Length ==  6) tx_header.DataLength = FDCAN_DLC_BYTES_6 ;
    else if (Length ==  7) tx_header.DataLength = FDCAN_DLC_BYTES_7 ;
    else if (Length ==  8) tx_header.DataLength = FDCAN_DLC_BYTES_8 ;
    else if (Length == 16) tx_header.DataLength = FDCAN_DLC_BYTES_16;
    else if (Length == 20) tx_header.DataLength = FDCAN_DLC_BYTES_20;
    else if (Length == 24) tx_header.DataLength = FDCAN_DLC_BYTES_24;
    else if (Length == 32) tx_header.DataLength = FDCAN_DLC_BYTES_32;
    else if (Length == 48) tx_header.DataLength = FDCAN_DLC_BYTES_48;
    else if (Length == 64) tx_header.DataLength = FDCAN_DLC_BYTES_64;
    else return HAL_ERROR;

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxBuffer(hfdcan, &tx_header, Data, FDCAN_TX_BUFFER0) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_FDCAN_EnableTxBufferRequest(hfdcan, FDCAN_TX_BUFFER0);

    return HAL_OK;
}
