//
// Created by ASUS on 2026/8/5.
//
#include "../Inc/Drv_MiniPC.h"
#include "../Inc/MiniPC_CRC.h"

DRV_MiniPC_StatusTypedef MiniPC_Data_init(MiniPC_Msg_Info_t *minipc) {
    memset(minipc, 0, sizeof(MiniPC_Msg_Info_t));

    return DRV_MINIPC_OK;
}

DRV_MiniPC_StatusTypedef Set_Data_MiniPC(MiniPC_Msg_Info_t *minipc, UART_HandleTypeDef *huart, float    V_x_fdb,
                                                                                               float    V_y_fdb,
                                                                                               float    Omega_z_fdb,
                                                                                               float    Big_Yaw,
                                                                                               bool     isSentryUnderHP_,
                                                                                               bool     isSentryLackOfBullets_,
                                                                                               uint8_t  auto_aim_mode_req,
                                                                                               float    Gimbal_Eular_Yaw,
                                                                                               float    Gimbal_Eular_Pitch,
                                                                                               float    Gimbal_Eular_Roll,
                                                                                               float    bullet_speed,
                                                                                               uint32_t mcu_timestamp)
{
    minipc->Tx_Info.current_mode              = auto_aim_mode_req;
    minipc->Tx_Info.actual_vx                 = V_x_fdb;
    minipc->Tx_Info.actual_vy                 = V_y_fdb;
    minipc->Tx_Info.actual_wz                 = Omega_z_fdb;
    minipc->Tx_Info.big_yaw                   = Big_Yaw;
    minipc->Tx_Info.sentry_state              = (isSentryUnderHP_ << 1) | (isSentryLackOfBullets_ << 0);
    minipc->Tx_Info.little_yaw                = Gimbal_Eular_Yaw;
    minipc->Tx_Info.pitch                     = Gimbal_Eular_Pitch;
    minipc->Tx_Info.roll                      = Gimbal_Eular_Roll;
    minipc->Tx_Info.bullet_speed              = bullet_speed;
    minipc->Tx_Info.mcu_timestamp             = mcu_timestamp;

    minipc->Tx_buffer[0]                      = __TX_MINIPC_HEADER_1;
    minipc->Tx_buffer[1]                      = __TX_MINIPC_HEADER_2;

    memcpy(&minipc->Tx_buffer[2], &minipc->Tx_Info, 35);

    minipc_append_crc16_check_sum(minipc->Tx_buffer, MINIPC_TX_MSG_LEN);

    if (minipc_verify_crc16_check_sum(minipc->Tx_buffer, MINIPC_TX_MSG_LEN) != true) {
        return DRV_MINIPC_ERROR;
    }

    if (HAL_UART_Transmit_DMA(huart, minipc->Tx_buffer, MINIPC_TX_MSG_LEN) != HAL_OK) {
        return DRV_MINIPC_ERROR;
    }else {
        return DRV_MINIPC_OK;
    }
}

void Rx_Info_Data_Process(MiniPC_Msg_Info_t *minipc, uint8_t *Data, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++){
        if (Data[i] != __RX_MINIPC_HEADER_1) {
            continue;
        }

        if (Data[i + 1] != __RX_MINIPC_HEADER_2) {
            continue;
        }

        if (i + MINIPC_RX_MSG_LEN - 2 > Length) {
            break;
        }

        if (minipc_verify_crc16_check_sum(Data + i, MINIPC_RX_MSG_LEN) != true) {
            continue;
        }

        memcpy(&minipc->Rx_Info, Data + i + 2, sizeof(__MiniPC_Rx_Info_t));
    }
}
