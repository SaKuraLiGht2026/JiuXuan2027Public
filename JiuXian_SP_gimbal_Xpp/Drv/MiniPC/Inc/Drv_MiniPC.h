//
// Created by ASUS on 2026/8/5.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <usart.h>

#define __TX_MINIPC_HEADER_1        0x47 // "G" 广
#define __TX_MINIPC_HEADER_2        0x44 // "D" 大

#define __RX_MINIPC_HEADER_1        0x51 // "Q" 庆
#define __RX_MINIPC_HEADER_2        0x59 // “Y" 园

#define MINIPC_TX_MSG_LEN             43
#define MINIPC_RX_MSG_LEN             34

    typedef enum {
        DRV_MINIPC_OK      = 0x00U,
        DRV_MINIPC_ERROR   = 0x01U,
        DRV_MINIPC_BUSY    = 0x02U,
        DRV_MINIPC_TIMEOUT = 0x03U
    }DRV_MiniPC_StatusTypedef;

    typedef struct __attribute__((packed))
    {
        /* data frame */
        uint8_t    current_mode;
        float      actual_vx;
        float      actual_vy;
        float      actual_wz;
        float      big_yaw;

        uint16_t   sentry_state;

        float      little_yaw;
        float      pitch;
        float      roll;

        float      bullet_speed;
        uint32_t   mcu_timestamp;

    }__MiniPC_Tx_Info_t;

    typedef struct __attribute__((packed))
    {
        /* data frame */
        uint8_t    mode;

        float      yaw;
        float      pitch;

        float      linear_x;
        float      linear_y;

        float      angular_z;

        float      distance;

        uint32_t   reserved1;

        float      target_v_yaw;

    }__MiniPC_Rx_Info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t            Tx_buffer[MINIPC_TX_MSG_LEN];
        uint8_t            Rx_buffer[MINIPC_RX_MSG_LEN];

        __MiniPC_Tx_Info_t Tx_Info;
        __MiniPC_Rx_Info_t Rx_Info;
    }MiniPC_Msg_Info_t;

    DRV_MiniPC_StatusTypedef MiniPC_Data_init(MiniPC_Msg_Info_t *minipc);
    DRV_MiniPC_StatusTypedef Set_Data_MiniPC(MiniPC_Msg_Info_t *minipc, UART_HandleTypeDef *huart,  float    V_x_fdb,
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
                                                                                                    uint32_t mcu_timestamp);

    void Rx_Info_Data_Process(MiniPC_Msg_Info_t *minipc, uint8_t *Data, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H