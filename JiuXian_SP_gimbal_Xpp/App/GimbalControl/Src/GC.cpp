//
// Created by ASUS on 2026/8/15.
//
#include "../Inc/GC.h"
#include "../../Algorithm/PID/Inc/PID.hpp"
#include "../../Algorithm/ComplianceControl/Inc/ComplianceControl.hpp"
#include "../../Algorithm/LADRC/Inc/LADRC.hpp"
#include "../../Algorithm/SMC/Inc/alg_smc.hpp"
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Drv/DJI/Inc/Drv_DJI.h"
#include "../../Drv/WFLY/Inc/Drv_WFLY.h"

#define Motor_Calibration       0

/**
 * Param of Big Yaw PID
 */
#define BIG_YAW_POS_PID_KP      1.00f
#define BIG_YAW_POS_PID_KI      0.1f
#define BIG_YAW_POS_PID_KD      0.004f
#define BIG_YAW_POS_PID_KF      0.0f
#define BIG_YAW_POS_PID_I_LIM   1.8f
#define BIG_YAW_POS_PID_U_LIM   20.0f

#define BIG_YAW_SPD_PID_KP      0.1f
#define BIG_YAW_SPD_PID_KI      0.00000f
#define BIG_YAW_SPD_PID_KD      0.0000f
#define BIG_YAW_SPD_PID_KF      0.0f
#define BIG_YAW_SPD_PID_I_LIM   0.5f
#define BIG_YAW_SPD_PID_U_LIM   1.5f

#define BIG_YAW_LADRC_WC        1000.f

/* ========================================================================================================================================= */

PIDController                   JiuXuan_Pitch_Pos_PID(3.00f, 0.000f, 0.005f, 0.0f, 8.0f, 35.0f, true);
PIDController                   JiuXuan_PITCH_Spd_PID(0.50f, 0.0005f, 0.0f, 0.0f, 5.0f, 22.0f, false);

SMC                             Jiuxuan_PITCH_SMC(30, 58, 0.001, 21, 27,22.0, 0.0024, 0.5);
SMC                             JiuXuan_Little_Yaw_Smc(25, 35, 0.001, 21, 27,22.0, 0.0024, 0.5);
SMC                             JiuXuan_Big_Yaw_Smc(0.4, 1900, 0.001, 3, 5, 23.8, 0.0024, 0.5);

PIDController                   JiuXuan_Big_Yaw_Pos_PID(BIG_YAW_POS_PID_KP, BIG_YAW_POS_PID_KI, BIG_YAW_POS_PID_KD, BIG_YAW_POS_PID_KF, BIG_YAW_POS_PID_I_LIM, BIG_YAW_POS_PID_U_LIM, true);
PIDController                   JiuXuan_Big_Yaw_Spd_PID(BIG_YAW_SPD_PID_KP, BIG_YAW_SPD_PID_KI, BIG_YAW_SPD_PID_KD, BIG_YAW_SPD_PID_KF, BIG_YAW_SPD_PID_I_LIM, BIG_YAW_SPD_PID_U_LIM, false);
PIDController                   JiuXuan_Big_Yaw_Tor_PID(0.45, 0.001f, 0.01f, 0.0f, 0.5f, 2.5f, false);

LADRC                           JiuXuan_Big_Yaw_LADRC(0.0036f, BIG_YAW_LADRC_WC, 400, -1.5f, 1.5f, 0.001);

ComplianceControl               JiuXuan_Big_Yaw_Compliance(0.5f, 0.01f, 2.5f, -2.5f);

/* pricate extern --------------------------------------------------- */
extern bool                     IsInit;

extern bool                     isFDCAN1FIFO0RXOK;

extern float                    DJI_M2006_Torq_1_to_4[4];
extern float                    DJI_M2006_Torq_5_to_8[4];
extern float                    DJI_M3508_Torq_1_to_4[4];
extern float                    DJI_M3508_Torq_5_to_8[4];
extern float                    DJI_GM6020_Torq_1_to_4[4];
extern float                    DJI_GM6020_Torq_5_to_7[4];
extern float                    DJI_GM6020_Vol_1_to_4[4];
extern float                    DJI_GM6020_Vol_5_to_7[4];

extern float                    pitch_angle;
extern float                    yaw_angle;

extern DJI_Msg_INFO_t           DJI_M2006_P36_G1;
extern DJI_Msg_INFO_t           DJI_M2006_P36_G2;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G1;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G2;
extern DJI_Msg_INFO_t           DJI_GM6020_G1;
extern DJI_Msg_INFO_t           DJI_GM6020_G2;

/**
 * @brief the thread of gimbal ctrl.
 * @param argument
 */
extern "C" void Gimbal_Ctrl_node(void *argument)
{
    /* USER CODE BEGIN Gimbal_Ctrl_node */
    JiuXuan_Little_Yaw_Smc.nearby = true;
    JiuXuan_Big_Yaw_Smc.nearby    = true;
    /* Infinite loop */
    for(;;)
    {
        if (IsInit) {

            if (isFDCAN1FIFO0RXOK) {

                // JiuXuan_Big_Yaw_Pos_PID.AdaptiveKp(1.2 * BIG_YAW_POS_PID_KP, 0.8 * BIG_YAW_POS_PID_KP, 0.1);

                // JiuXuan_Big_Yaw_Spd_PID.AdaptiveKp(1.1 * BIG_YAW_SPD_PID_KP, 0.9 * BIG_YAW_SPD_PID_KP, 0.05);

#if Motor_Calibration

                DJI_GM6020_Vol_5_to_7[0] = DATA_RESERVE_f;
                DJI_GM6020_Vol_5_to_7[1] = DATA_RESERVE_f;
                DJI_GM6020_Vol_5_to_7[2] = DATA_RESERVE_f;
                DJI_GM6020_Vol_5_to_7[3] = DATA_RESERVE_f;

#else

                DJI_GM6020_Vol_5_to_7[0] = DATA_RESERVE_f;
                DJI_GM6020_Vol_5_to_7[1] = JiuXuan_Little_Yaw_Smc.u;
                DJI_GM6020_Vol_5_to_7[2] = JiuXuan_PITCH_Spd_PID.getOutput() + 0.0;
                DJI_GM6020_Vol_5_to_7[3] = DATA_RESERVE_f;
#endif

                DJI_OUTPUT_Vol_MEMCPY(&DJI_GM6020_G2, DJI_GM6020_Vol_5_to_7, 4);
                /* GM6020 data sent */
                DJI_Ctrl_Msg_data_pack(&DJI_GM6020_G2, CAN1_0x2ff_Tx_Data, 8, DJI_GM6020_Vol_GROUP_2);
                if (CAN_Transmit_Data(&hfdcan1, DJI_GM6020_G2.DJI_Ctrl_Msg.tx_ID, CAN1_0x2ff_Tx_Data, 8) == HAL_OK) {
                    isFDCAN1FIFO0RXOK = false;
                }
            }
        }
        osDelay(1);
    }
    /* USER CODE END Gimbal_Ctrl_node */
}
