//
// Created by ASUS on 2026/7/14.
//
#include "../Inc/FC.h"
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
#include "../../Config/JiuXuan_Status.hpp"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Drv/DJI/Inc/Drv_DJI.h"
#include "../../Drv/WFLY/Inc/Drv_WFLY.h"
#include "../../Algorithm/PID/Inc/PID.hpp"

#define CAN3_Test             0
#define Motor_Open_Loop_Test  0
#define Motor_Close_Loop_Test 1

extern float                    DJI_M2006_Torq_1_to_4[4];
extern float                    DJI_M2006_Torq_5_to_8[4];
extern float                    DJI_M3508_Torq_1_to_4[4];
extern float                    DJI_M3508_Torq_5_to_8[4];
extern float                    DJI_GM6020_Torq_1_to_4[4];
extern float                    DJI_GM6020_Torq_5_to_7[4];
extern float                    DJI_GM6020_Vol_1_to_4[4];
extern float                    DJI_GM6020_Vol_5_to_7[4];

extern DJI_Msg_INFO_t           DJI_M2006_P36_G1;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G2;

extern JiuXuan_Status_Info_t    JiuXuan_Status_Info;

extern WFLY_Data_t              WFLY_Data;

extern bool                     isFDCAN3FIFO0RXOK;

extern bool                     isPluckPIDCalcOK;

extern bool                     IsInit;

PIDController PluckPID(0.43, 0.01, 0, 0, 0.3, 1.8, false);
PIDController PluckSemiPID(1.5, 0.001, 0.005, 0, 4.8, 22.0, true);
PIDController FricLPID(0.13, 0.005, 0, 0, 0.3, 1.0, false);
PIDController FricRPID(0.13, 0.005, 0, 0, 0.3, 1.0, false);

extern "C" void FC_node(void *argument)
{
    /* USER CODE BEGIN FC_node */
#if CAN3_Test
    uint8_t CAN3_test_msg[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
#endif
    /* Infinite loop */
    for(;;)
    {
        if(IsInit) {
#if CAN3_Test
            CAN_Transmit_Data(&hfdcan3, 0x200, CAN3_test_msg, 8);
#endif

            while (isFDCAN3FIFO0RXOK) {
#if Motor_Open_Loop_Test
                DJI_M2006_Torq_1_to_4[0] = 0.2f;

#endif

#if Motor_Close_Loop_Test

                if (isPluckPIDCalcOK) {
                    DJI_M2006_Torq_1_to_4[0] = PluckPID.getOutput();

                    isPluckPIDCalcOK = false; // Reset the flag after using the calculated output
                }

                DJI_M3508_Torq_5_to_8[2] = FricRPID.getOutput();
                DJI_M3508_Torq_5_to_8[3] = FricLPID.getOutput();
#endif
                DJI_OUTPUT_Cur_MEMCPY(&DJI_M2006_P36_G1, DJI_M2006_Torq_1_to_4, 4);
                DJI_OUTPUT_Cur_MEMCPY(&DJI_M3508_P19_G2, DJI_M3508_Torq_5_to_8, 4);

                DJI_Ctrl_Msg_data_pack(&DJI_M2006_P36_G1, CAN3_0x200_Tx_Data, 8, DJI_M2006_P36_GROUP_1);
                DJI_Ctrl_Msg_data_pack(&DJI_M3508_P19_G2, CAN3_0x1ff_Tx_Data, 8, DJI_M3508_P19_GROUP_2);
                if (CAN_Transmit_Data(&hfdcan3, DJI_M2006_P36_G1.DJI_Ctrl_Msg.tx_ID, CAN3_0x200_Tx_Data, 8) == HAL_OK) {
                    if (CAN_Transmit_Data(&hfdcan3, DJI_M3508_P19_G2.DJI_Ctrl_Msg.tx_ID, CAN3_0x1ff_Tx_Data, 8) == HAL_OK) {
                        isFDCAN3FIFO0RXOK = false;
                    }
                }
            }
        }
        osDelay(1);
        // taskYIELD();
    }
    /* USER CODE END FC_node */
}
