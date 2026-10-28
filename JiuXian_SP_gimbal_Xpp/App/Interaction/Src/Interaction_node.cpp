//
// Created by ASUS on 2026/7/15.
//
#include "../Inc/Interaction_node.h"
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Algorithm/MathUnit/Inc/MathUnit.hpp"

#define FDCAN_test 0
#define DATA_Transform_test 0

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;
extern bool                                  isFDCAN2FIFO0RXOK;

#if FDCAN_test
uint8_t fdcan_test_msg[64] = {
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77
};
#endif

#if DATA_Transform_test
float   test_data        = -0.5f;
uint16_t test_data_int    =   0 ;
float   data_float_final = 0.0f;
#endif

extern bool IsInit;

extern "C" void Interaction_node(void *argument)
{
    /* USER CODE BEGIN Interaction_node */
    /* Infinite loop */
    for(;;)
    {
        if (IsInit) {

#if DATA_Transform_test
            test_data_int = float_to_uint(test_data, -1.0f, 1.0f, 16);
            data_float_final = uint_to_float(test_data_int, -1.0f, 1.0f, 16);
#endif

#if FDCAN_test
            FDCAN_Transmit_Data(&hfdcan2, 0x283, Sentry_Chassis_Gimbal_Interact_Info.tx_cmd_msg, 64);
#endif
            if (isFDCAN2FIFO0RXOK) {
                FDCAN_Transmit_Data(&hfdcan2, 0x283, Sentry_Chassis_Gimbal_Interact_Info.tx_cmd_msg, 64);

                isFDCAN2FIFO0RXOK = false;
            }
        }
        osDelay(1);
        // taskYIELD();
    }
    /* USER CODE END Interaction_node */
}
