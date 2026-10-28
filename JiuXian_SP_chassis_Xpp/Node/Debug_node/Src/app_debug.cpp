//
// Created by ASUS on 2026/7/23.
//
#include "../Inc/app_debug.h"
#include "usart.h"
#include "dma.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/DJI/Inc/drv_DJI.h"
#include "../../Drv/HIPNUC/Inc/drv_HIPNUC_Hl12S2.h"
#include "../../../Config/Inc/JiuXian_Config.hpp"
#include "../../Applications/Vofa/Inc/app_vofa.hpp"

extern float                  big_yaw_current;
extern bool                   isUART7DMATXOK;
extern bool                   isUART7DMARXOK;
extern bool                   IsChassisSystemInit;
extern DJI_Msg_INFO_t         DJI_GM6020_G1;
extern DJI_Msg_INFO_t         DJI_GM6020_G2;
extern DJI_Msg_INFO_t         DJI_M3508_P19_G1;
extern HIPNUC_data_info_t     HIPNUC_Hl12S2;

extern Drv_UART_Manage_Object UART7_Manage_Object;
extern DMA_HandleTypeDef      hdma_uart7_rx;
extern uint64_t               JiuXuan_System_Tick;
extern uint64_t               debug_node_tick;

/* USER VOFA Class ----------------------------------------------------------------------- */
VOFA::VOFA_Justfloat_Manager RoboMaster_Chassis_Param_Debug_Vofa_JustFloat(&huart7);
VOFA::VOFA_FireWater_Manager RoboMaster_Chassis_Param_Debug_vofa_FireWater(&huart7);

/* VOFA RX INFO Create ------------------------------------------------------------------- */
VOFA_Rx_Info_t               UART7_VOFA_Rx_Info;

/* USER VOFA UI Create ------------------------------------------------------------------- */
const char*                  jiuxuan_steer_id1_pid_kp = "steerId1PIDKp";
VOFA::VOFA_UI_Manager        JiuXuan_Steer_ID1_PID_kp(jiuxuan_steer_id1_pid_kp, SIMPLE_FLOAT_CONTROL, nullptr, &UART7_VOFA_Rx_Info);

const char*                  jiuxuan_chassis_vofa_debug = "chassis";
VOFA::VOFA_UI_Manager        JiuXuan_Chassis_Vofa_Debug(jiuxuan_chassis_vofa_debug, REMOTE_CONTROL, nullptr, &UART7_VOFA_Rx_Info);

/* USER Private variables ---------------------------------------------------------------- */
float test_Vofa[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint32_t vofa_tx_addr_justfloat;
uint32_t vofa_tx_addr_firewater;

void VOFA_UART7_Callback_Function(uint8_t *Buffer, uint16_t Length) {
    /* The Template of Vofa's Sticky Package Handling Method ----------------------------- */
    bool record_name   = false;
    bool record_value  = false;

    uint16_t word_len  = 0;
    uint16_t value_len = 0;

    for (uint16_t i = 0; i < Length; i++) {
        if (i == 0) {
            record_name  = true;
            record_value = false;
        }
        else if (Buffer[i] == ':') {
            memcpy(UART7_VOFA_Rx_Info.name, &Buffer[i - word_len], word_len);
            UART7_VOFA_Rx_Info.name_length  = word_len;

            record_name  = false;
            record_value = false;
        }
        else if (Buffer[i] == '\n') {
            memcpy(UART7_VOFA_Rx_Info.value, &Buffer[i - value_len], value_len);
            UART7_VOFA_Rx_Info.value_length = value_len;

            record_name  = false;
            record_value = false;
        }
        else if (Buffer[i - 1] == '\n'){
            record_name  = true;
            record_value = false;
        }
        else if (Buffer[i - 1] == ':') {
            record_name  = false;
            record_value = true;
        }

        if (record_name)  word_len  += 1;
        else              word_len   = 0;
        if (record_value) value_len += 1;
        else              value_len  = 0;
    }

    UART7_VOFA_Rx_Info.isMemcpyOK = true;
}

/* USER Thread  */
extern "C" void Debug_node(void *argument)
{
    /* USER CODE BEGIN Debug_node */
    UART_Init(&UART7_Manage_Object, &huart7, VOFA_UART7_Callback_Function);

    /* USER VOFA UI -------------------------------------------------------------------------- */
    /* VOFA UI SETTING BEGIN 0 */

    /* VOFA UI SETTING END 0 */

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

    /* Infinite loop */
    for(;;)
    {
        debug_node_tick++;

        if (IsChassisSystemInit) {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

            /* Vofa Show Data Begin ------------------------------------------- */
            RoboMaster_Chassis_Param_Debug_Vofa_JustFloat.Set_Tx_Buffer_Addr(0x30000000);
            vofa_tx_addr_justfloat = RoboMaster_Chassis_Param_Debug_Vofa_JustFloat.Gain_Tx_Buffer_Addr();

            RoboMaster_Chassis_Param_Debug_vofa_FireWater.Set_Tx_Buffer_Addr(0x30000100);
            vofa_tx_addr_firewater = RoboMaster_Chassis_Param_Debug_vofa_FireWater.Gain_Tx_Buffer_Addr();

            if (UART7_VOFA_Rx_Info.isMemcpyOK) {
                if (JiuXuan_Steer_ID1_PID_kp.Set_Param())        UART7_VOFA_Rx_Info.isMemcpyOK = false;
                else if (JiuXuan_Chassis_Vofa_Debug.Set_Param()) UART7_VOFA_Rx_Info.isMemcpyOK = false;
            }

            test_Vofa[0] = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_pos;
            test_Vofa[1] = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel;
            test_Vofa[2] = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor;
            test_Vofa[3] = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp;
            test_Vofa[4] = HIPNUC_Hl12S2.data_frame.gyro_z;
            test_Vofa[5] = big_yaw_current;

            if (isUART7DMATXOK == false) {
                isUART7DMATXOK = true;
                RoboMaster_Chassis_Param_Debug_Vofa_JustFloat.Write_data(test_Vofa, VOFA_SHOW_NUM_6);
            }
            // RoboMaster_Chassis_Param_Debug_vofa_FireWater.Write_data(test_Vofa, VOFA_SHOW_NUM_5);

            memset(UART7_VOFA_Rx_Info.value, 0, UART7_VOFA_Rx_Info.value_length);
            isUART7DMARXOK = false;
            /* Vofa Show Data End --------------------------------------------- */
        }
        // osDelay(1);
        taskYIELD();
    }
    /* USER CODE END Debug_node */
}