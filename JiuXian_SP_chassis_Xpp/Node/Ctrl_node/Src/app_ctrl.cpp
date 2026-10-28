//
// Created by ASUS on 2026/4/4.
//
#include "../Inc/app_ctrl.h"
#include "../../../Bsp/FDCAN/Inc/bsp_fdcan.h"
#include "../../../Bsp/RS485/Inc/bsp_rs485.h"
#include "../../../Drv/DJI/Inc/drv_DJI.h"
#include "../../../Drv/XiaoHe/Inc/drv_XiaoHe.h"
#include "../../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../../Algorithm/PID/Inc/alg_pid.hpp"
#include "../../../Algorithm/SMC/Inc/alg_smc.hpp"
#include "../../../Config/Inc/JiuXian_Config.hpp"
#include "dma.h"

#define Motor_Calibration       0
#define PID_Test                0

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;
extern Struct_RS485_Manage_Object            USART3_RS485_Manage_Object;

extern DMA_HandleTypeDef        hdma_usart3_rx;

extern float                    DJI_M2006_Torq_1_to_4[4];
extern float                    DJI_M2006_Torq_5_to_8[4];
extern float                    DJI_M3508_Torq_1_to_4[4];
extern float                    DJI_M3508_Torq_5_to_8[4];
extern float                    DJI_GM6020_Torq_1_to_4[4];
extern float                    DJI_GM6020_Torq_5_to_7[4];
extern float                    DJI_GM6020_Vol_1_to_4[4];
extern float                    DJI_GM6020_Vol_5_to_7[4];

extern float                    Wheel_speed_radps[4];
extern float                    Steer_angle[4];

extern float                    big_yaw_current;

extern DJI_Msg_INFO_t           DJI_M2006_P36_G1;
extern DJI_Msg_INFO_t           DJI_M2006_P36_G2;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G1;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G2;
extern DJI_Msg_INFO_t           DJI_GM6020_G1;
extern DJI_Msg_INFO_t           DJI_GM6020_G2;

extern XiaoHE_Msg_INFO_t        XiaoHE[8];

extern Drv_UART_Manage_Object   USART3_Manage_Object;

extern bool                     ALL_FDCAN_INIT_OK;

extern bool                     isFDCAN1FIFO0RXOK;
extern bool                     isFDCAN3FIFO0RXOK;

extern bool                     isUASRT3DMARXOK;

extern uint8_t                  usart3_uart_txbuffer[256];

extern uint64_t                 ctrl_node_tick;

extern Chassis_Config           JiuXuan_Chassis;

#if PID_Test
PIDController testPIDGM6020(0.007, 0, 0, 0, 0.5, 1.5, true);
PIDController testPIDM3508(0.007, 0, 0, 0, 0.5, 1.5, false);
#endif

#if PID_METHOD

PIDControllerV2 Steer_ID1_PID(8.5, 0.5, 0.00, 0.03, 0, 0.02, 0.741, 1.0, 0.5, 10.0, 1.4);
PIDControllerV2 Steer_ID2_PID(8.5, 0.5, 0.00, 0.03, 0, 0.02, 0.741, 1.0, 0.5, 10.0, 1.4);
PIDControllerV2 Steer_ID3_PID(8.5, 0.5, 0.00, 0.03, 0, 0.02, 0.741, 1.0, 0.5, 10.0, 1.4);
PIDControllerV2 Steer_ID4_PID(8.5, 0.5, 0.00, 0.03, 0, 0.02, 0.741, 1.0, 0.5, 10.0, 1.4);

#endif

/**
 * @brief Chassis PID param
 */
PIDController   Chassis_Angle_PID(0.045, 0.0f, 0.00, 0, 0.005 * CMD_MAX_OMEGA, 0.05 * CMD_MAX_OMEGA, true);

/**
 * @brief Big yaw PID param
 */
// PIDController   JiuXuan_Big_Yaw_Pos_PID()
PIDController   JiuXuan_Big_Yaw_Spd_PID(0.007, 0.008, 0.0, 0.0, 0.3, 3.0, false);
PIDController   JiuXuan_Big_Yaw_Cur_PID(8.55, 1.6, 0.000, 0.0, 4.5, 23.8, false);

/**
 * @brief Wheel PID param
 */
PIDController   Wheel_ID1_PID(0.5, 0.001, 0.1, 0, 0.5, 1.2, false);
PIDController   Wheel_ID2_PID(0.5, 0.001, 0.1, 0, 0.5, 1.2, false);
PIDController   Wheel_ID3_PID(0.5, 0.001, 0.1, 0, 0.5, 1.2, false);
PIDController   Wheel_ID4_PID(0.5, 0.001, 0.1, 0, 0.5, 1.2, false);

#if SMC_METHOD

SMC             Steer_ID1_SMC(2.5, 4.2, 0, 0.001, 1.3, 0.0024, 0.5);
SMC             Steer_ID2_SMC(2.5, 4.2, 0, 0.001, 1.3, 0.0024, 0.5);
SMC             Steer_ID3_SMC(2.5, 4.2, 0, 0.001, 1.3, 0.0024, 0.5);
SMC             Steer_ID4_SMC(2.5, 4.2, 0, 0.001, 1.3, 0.0024, 0.5);

#endif



void Data_Process_of_Power_Meter(uint8_t *Buffer, uint16_t Length) {
    XiaoHE_Data_Process(Buffer, Length);
}

/**
* @brief Function implementing the CtrlTask thread.
* @param argument: Not used
* @retval None
*/
extern "C" void appCtrl(void *argument)
{
    /* USER CODE BEGIN appCtrl */
    uint64_t rs485_tick        = 0;
    uint8_t  power_meter_id_tx = 0;

    fdcan_classic_filter_init(&hfdcan1);
    fdcan_filter_init(&hfdcan2);
    fdcan_classic_filter_init(&hfdcan3);
    XiaoHE_INIT();
    DJI_INIT();
    BSP_RS485_Init(&USART3_RS485_Manage_Object, &huart3, GPIOB, GPIO_PIN_14, TX_DMA, true);
    UART_Init(&USART3_Manage_Object, &huart3, Data_Process_of_Power_Meter);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // Disable Half Transfer interrupt for DMA

    ALL_FDCAN_INIT_OK = true;
    /* Infinite loop */
    for(;;)
    {
        rs485_tick++;
        ctrl_node_tick++;

#if XIAOHE_DATA_TEST
        if (rs485_tick == 4) {
            rs485_tick = 0;
            switch (power_meter_id_tx) {
                case 0:
                    XiaoHE_Tx_Test_Read_Req(1);
                    break;
                case 1:
                    XiaoHE_Tx_Test_Read_Req(2);
                    break;
                case 2:
                    XiaoHE_Tx_Test_Read_Req(3);
                    break;
                case 3:
                    XiaoHE_Tx_Test_Read_Req(4);
                    break;
                default:
                    power_meter_id_tx = 0;
                    break;
            }
            memcpy(usart3_uart_txbuffer, XiaoHE[power_meter_id_tx++].tx_requset_msg, REQUEST_MSG_SIZE);
            BSP_RS485_Send(&USART3_RS485_Manage_Object, usart3_uart_txbuffer, REQUEST_MSG_SIZE);
        }
#else

        if (rs485_tick > 2) {
            rs485_tick = 0;
            switch (power_meter_id_tx) {
                case 0:
                    XiaoHE_Read_Cmd(1, Read_All);
                    // if (XiaoHE[0].id_fdb == 0x01) power_meter_id_tx++;
                    if (XiaoHE[0].is_Connected) power_meter_id_tx++;
                    break;
                case 1:
                    XiaoHE_Read_Cmd(2, Read_All);
                    // if (XiaoHE[1].id_fdb == 0x02) power_meter_id_tx++;
                    if (XiaoHE[1].is_Connected) power_meter_id_tx++;
                    break;
                case 2:
                    XiaoHE_Read_Cmd(3, Read_All);
                    // if (XiaoHE[2].id_fdb == 0x03) power_meter_id_tx++;
                    if (XiaoHE[2].is_Connected) power_meter_id_tx++;
                    break;
                case 3:
                    XiaoHE_Read_Cmd(4, Read_All);
                    // if (XiaoHE[3].id_fdb == 0x04) power_meter_id_tx++;
                    if (XiaoHE[3].is_Connected) power_meter_id_tx++;
                    break;
                default:
                    power_meter_id_tx = 0;
                    break;
            }
            memcpy(usart3_uart_txbuffer, XiaoHE[power_meter_id_tx].tx_requset_msg, REQUEST_MSG_SIZE);
            BSP_RS485_Send(&USART3_RS485_Manage_Object, usart3_uart_txbuffer, REQUEST_MSG_SIZE);
        }

#endif

        while (isFDCAN1FIFO0RXOK) {

            Steer_angle[0] = JiuXuan_Chassis.steer_angle_ref_[0] * 180.0f / M_PI;
            Steer_angle[1] = JiuXuan_Chassis.steer_angle_ref_[1] * 180.0f / M_PI;
            Steer_angle[2] = JiuXuan_Chassis.steer_angle_ref_[2] * 180.0f / M_PI;
            Steer_angle[3] = JiuXuan_Chassis.steer_angle_ref_[3] * 180.0f / M_PI;

            #if Motor_Calibration
            DJI_GM6020_Torq_1_to_4[0] =  0.0f;
            DJI_GM6020_Torq_1_to_4[1] =  0.0f;
            DJI_GM6020_Torq_1_to_4[2] =  0.0f;
            DJI_GM6020_Torq_1_to_4[3] =  0.0f;

            DJI_GM6020_Torq_5_to_7[0] =  0.0f;
            #else
            /* set torque to GM6020(steering motor) */

            #if PID_Test
            if (testPIDGM6020.update(0, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos)) {
                DJI_GM6020_Torq_1_to_4[0] = testPIDGM6020.getOutput();
            }
            #else
            big_yaw_current = 0.4;

            /* Sterr Motor Ctrl Msg */
            DJI_GM6020_Torq_1_to_4[0] =  Steer_ID1_SMC.u;
            DJI_GM6020_Torq_1_to_4[1] =  Steer_ID2_SMC.u;
            DJI_GM6020_Torq_1_to_4[2] =  Steer_ID3_SMC.u;
            DJI_GM6020_Torq_1_to_4[3] =  Steer_ID4_SMC.u;

            /* Big Yaw Motor Ctrl Msg */
            DJI_GM6020_Torq_5_to_7[0] =  Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.set_big_yaw_controller_cmd;
            // DJI_GM6020_Torq_5_to_7[0] =  JiuXuan_Big_Yaw_Cur_PID.getOutput();
            DJI_GM6020_Torq_5_to_7[1] =  DATA_RESERVE_f;
            DJI_GM6020_Torq_5_to_7[2] =  DATA_RESERVE_f;
            DJI_GM6020_Torq_5_to_7[3] =  DATA_RESERVE_f;
            #endif

            for (uint8_t i = 0; i < 4; i++) {
                if (DJI_GM6020_G1.DJI_FeedBack_Msg[i].connect_tick > 450) DJI_GM6020_Torq_1_to_4[i] = 0.0f;
            }

            #endif
            DJI_OUTPUT_Cur_MEMCPY(&DJI_GM6020_G1, DJI_GM6020_Torq_1_to_4, 4);
            DJI_OUTPUT_Vol_MEMCPY(&DJI_GM6020_G2, DJI_GM6020_Torq_5_to_7, 4);
            /* GM6020 data sent */
            DJI_Ctrl_Msg_data_pack(&DJI_GM6020_G1, CAN1_0x1fe_Tx_Data, 8, DJI_GM6020_Cur_GROUP_1);
            DJI_Ctrl_Msg_data_pack(&DJI_GM6020_G2, CAN1_0x2ff_Tx_Data, 8, DJI_GM6020_Vol_GROUP_2);
            if(CAN_Transmit_Data(&hfdcan1,DJI_GM6020_G1.DJI_Ctrl_Msg.tx_ID,CAN1_0x1fe_Tx_Data,8) == HAL_OK) {
                if (CAN_Transmit_Data(&hfdcan1, DJI_GM6020_G2.DJI_Ctrl_Msg.tx_ID, CAN1_0x2ff_Tx_Data, 8) == HAL_OK) {
                    isFDCAN1FIFO0RXOK = false;
                }
            }
        }

        while (isFDCAN3FIFO0RXOK) {

            Wheel_speed_radps[0] = JiuXuan_Chassis.wheel_speed_ref_[0] / 0.058f;
            Wheel_speed_radps[1] = JiuXuan_Chassis.wheel_speed_ref_[1] / 0.058f;
            Wheel_speed_radps[2] = JiuXuan_Chassis.wheel_speed_ref_[2] / 0.058f;
            Wheel_speed_radps[3] = JiuXuan_Chassis.wheel_speed_ref_[3] / 0.058f;

            #if Motor_Calibration
            DJI_M3508_Torq_1_to_4[0]  =  0.0f;
            DJI_M3508_Torq_1_to_4[1]  =  0.0f;
            DJI_M3508_Torq_1_to_4[2]  =  0.0f;
            DJI_M3508_Torq_1_to_4[3]  =  0.0f;
            #else
            /* set torque to M3508(Wheel motor) */

            DJI_M3508_Torq_1_to_4[0]  =  Wheel_ID1_PID.getOutput();
            DJI_M3508_Torq_1_to_4[1]  =  Wheel_ID2_PID.getOutput();
            DJI_M3508_Torq_1_to_4[2]  =  Wheel_ID3_PID.getOutput();
            DJI_M3508_Torq_1_to_4[3]  =  Wheel_ID4_PID.getOutput();
            #endif

            for (uint8_t i = 0; i < 4; i++) {
                if (DJI_M3508_P19_G1.DJI_FeedBack_Msg[i].connect_tick > 450) DJI_M3508_Torq_1_to_4[i] = 0.0f;
            }

            DJI_OUTPUT_Cur_MEMCPY(&DJI_M3508_P19_G1, DJI_M3508_Torq_1_to_4, 4);
            /* M3508 data sent */
            DJI_Ctrl_Msg_data_pack(&DJI_M3508_P19_G1, CAN3_0x200_Tx_Data, 8, DJI_M3508_P19_GROUP_1);
            if (CAN_Transmit_Data(&hfdcan3, DJI_M3508_P19_G1.DJI_Ctrl_Msg.tx_ID, CAN3_0x200_Tx_Data, 8) == HAL_OK) {
                isFDCAN3FIFO0RXOK = false;
            }
        }

        UART_Receive_DMA_Repeat(&USART3_Manage_Object);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // Disable Half Transfer interrupt for DMA

        // taskYIELD();
        osDelay(1);
    }
    /* USER CODE END appCtrl */
}
