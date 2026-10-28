//
// Created by ASUS on 2026/4/4.
//
#include "../Inc/app_it.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "../../../Bsp/FDCAN/Inc/bsp_fdcan.h"
#include "../../../Bsp/RS485/Inc/bsp_rs485.h"
#include "../../../Drv/DJI/Inc/drv_DJI.h"
#include "../../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../../Drv/HIPNUC/Inc/drv_HIPNUC_Hl12S2.h"
#include "../../../Applications/Vofa/Inc/app_vofa.hpp"
#include "../../../Applications/WS2812/Inc/Indicator.h"
#include "../../../Config/Inc/JiuXian_Config.hpp"
#include "../../../Algorithm/PID/Inc/alg_pid.hpp"
#include "../../../Algorithm/Ramp_Planner/Ramp.hpp"
#include "../../../Algorithm/SMC/Inc/alg_smc.hpp"
#include "../../../Algorithm/S_Curve/Inc/alg_s_curve.hpp"

#if USE_DMA_DOUBLE_BUFFER
extern Drv_UART_Manage_Object   USART3_Manage_Object;
extern Drv_UART_Manage_Object   UART7_Manage_Object;
extern Drv_UART_Manage_Object   UART10_Manage_Object;
#endif

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;
extern Struct_RS485_Manage_Object            USART3_RS485_Manage_Object;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern DJI_Msg_INFO_t           DJI_GM6020_G1;
extern DJI_Msg_INFO_t           DJI_GM6020_G2;
extern DJI_Msg_INFO_t           DJI_M3508_P19_G1;

extern HIPNUC_data_info_t       HIPNUC_Hl12S2;

#if PID_METHOD
extern PIDControllerV2          Steer_ID1_PID;
extern PIDControllerV2          Steer_ID2_PID;
extern PIDControllerV2          Steer_ID3_PID;
extern PIDControllerV2          Steer_ID4_PID;
#endif

extern PIDController            Chassis_Angle_PID;
extern PIDController            Chassis_Omega_PID;

extern PIDController            JiuXuan_Big_Yaw_Spd_PID;
extern PIDController            JiuXuan_Big_Yaw_Cur_PID;

extern PIDController            Wheel_ID1_PID;
extern PIDController            Wheel_ID2_PID;
extern PIDController            Wheel_ID3_PID;
extern PIDController            Wheel_ID4_PID;

#if SMC_METHOD
extern SMC                      Steer_ID1_SMC;
extern SMC                      Steer_ID2_SMC;
extern SMC                      Steer_ID3_SMC;
extern SMC                      Steer_ID4_SMC;
#endif

extern Chassis_Config           JiuXuan_Chassis;

extern SCurvePlanner            JiuXuan_Chassis_Speed_Curve;
extern Ramp_Planner             JiuXuan_Chassis_Speed_x_Ramp;
extern Ramp_Planner             JiuXuan_Chassis_Speed_y_Ramp;
extern Ramp_Planner             JiuXuan_Chassis_Omega_Ramp;

extern bool                     isFDCAN1FIFO0RXOK;
extern bool                     isFDCAN2FIFO0RXOK;
extern bool                     isFDCAN3FIFO0RXOK;

extern bool                     isUASRT3DMARXOK;
extern bool                     isUART7DMATXOK;
extern bool                     isUART7DMARXOK;
extern bool                     isUSART10DMARXOK;

#if USE_DMA_IDLE_RX
extern uint8_t                  uart8_uart_test_buffer[256];
extern uint8_t                  uart9_uart_test_buffer[256];
extern uint8_t                  uart10_rx_test_buffer[256];
#endif

extern uint16_t                 Gx_Tick;

extern float                    Wheel_speed_radps[4];
extern float                    Steer_angle[4];
extern float                    chassis_imu_init_angle_yaw;
extern float                    big_yaw_current;

extern uint64_t                 JiuXuan_System_Tick;
extern uint64_t                 Sign_Tick;
extern uint64_t                 ctrl_node_tick;
extern uint64_t                 debug_node_tick;
extern uint64_t                 imu_node_tick;
extern uint64_t                 init_node_tick;
extern uint64_t                 interaction_node_tick;

extern uint64_t                 ctrl_node_freq;
extern uint64_t                 debug_node_freq;
extern uint64_t                 imu_node_freq;
extern uint64_t                 init_node_freq;
extern uint64_t                 interaction_node_freq;

extern DMA_HandleTypeDef        hdma_usart3_rx;
extern DMA_HandleTypeDef        hdma_uart7_rx;
extern DMA_HandleTypeDef        hdma_uart8_rx;
extern DMA_HandleTypeDef        hdma_uart9_rx;
extern DMA_HandleTypeDef        hdma_usart10_rx;

extern "C" {
    /**
     * @brief HAL库CAN接收FIFO0中断
     *
     * @param hfdcan CAN编号
     */
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
        if(hfdcan->Instance == FDCAN1)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer) == HAL_OK){
                DJI_GM6020_FeedBack_Msg_PackingCallBack(CAN1_Manage_Object.Rx_Buffer, 8, CAN1_Manage_Object.Rx_Header.Identifier);

                isFDCAN1FIFO0RXOK = true;
            }
        }
        else if(hfdcan->Instance == FDCAN2)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer_FDCAN) == HAL_OK){
                memcpy(&Sentry_Chassis_Gimbal_Interact_Info.rx_cmd, CAN2_Manage_Object.Rx_Buffer_FDCAN, sizeof(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd));

                isFDCAN2FIFO0RXOK = true;
            }
        }
        else if(hfdcan->Instance == FDCAN3)
        {
            while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer) == HAL_OK){
                DJI_M3508_P19_FeedBack_Msg_UpackingCallBack(CAN3_Manage_Object.Rx_Buffer, 8, CAN3_Manage_Object.Rx_Header.Identifier);

                isFDCAN3FIFO0RXOK = true;
            }
        }
    }

    /**
     * @brief HAL库CAN错误中断
     *
     * @param hfdcan CAN编号
     * @param ErrorStatusITs 错误状态
     */
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
    {
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
        {
            isFDCAN1FIFO0RXOK = false;
            isFDCAN2FIFO0RXOK = false;
            isFDCAN3FIFO0RXOK = false;
            // CAN总线离线, 重新启动CAN
            HAL_FDCAN_Stop(hfdcan);
            HAL_FDCAN_Start(hfdcan);
        }
    }

    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
        if (huart->Instance == USART3) {
            // RS485_RX(&USART3_RS485_Manage_Object);
        } else if (huart->Instance == UART7) {
            isUART7DMATXOK = false;
        } else {
            return;
        }
    }

#if USE_DMA_IDLE_RX
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        if (huart->Instance == USART3) {
            USART3_Manage_Object.Rx_Buffer_Already = USART3_Manage_Object.Rx_Buffer_Active;
            if (USART3_Manage_Object.Rx_Buffer_Active == USART3_Manage_Object.Rx_Buffer_A) {
                USART3_Manage_Object.Rx_Buffer_Active = USART3_Manage_Object.Rx_Buffer_B;
            }else {
                USART3_Manage_Object.Rx_Buffer_Active = USART3_Manage_Object.Rx_Buffer_A;
            }

            USART3_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

            if (USART3_Manage_Object.Callback_Function != nullptr) {
                USART3_Manage_Object.Callback_Function(USART3_Manage_Object.Rx_Buffer_Already, Size);
            }

            isUASRT3DMARXOK = true;

        }else if (huart->Instance == UART7) {
            UART7_Manage_Object.Rx_Buffer_Already = UART7_Manage_Object.Rx_Buffer_Active;
            if (UART7_Manage_Object.Rx_Buffer_Active == UART7_Manage_Object.Rx_Buffer_A) {
                UART7_Manage_Object.Rx_Buffer_Active = UART7_Manage_Object.Rx_Buffer_B;
            }else {
                UART7_Manage_Object.Rx_Buffer_Active = UART7_Manage_Object.Rx_Buffer_A;
            }

            UART7_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

            if (UART7_Manage_Object.Callback_Function != nullptr) {
                UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer_Already, Size);
            }

            isUART7DMARXOK = true;
        }
        else if (huart->Instance == UART8)
        {
            HAL_UART_Transmit_DMA(&huart8, uart8_uart_test_buffer, Size);

            HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_uart_test_buffer, 256);
            __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
        }
        else if (huart->Instance == UART9)
        {
            HAL_UART_Transmit_DMA(&huart9, uart9_uart_test_buffer, Size);

            HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_uart_test_buffer, 256);
            __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
        }
        else if (huart->Instance == USART10)
        {
            UART10_Manage_Object.Rx_Buffer_Already = UART10_Manage_Object.Rx_Buffer_Active;
            if (UART10_Manage_Object.Rx_Buffer_Active == UART10_Manage_Object.Rx_Buffer_A) {
                UART10_Manage_Object.Rx_Buffer_Active = UART10_Manage_Object.Rx_Buffer_B;
            }else {
                UART10_Manage_Object.Rx_Buffer_Active = UART10_Manage_Object.Rx_Buffer_A;
            }

            UART10_Manage_Object.Rx_Time_Stamp = JiuXuan_System_Tick;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT); // Disable Half Transfer interrupt for DMA

            if (UART10_Manage_Object.Callback_Function != nullptr) {
                UART10_Manage_Object.Callback_Function(UART10_Manage_Object.Rx_Buffer_Already, Size);
            }

            isUSART10DMARXOK = true;
        }
    }
#elif USE_IT_RX
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART10)
        {

        }
    }
#endif

    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART3) {
            // 清所有错误标志
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_RTOF);
            huart->ErrorCode = HAL_UART_ERROR_NONE;

            UART_Receive_DMA_Repeat(&USART3_Manage_Object);
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
        }
        else if (huart->Instance == UART7) {
            // 清所有错误标志
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_RTOF);
            huart->ErrorCode = HAL_UART_ERROR_NONE;

            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
        }
        else if (huart->Instance == UART8) {

        }
        else if (huart->Instance == UART9) {

        }
        else if (huart->Instance == USART10) {
            // 清所有错误标志
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_RTOF);
            huart->ErrorCode = HAL_UART_ERROR_NONE;

            // 重启 DMA 接收
            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
        }
    }
}

void TIM1_PeriodElapsedCallback() {
    JiuXuan_System_Tick++;

    /* Counting Connect Tick */
    DJI_GM6020_G1.DJI_FeedBack_Msg[0].connect_tick++;
    DJI_GM6020_G1.DJI_FeedBack_Msg[1].connect_tick++;
    DJI_GM6020_G1.DJI_FeedBack_Msg[2].connect_tick++;
    DJI_GM6020_G1.DJI_FeedBack_Msg[3].connect_tick++;

    DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].connect_tick++;
    DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].connect_tick++;
    DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].connect_tick++;
    DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].connect_tick++;

    /* Connect Timeout */
    if (DJI_GM6020_G1.DJI_FeedBack_Msg[0].connect_tick == 500) DJI_GM6020_G1.DJI_FeedBack_Msg[0].connect_tick = 500;
    if (DJI_GM6020_G1.DJI_FeedBack_Msg[1].connect_tick == 500) DJI_GM6020_G1.DJI_FeedBack_Msg[1].connect_tick = 500;
    if (DJI_GM6020_G1.DJI_FeedBack_Msg[2].connect_tick == 500) DJI_GM6020_G1.DJI_FeedBack_Msg[2].connect_tick = 500;
    if (DJI_GM6020_G1.DJI_FeedBack_Msg[3].connect_tick == 500) DJI_GM6020_G1.DJI_FeedBack_Msg[3].connect_tick = 500;

    if (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].connect_tick == 500) DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].connect_tick = 500;
    if (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].connect_tick == 500) DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].connect_tick = 500;
    if (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].connect_tick == 500) DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].connect_tick = 500;
    if (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].connect_tick == 500) DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].connect_tick = 500;

#if PID_METHOD
    Steer_ID1_PID.update(0, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel);
    Steer_ID2_PID.update(0, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel);
    Steer_ID3_PID.update(0, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel);
    Steer_ID4_PID.update(0, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel);
#endif

#if SMC_METHOD
    Steer_ID1_SMC.ref = STEER_TRANS(Steer_angle[0], STEER_ID1_INIT);
    Steer_ID2_SMC.ref = STEER_TRANS(Steer_angle[1], STEER_ID2_INIT);
    Steer_ID3_SMC.ref = STEER_TRANS(Steer_angle[2], STEER_ID3_INIT);
    Steer_ID4_SMC.ref = STEER_TRANS(Steer_angle[3], STEER_ID4_INIT);

    Steer_ID1_SMC.IsSteerUseOptimalAngle = true;
    Steer_ID2_SMC.IsSteerUseOptimalAngle = true;
    Steer_ID3_SMC.IsSteerUseOptimalAngle = true;
    Steer_ID4_SMC.IsSteerUseOptimalAngle = true;

    /**
     * @brief 美术效果姿势（初始姿态）
     */
    if (JiuXuan_Chassis.Get_V_x_Cmd() == 0.0f && JiuXuan_Chassis.Get_V_y_Cmd() == 0.0f && JiuXuan_Chassis.Get_Omega_Cmd() == 0.0f) {
        Steer_ID1_SMC.ref = STEER_TRANS(135.0f, STEER_ID1_INIT - 90.0f);
        Steer_ID2_SMC.ref = STEER_TRANS( 45.0f, STEER_ID2_INIT - 90.0f);
        Steer_ID3_SMC.ref = STEER_TRANS(135.0f, STEER_ID3_INIT - 90.0f);
        Steer_ID4_SMC.ref = STEER_TRANS( 45.0f, STEER_ID4_INIT - 90.0f);

        Steer_ID1_SMC.IsSteerUseOptimalAngle = false;
        Steer_ID2_SMC.IsSteerUseOptimalAngle = false;
        Steer_ID3_SMC.IsSteerUseOptimalAngle = false;
        Steer_ID4_SMC.IsSteerUseOptimalAngle = false;
    }

    Steer_ID1_SMC.SMC_Update(DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel);
    Steer_ID2_SMC.SMC_Update(DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel);
    Steer_ID3_SMC.SMC_Update(DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel);
    Steer_ID4_SMC.SMC_Update(DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel);
#endif

    float steer_angle_id1_err = Steer_ID1_SMC.GetErr() * M_PI / 180.0f;
    if (fabs(steer_angle_id1_err) > M_PI / 2.0f) steer_angle_id1_err = (steer_angle_id1_err > 0 ? M_PI / 2.0f : -M_PI / 2.0f);
    else if (fabs(steer_angle_id1_err) < 0.05) steer_angle_id1_err = 0.0f;
    float cosK1 = cosf(steer_angle_id1_err) * cosf(steer_angle_id1_err) * cosf(steer_angle_id1_err);

    float steer_angle_id2_err = Steer_ID2_SMC.GetErr() * M_PI / 180.0f;
    if (fabs(steer_angle_id2_err) > M_PI / 2.0f) steer_angle_id2_err = (steer_angle_id2_err > 0 ? M_PI / 2.0f : -M_PI / 2.0f);
    else if (fabs(steer_angle_id2_err) < 0.05) steer_angle_id2_err = 0.0f;
    float cosK2 = cosf(steer_angle_id2_err) * cosf(steer_angle_id2_err) * cosf(steer_angle_id2_err);

    float steer_angle_id3_err = Steer_ID3_SMC.GetErr() * M_PI / 180.0f;
    if (fabs(steer_angle_id3_err) > M_PI / 2.0f) steer_angle_id3_err = (steer_angle_id3_err > 0 ? M_PI / 2.0f : -M_PI / 2.0f);
    else if (fabs(steer_angle_id3_err) < 0.05) steer_angle_id3_err = 0.0f;
    float cosK3 = cosf(steer_angle_id3_err) * cosf(steer_angle_id3_err) * cosf(steer_angle_id3_err);

    float steer_angle_id4_err = Steer_ID4_SMC.GetErr() * M_PI / 180.0f;
    if (fabs(steer_angle_id4_err) > M_PI / 2.0f) steer_angle_id4_err = (steer_angle_id4_err > 0 ? M_PI / 2.0f : -M_PI / 2.0f);
    else if (fabs(steer_angle_id4_err) < 0.05) steer_angle_id4_err = 0.0f;
    float cosK4 = cosf(steer_angle_id4_err) * cosf(steer_angle_id4_err) * cosf(steer_angle_id4_err);

    // JiuXuan_Big_Yaw_Spd_PID.update(0.5, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel - JiuXuan_Chassis.omega_fdb_);
    JiuXuan_Big_Yaw_Cur_PID.update(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.set_big_yaw_controller_cmd, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor);

    Wheel_ID1_PID.update(Steer_ID1_SMC.wheel_dir * WHEEL_TRANS(Wheel_speed_radps[0], 1) * cosK1, DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_vel);
    Wheel_ID2_PID.update(Steer_ID2_SMC.wheel_dir * WHEEL_TRANS(Wheel_speed_radps[1], 2) * cosK2, DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_vel);
    Wheel_ID3_PID.update(Steer_ID3_SMC.wheel_dir * WHEEL_TRANS(Wheel_speed_radps[2], 3) * cosK3, DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_vel);
    Wheel_ID4_PID.update(Steer_ID4_SMC.wheel_dir * WHEEL_TRANS(Wheel_speed_radps[3], 4) * cosK4, DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_vel);

}

void TIM3_PeriodElapsedCallback() {
    Sign_Tick++;

    Chassis_Angle_PID.update(IMU_TRANS(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.ginbal_yaw, chassis_imu_init_angle_yaw), HIPNUC_Hl12S2.data_frame.yaw);

    JiuXuan_Chassis.Set_V_x(Sentry_Chassis_Gimbal_Interact_Info.chassis_vel_x_cmd);
    JiuXuan_Chassis.Set_V_y(Sentry_Chassis_Gimbal_Interact_Info.chassis_vel_y_cmd);

    if (Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_ == 0) {
        /**
         * @brief 具有死区限制的底盘位置环PID
         */
        if (fabs(Chassis_Angle_PID.getError()) < 1.0f) {
            JiuXuan_Chassis.Set_Omega(0.0f);
        }else {
            JiuXuan_Chassis.Set_Omega(Chassis_Angle_PID.getOutput());
        }
    }else {
        JiuXuan_Chassis.Set_Omega(Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_);
    }

    JiuXuan_Chassis.Chassis_Kinematics_Calculation();

    JiuXuan_Chassis.Get_Wheel_Speed(DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_vel,
                                    DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_vel,
                                    DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_vel,
                                    DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_vel);

    JiuXuan_Chassis.Get_Steer_Angle(DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos,
                                    DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos,
                                    DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos,
                                    DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos);

    JiuXuan_Chassis.Chassis_Inverse_Kinematics_Calculation();

    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.fdb_vel_x       = JiuXuan_Chassis.v_fdb_x_;
    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.fdb_vel_y       = JiuXuan_Chassis.v_fdb_y_;
    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.fdb_omega       = JiuXuan_Chassis.omega_fdb_;

    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.big_yaw_angle   = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_pos;
    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.big_yaw_speed   = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel;
    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.big_yaw_current = DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_cur;

    if (Sign_Tick > 10) {
        Sign_Tick = 0;

        Normal_Breathing();
    }


    if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 50   == 0) {
        HAL_GPIO_TogglePin(Sign_LED_GPIO_Port, Sign_LED_Pin);
    }

    if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 100  == 0) {

    }

    if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 200  == 0) {
        Gx_Tick++;
        if (Gx_Tick > 3) Gx_Tick = 0;
    }

    if (JiuXuan_System_Tick != 0 && JiuXuan_System_Tick % 1000 == 0) {
        ctrl_node_freq        = ctrl_node_tick;
        debug_node_freq       = debug_node_tick;
        imu_node_freq         = imu_node_tick;
        init_node_freq        = init_node_tick;
        interaction_node_freq = interaction_node_tick;

        ctrl_node_tick        = 0;
        debug_node_tick       = 0;
        imu_node_tick         = 0;
        init_node_tick        = 0;
        interaction_node_tick = 0;
    }

    switch (Gx_Tick) {
        case 1 : {
            HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_SET);
            break;
        }
        case 2 : {
            HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
            break;
        }
        case 3 : {
            HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_SET);
            break;
        }
        default : {
            HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_RESET);
            break;
        }
    }
}
