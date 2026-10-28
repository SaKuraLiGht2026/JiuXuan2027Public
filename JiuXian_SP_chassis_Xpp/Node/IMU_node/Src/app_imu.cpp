//
// Created by ASUS on 2026/7/19.
//
#include "../Inc/app_imu.h"
#include <usart.h>
#include <dma.h>
#include "../../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../../Drv/HIPNUC/Inc/drv_HIPNUC_Hl12S2.h"
#include "../../../Config/Inc/JiuXian_Config.hpp"

#define UART10_TEST             0
#define UART10_RX_TEST          0
#define Double_Buffer_Echo_Test 0

extern bool                   isUSART10DMA0RXOK;
extern DMA_HandleTypeDef      hdma_usart10_rx;
extern Drv_UART_Manage_Object UART10_Manage_Object;
extern HIPNUC_data_info_t     HIPNUC_Hl12S2;

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;

HAL_StatusTypeDef             uart_rx_check;

#if USE_DMA_IDLE_RX
extern uint8_t                uart10_rx_test_buffer[256];
#endif

extern uint64_t               imu_node_tick;
extern uint64_t               imu_isr_Tick;

extern float                  chassis_imu_init_angle_yaw;

#if UART10_TEST
__attribute__((section(".dma_buffer"))) uint8_t uart10_dma_test_msg[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
HAL_StatusTypeDef UART10_test_Status;
#endif

void HIPNUC_UART_Callback_Function(uint8_t *Buffer, uint16_t Length) {
#if Double_Buffer_Echo_Test
    HAL_UART_Transmit_DMA(&huart10, Buffer, Length);
#endif
    HIPNUC_Hl12S2_UART_Hl91_DATA_Process(&HIPNUC_Hl12S2, Buffer, Length);
    if (imu_isr_Tick == 0) {
        chassis_imu_init_angle_yaw = HIPNUC_Hl12S2.data_frame.yaw;
    }
    imu_isr_Tick++;
}

extern "C" void imuTask(void *argument)
{
    /* USER CODE BEGIN imuTask */

#if USE_DMA_IDLE_RX

#if USE_DMA_DOUBLE_BUFFER
    UART_Init(&UART10_Manage_Object, &huart10, HIPNUC_UART_Callback_Function);
#endif

#if UART10_RX_TEST
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10_rx_test_buffer, 256); // Start UART10 reception with DMA and IDLE interrupt
#else

#endif

#elif USE_IT_RX

#if UART10_RX_TEST
    HAL_UART_Receive_IT(&huart10, uart10_rx_test_buffer, 256); // Start UART10 reception with interrupt
#else
    HAL_UART_Receive_IT(&huart10, uart10_dma_test_msg, 8); // Start UART10 reception with interrupt
#endif

#else

#endif

    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT); // Disable Half Transfer interrupt for DMA

    /* Infinite loop */
    for(;;)
    {
        imu_node_tick++;

        float relevant_angle = IMU_TRANS(HIPNUC_Hl12S2.data_frame.yaw, chassis_imu_init_angle_yaw) * M_PI / 180.0f ;

        Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.relevant_yaw_angle = relevant_angle;

        Sentry_Chassis_Gimbal_Interact_Info.chassis_vel_x_cmd = Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_y_ * sinf(relevant_angle) + Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_x_ * cosf(relevant_angle);
        Sentry_Chassis_Gimbal_Interact_Info.chassis_vel_y_cmd = Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_y_ * cosf(relevant_angle) - Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_x_ * sinf(relevant_angle);

#if UART10_RX_TEST
        uart_rx_check = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10_rx_test_buffer, 256);
#endif

#if USE_DMA_DOUBLE_BUFFER
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
#endif
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
#if UART10_TEST
        UART10_test_Status = HAL_UART_Transmit_DMA(&huart10, uart10_dma_test_msg, 8);
#endif

        // osDelay(1);
        taskYIELD();
    }
    /* USER CODE END imuTask */
}
