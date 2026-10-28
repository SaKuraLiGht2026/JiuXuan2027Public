//
// Created by ASUS on 2026/8/28.
//
#include "../Inc/imu_node.h"
#include <usart.h>
#include "../../Drv/DJI/Inc/Drv_DJI.h"
#include "../../../Drv/HIPNUC/Hl14S3/Inc/drv_HIPNUC_Hl14S3.h"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"

extern bool                                             isPitchInfoRXOK;
extern bool                                             isLittleYawInfoRXOK;
extern bool                                             isBigYawInfoRXOK;
extern bool                                             isIMUInfoRXOK;

extern bool                                             GimbalIMUCalibrationStart;

extern bool                                             isGimbalIMUCalibrationOK;

extern bool                                             IsInit;

extern float                                            gimbal_imu_init_yaw;
extern float                                            gimbal_imu_init_pitch;
extern float                                            gimbal_imu_init_roll;

extern DJI_Msg_INFO_t                                   DJI_GM6020_G2;

extern DMA_HandleTypeDef                                hdma_usart10_rx;

extern Drv_UART_Manage_Object                           USART10_Manage_Object;

extern HIPNUC_data_info_t                               HIPNUC_Hl14S3;

extern JiuXuan_Status_Info_t                            JiuXuan_Status_Info;

extern Sentry_Chassis_Gimbal_Interact_Info_t            Sentry_Chassis_Gimbal_Interact_Info;

void HIPNUC_UART_DATA_Process(uint8_t *Buffer, uint16_t Length) {
    HIPNUC_Hl14S3_UART_Hl91_DATA_Process(&HIPNUC_Hl14S3, Buffer, Length);
}

extern "C" void IMU_node(void *argument)
{
    /* USER CODE BEGIN IMU_node */
    UART_Init(&USART10_Manage_Object, &huart10, HIPNUC_UART_DATA_Process);
    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
    /* Infinite loop */
    for(;;)
    {
        if (USART10_Manage_Object.UART_Timeout) {
            UART_Receive_DMA_Repeat(&USART10_Manage_Object);
            __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
        }

        /**
         * @brief
         */
        if (isBigYawInfoRXOK && isLittleYawInfoRXOK && isPitchInfoRXOK && isIMUInfoRXOK) {
            GimbalIMUCalibrationStart = true;
        }

        if (isGimbalIMUCalibrationOK == false) {
            if (GimbalIMUCalibrationStart) {
                /**
                 * yaw
                 */
                gimbal_imu_init_yaw   = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_angle, Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.big_yaw_init_abs_angle)) +
                                        Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.relevant_yaw_angle * 180 / M_PI                                                                                                   +
                                        ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos, LITTLE_YAW_INIT_ANGLE)) -
                                        HIPNUC_Hl14S3.data_frame.yaw;
                // gimbal_imu_init_yaw   = gimbal_imu_init_yaw > 180.0f ? gimbal_imu_init_yaw - 360.0f : gimbal_imu_init_yaw < -180.0f ? gimbal_imu_init_yaw + 360.0f : gimbal_imu_init_yaw;

                /**
                 * pitch
                 */
                gimbal_imu_init_pitch = ANGLE_TRANS_Absolute_to_Signed(ANGLE_TRANS_INV(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos, PITCH_INIT_ANGLE)) - HIPNUC_Hl14S3.data_frame.pitch;

                /**
                 * roll
                 */
                gimbal_imu_init_roll  = 0.0f;

                isGimbalIMUCalibrationOK = true;
            }
        }

        /**
         * @brief 机器人阵亡后数据异常保护
         */
        // if (!JiuXuan_Status_Info.IsAlive) {
        //     isGimbalIMUCalibrationOK = false;
        //
        //     isBigYawInfoRXOK         = false;
        //     isLittleYawInfoRXOK      = false;
        //     isPitchInfoRXOK          = false;
        //     isIMUInfoRXOK            = false;
        // }

        osDelay(1);
    }
    /* USER CODE END IMU_node */
}
