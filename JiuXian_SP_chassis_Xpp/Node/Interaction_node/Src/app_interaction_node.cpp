//
// Created by ASUS on 2026/7/28.
//
#include "../Inc/app_interaction_node.h"
#include "../../Config/Inc/JiuXian_Config.hpp"
#include "../../../Bsp/FDCAN/Inc/bsp_fdcan.h"
#include "../../../Algorithm/MathUnit/Inc/MathUnit.hpp"
#include "../../../Algorithm/Ramp_Planner/Ramp.hpp"
#include "../../../Algorithm/S_Curve/Inc/alg_s_curve.hpp"
#include "fdcan.h"

#define FDCAN_TEST              0

extern bool                     IsChassisSystemInit;

extern bool                     ALL_FDCAN_INIT_OK;
extern bool                     isFDCAN2MSGTXOK;
extern bool                     isFDCAN2FIFO0RXOK;

extern uint64_t                 interaction_node_tick;
extern uint64_t                 can2_timeout_Tick;

extern Sentry_Chassis_Gimbal_Interact_Info_t Sentry_Chassis_Gimbal_Interact_Info;

extern SCurvePlanner            JiuXuan_Chassis_Speed_Curve;
extern Ramp_Planner             JiuXuan_Chassis_Speed_x_Ramp;
extern Ramp_Planner             JiuXuan_Chassis_Speed_y_Ramp;

#if FDCAN_TEST

uint8_t fdcan_test_msg1[64] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

#endif

extern "C" void Interaction_node(void *argument)
{
    /* USER CODE BEGIN Interaction_node */
    Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.big_yaw_init_abs_angle = BIG_YAW_INIT;
    /* Infinite loop */
    for(;;)
    {
        interaction_node_tick++;

        if (IsChassisSystemInit) {
            memcpy(Sentry_Chassis_Gimbal_Interact_Info.tx_cmd_msg, &Sentry_Chassis_Gimbal_Interact_Info.tx_cmd, sizeof(Sentry_Chassis_Gimbal_Interact_Info.tx_cmd));

            FDCAN_Transmit_Data_with_TxBuffer(&hfdcan2, 0x346, Sentry_Chassis_Gimbal_Interact_Info.tx_cmd_msg, 64);
        }

        if (isFDCAN2FIFO0RXOK) {
            can2_timeout_Tick = 0;

            // Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_x_ = JiuXuan_Chassis_Speed_x_Ramp.update(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.cmd_vel_x);
            // Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_y_ = JiuXuan_Chassis_Speed_y_Ramp.update(Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.cmd_vel_y);

            Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_x_ = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.cmd_vel_x;
            Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_y_ = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.cmd_vel_y;

            Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_ = Sentry_Chassis_Gimbal_Interact_Info.rx_cmd.cmd_omega;
            if (fabs(Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_) < 0.1) Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_ = 0.0f;

            isFDCAN2FIFO0RXOK = false;
        }else {
            can2_timeout_Tick++;

            if (can2_timeout_Tick > 200) {
                can2_timeout_Tick = 200;

                Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_x_ = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.cmd_vel_y_ = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.cmd_omega_ = 0.0f;
            }
        }


        osDelay(1);
        // taskYIELD();
    }
    /* USER CODE END Interaction_node */
}
