//
// Created by ASUS on 2026/8/6.
//
#include "../Inc/RemoteCtrl_node.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"
#include "../../Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h"
#include "../../Drv/WFLY/Inc/Drv_WFLY.h"
#include "../../Algorithm/MathUnit/Inc/MathUnit.hpp"
#include "../../PreApp/Signal_Generator/Inc/Signal_Generator.hpp"
#include "../../Drv/RoboMaster_Referee_System/VTM_Link/Inc/drv_robomaster_vtm_link.h"
#include "../../Drv/MiniPC/Inc/Drv_MiniPC.h"
#include <usart.h>

bool                                                  IsWFLYConnectTimeout = false;

uint64_t                                              wfly_connect_tick = 0;

extern bool                                           IsInit;

extern bool                                           isPluckErr;

extern float                                          set_Pluck_speed;
extern float                                          set_friction_speed;

extern float                                          pitch_angle;
extern float                                          yaw_angle;
extern float                                          set_yaw_speed;
extern float                                          gimbal_navy_yaw;

extern uint64_t                                       JiuXuan_System_Tick;

extern DMA_HandleTypeDef                              hdma_uart5_rx;

extern Drv_UART_Manage_Object                         UART5_Manage_Object;

extern WFLY_Data_t                                    WFLY_Data;

extern MiniPC_Msg_Info_t                              minipc_msg_info;

extern JiuXuan_Status_Info_t                          JiuXuan_Status_Info;
extern Sentry_Chassis_Gimbal_Interact_Info_t          Sentry_Chassis_Gimbal_Interact_Info;

extern RoboMaster_Referee_System_VTM_Link_Info_t      RM_Referee_VTM_Link_Info;

extern Pulse_Generator                                JiuXuan_Fire_Semi_Auto_Trigger;
extern Pulse_Generator                                JiuXuan_RM_Client_Fire_Switch;
extern Pulse_Generator                                JiuXuan_RM_Client_Gyro_Switch;
extern Pulse_Generator                                JiuXuan_RM_Client_Semi_Switch;

void WFLY_Data_Receiver_CpltCallback(uint8_t *Buffer, uint16_t Length) {
    WFLY_Data_Process(Buffer, Length);

    wfly_connect_tick = 0;
    IsWFLYConnectTimeout = false;
}

extern "C" void Remote_node(void *argument)
{
    /* USER CODE BEGIN Remote_node */
    UART_Init(&UART5_Manage_Object, &huart5, WFLY_Data_Receiver_CpltCallback);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

    JiuXuan_Status_Info.IsSentryAttack = false;

    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x = 1;
    JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y = 1;
    /* Infinite loop */
    for(;;)
    {
        if (IsInit) {
            if (UART5_Manage_Object.UART_Timeout) {
                UART_Receive_DMA_Repeat(&UART5_Manage_Object);
                __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
            }

            wfly_connect_tick++;
            if (wfly_connect_tick > 1000) {
                wfly_connect_tick = 1000;
                IsWFLYConnectTimeout = true;
            }

            /* ============================================================================================================ */
            JiuXuan_Menu_Selector(WFLY_Data.doubel_switch_1, WFLY_Data.doubel_switch_2, WFLY_Data.doubel_switch_3, WFLY_Data.doubel_switch_4, WFLY_Data.doubel_switch_5, WFLY_Data.left_bool_switch, WFLY_Data.doubel_switch_6, WFLY_Data.right_bool_switch);

            JiuXuan_Status_Info.IsSentryAttack = (minipc_msg_info.Rx_Info.mode == 0x01 || minipc_msg_info.Rx_Info.mode == 0x02 ? true : false);

            /* ============================================================================================================ */
            if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_work_mode == Free) {
                if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Infantry && JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode == JiuXuan_Remote_Ctrl) {
                    float vel_gimbal_axis_x = WFLY_Data.left_remote_y * WFLY_Data.knob_up_left /  2.0f;
                    float vel_gimbal_axis_y = WFLY_Data.left_remote_x * WFLY_Data.knob_up_left / -2.0f;

                    // Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = WFLY_Data.left_remote_x  * CMD_MAX_VEL   * WFLY_Data.knob_up_left;
                    // Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = WFLY_Data.left_remote_y  * CMD_MAX_VEL   * WFLY_Data.knob_up_left;
                    Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = (vel_gimbal_axis_x * cosf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f) -
                                                                     vel_gimbal_axis_y * sinf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f)) * CMD_MAX_VEL;
                    Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = (vel_gimbal_axis_x * sinf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f) +
                                                                    vel_gimbal_axis_y * cosf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f)) * CMD_MAX_VEL;

                    Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = WFLY_Data.knob_down_left * CMD_MAX_OMEGA;

                    set_yaw_speed                                 =                    -1.0f *        1800.0f * (fabs(WFLY_Data.right_remote_x) < 0.1f ? 0.0f : WFLY_Data.right_remote_x) * WFLY_Data.knob_up_right;
                    pitch_angle                                   = WFLY_Data.right_remote_y * PITCH_WORKSPACE_RANGE * 0.5f;

                    // yaw_angle                                    += set_yaw_speed * 0.001f;

                    if (JiuXuan_Status_Info.IsSentryAttack) {
                        yaw_angle = minipc_msg_info.Rx_Info.yaw;

                        pitch_angle = minipc_msg_info.Rx_Info.pitch;
                    }

                    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_friction_switch == Friction_ON) {
                        set_friction_speed = 15.0f;
                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode == Disable_Fire) {
                            set_Pluck_speed = 0.0f;
                            JiuXuan_Fire_Semi_Auto_Trigger.Set_Pulse_Generato_Input(false);
                        }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode == Full_Auto) {
                            JiuXuan_Fire_Semi_Auto_Trigger.Set_Pulse_Generato_Input(false);

                            if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_switch == Fire_ON) {
                                if (isPluckErr != true) {
                                    set_Pluck_speed = (10.0f + 6.0f * WFLY_Data.knob_down_right * -1.0f);
                                }
                            }else {
                                set_Pluck_speed =  0.0f;
                            }
                        }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_mode == Semi_Auto) {
                            set_Pluck_speed =  0.0f;
                            if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_fire_switch == Fire_ON) {
                                JiuXuan_Fire_Semi_Auto_Trigger.Set_Pulse_Generato_Input(true);
                            }else {
                                JiuXuan_Fire_Semi_Auto_Trigger.Set_Pulse_Generato_Input(false);
                            }
                        }
                    }else {
                        set_friction_speed =   0.0f;
                        set_Pluck_speed    =   0.0f;
                    }
                }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Infantry && JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_infantry_ctrl_mode == JiuXuan_RM_Client_Ctrl) {
                    yaw_angle   -= RM_Referee_VTM_Link_Info.remote_control_vt02_241225.mouse_x /  16.0f;
                    pitch_angle += RM_Referee_VTM_Link_Info.remote_control_vt02_241225.mouse_y / 256.0f;

                    float vel_gimbal_axis_y = ((RM_Referee_VTM_Link_Info.A ?  0.3f : 0.0f) + (RM_Referee_VTM_Link_Info.D ? -0.3f : 0.0f)) * JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x;
                    float vel_gimbal_axis_x = ((RM_Referee_VTM_Link_Info.S ? -0.3f : 0.0f) + (RM_Referee_VTM_Link_Info.W ?  0.3f : 0.0f)) * JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y;

                    if (RM_Referee_VTM_Link_Info.Shift) {
                        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x += 0.0005f;
                        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y += 0.0005f;
                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x > 5) JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x = 5;
                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y > 5) JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y = 5;
                    }else {
                        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x -= 0.0005f;
                        JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y -= 0.0005f;
                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x < 1) JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_x = 1;
                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y < 1) JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.speed_gain_y = 1;
                    }

                    JiuXuan_RM_Client_Fire_Switch.Set_Pulse_Generato_Input(RM_Referee_VTM_Link_Info.F);
                    JiuXuan_RM_Client_Gyro_Switch.Set_Pulse_Generato_Input(RM_Referee_VTM_Link_Info.G);
                    JiuXuan_RM_Client_Semi_Switch.Set_Pulse_Generato_Input(RM_Referee_VTM_Link_Info.R);

                    if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientFricON) {
                        set_friction_speed = 20.0f;

                        if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientSemiON) {
                            JiuXuan_Fire_Semi_Auto_Trigger.Set_Pulse_Generato_Input(RM_Referee_VTM_Link_Info.remote_control_vt02_241225.left_button_down != 0);
                        }else {
                            if (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.left_button_down != 0) {
                                set_Pluck_speed = 15.0f;
                            }else {
                                set_Pluck_speed = 0.0f;
                            }
                        }
                    }else {
                        set_friction_speed =  0.0f;
                    }

                    Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = (vel_gimbal_axis_x * cosf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f) -
                                                                     vel_gimbal_axis_y * sinf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f)) * CMD_MAX_VEL;
                    Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = (vel_gimbal_axis_x * sinf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f) +
                                                                     vel_gimbal_axis_y * cosf(ANGLE_TRANS_Absolute_to_Signed(gimbal_navy_yaw) * M_PI / 180.0f)) * CMD_MAX_VEL;

                    Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_rm_client_info.isRMClientGyroON ? 6.28f : 0.0f);
                }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Sentry) {
                    if (JiuXuan_Status_Info.IsSentryAttack) {
                        if (minipc_msg_info.Rx_Info.mode == 0x02) {
                            yaw_angle          = minipc_msg_info.Rx_Info.yaw   * 180.0f / M_PI;
                            pitch_angle        = minipc_msg_info.Rx_Info.pitch * 180.0f / M_PI;

                            set_friction_speed = 18.0f;
                            set_Pluck_speed    = 20.0f;
                        }else {
                            yaw_angle          = minipc_msg_info.Rx_Info.yaw   * 180.0f / M_PI;
                            pitch_angle        = minipc_msg_info.Rx_Info.pitch * 180.0f / M_PI;

                            set_friction_speed = 0.0f;
                            set_Pluck_speed    = 0.0f;
                        }
                    }else {
                        set_yaw_speed  = -1000.0f;
                        // yaw_angle     += set_yaw_speed * 0.001f;

                        set_friction_speed = 0.0f;
                        set_Pluck_speed    = 0.0f;
                    }

                    Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = minipc_msg_info.Rx_Info.linear_x;
                    Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = minipc_msg_info.Rx_Info.linear_y;
                    if (minipc_msg_info.Rx_Info.angular_z == 0.0f) {
                        // Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = CMD_MAX_OMEGA * 0.5 * sinf(JiuXuan_System_Tick / 1000.0f) + CMD_MAX_OMEGA * 0.8;
                        Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = 0.0f;
                    }else {
                        Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = minipc_msg_info.Rx_Info.angular_z;
                    }

                }else {
                    Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = 0.0f;
                    Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = 0.0f;
                    Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = 0.0f;

                    set_yaw_speed                                 = 0.0f;
                    pitch_angle                                   = 0.0f;
                }
            }else if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_work_mode == War) {
                Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = 0.0f;

                set_yaw_speed                                 = 0.0f;
                pitch_angle                                   = 0.0f;
            }else {
                Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.omega_cmd = 0.0f;

                set_yaw_speed                                 = 0.0f;
                pitch_angle                                   = 0.0f;
            }

            Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_vel_x = fabs(Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd) > 0.05f ? Sentry_Chassis_Gimbal_Interact_Info.vel_x_cmd : 0.0f;
            Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_vel_y = fabs(Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd) > 0.05f ? Sentry_Chassis_Gimbal_Interact_Info.vel_y_cmd : 0.0f;
            Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_omega = fabs(Sentry_Chassis_Gimbal_Interact_Info.omega_cmd) > 0.05f ? Sentry_Chassis_Gimbal_Interact_Info.omega_cmd : 0.0f;

            if (IsWFLYConnectTimeout) {
                Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_vel_x = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_vel_y = 0.0f;
                Sentry_Chassis_Gimbal_Interact_Info.tx_cmd.cmd_omega = 0.0f;
            }

            memcpy(Sentry_Chassis_Gimbal_Interact_Info.tx_cmd_msg, &Sentry_Chassis_Gimbal_Interact_Info.tx_cmd, sizeof(Sentry_Chassis_Gimbal_Interact_Info.tx_cmd));
        }
        osDelay(1);
    }
    /* USER CODE END Remote_node */
}
