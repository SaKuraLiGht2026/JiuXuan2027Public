//
// Created by ASUS on 2026/7/15.
//
#include "../Inc/buzzer_node.h"
#include "../../Drv/Buzzer/Inc/Drv_buzzer.h"
#include "../../Config/JiuXian_Gimbal_Config.hpp"

extern bool                     IsInit;

extern JiuXuan_Status_Info_t    JiuXuan_Status_Info;

extern "C" void Buzzer_node(void *argument)
{
    /* USER CODE BEGIN Buzzer_node */
    sentry_open();
    /* Infinite loop */
    for(;;)
    {
        if (IsInit) {
            if (JiuXuan_Status_Info.JiuXuan_Menu_Info.jiu_xuan_troop == Sentry) {
                if (JiuXuan_Status_Info.IsSentryAttack) {
                    attack_status(19, 800);
                }else {
                    normal_status(19, 1400);
                }
            }
            // normal_status(19, 1400);
            // attack_status(19, 800);
        }
        taskYIELD();
    }
    /* USER CODE END Buzzer_node */
}
