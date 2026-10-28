//
// Created by ASUS on 2026/7/15.
//
#include "../Inc/Drv_buzzer.h"

/**
 * @brief : Sentry open sound effect
 */
void sentry_open(void) {
    TIM12->PSC  = 225;
    TIM12->CCR2 = 80;
    osDelay(250);

    TIM12->PSC  = 200;
    TIM12->CCR2 = 80;
    osDelay(250);

    TIM12->PSC  = 147;
    TIM12->CCR2 = 80;
    osDelay(400);

    TIM12->CCR2 = 0;
    osDelay(550);
}

/**
 * @brief : Normal status sound effect
 * @param normal_tick
 * @param tick
 */
void normal_status(uint32_t normal_tick, uint32_t tick)
{
    TIM12->PSC  = normal_tick;
    TIM12->CCR2 = 600;
    osDelay(200);

    TIM12->CCR2 = 0;
    osDelay(tick);
}

/**
 * @brief : Attack status sound effect
 * @param normal_tick
 * @param tick
 */
void attack_status(uint32_t normal_tick, uint32_t tick)
{
    TIM12->PSC  = normal_tick;
    TIM12->CCR2 = 600;
    osDelay(50);

    TIM12->CCR2 = 0;
    osDelay(50);

    TIM12->CCR2 = 600;
    osDelay(100);

    TIM12->CCR2 = 0;
    osDelay(tick);
}
