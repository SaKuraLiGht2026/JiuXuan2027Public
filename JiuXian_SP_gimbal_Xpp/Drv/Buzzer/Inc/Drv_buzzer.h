//
// Created by ASUS on 2026/7/15.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_BUZZER_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include "tim.h"
#include "cmsis_os2.h"

    void sentry_open(void);
    void normal_status(uint32_t normal_tick, uint32_t tick);
    void attack_status(uint32_t normal_tick, uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_BUZZER_H