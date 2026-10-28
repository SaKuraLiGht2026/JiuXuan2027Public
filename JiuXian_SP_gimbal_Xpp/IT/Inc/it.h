//
// Created by ASUS on 2026/7/14.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_IT_H
#define JIUXIAN_SP_GIMBAL_XPP_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx.h"

    extern void TIM1_Period_Callback(void);
    extern void TIM3_State_Update_Callback(void);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_IT_H