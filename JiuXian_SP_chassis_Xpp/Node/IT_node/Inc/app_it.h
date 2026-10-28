//
// Created by ASUS on 2026/4/4.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_APP_IT_H
#define JIUXIAN_SP_CHASSIS_XPP_APP_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

void TIM1_PeriodElapsedCallback();
void TIM3_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_CHASSIS_XPP_APP_IT_H