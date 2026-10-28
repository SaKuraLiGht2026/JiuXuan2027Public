//
// Created by ASUS on 2026/7/14.
//
#include "../Inc/Indicator.h"

uint8_t Breath_UP_or_DOWN = 0;
uint8_t Breath_Value      = 0;

void Normal_Breathing(void) {
    if (Breath_UP_or_DOWN == 1) Breath_Value += 1;
    if (Breath_UP_or_DOWN == 0) Breath_Value -= 1;

    if (Breath_Value      == 255) Breath_UP_or_DOWN = 0;
    if (Breath_Value      == 0  ) Breath_UP_or_DOWN = 1;

    WS2812_Ctrl(0, Breath_Value, 0);
}
