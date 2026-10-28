//
// Created by ASUS on 2026/7/14.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_WS2812_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define WS2812_SPI_UNIT     hspi6
#define WS2812_LowLevel     0xC0     // 0码
#define WS2812_HighLevel    0xF0     // 1码

    extern SPI_HandleTypeDef WS2812_SPI_UNIT;

    void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_WS2812_H