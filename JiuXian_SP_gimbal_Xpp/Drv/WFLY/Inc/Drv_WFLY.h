//
// Created by ASUS on 2026/8/7.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_WFLY_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_WFLY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define WFLY_CHANNEL_COUNT  16
#define WFLY_DATA_LEN       25

#define Startup_threshold   100
#define Effective_range     500

    typedef struct __attribute__((packed))
    {
        uint16_t Channel_Data[WFLY_CHANNEL_COUNT];

        float    left_remote_x;
        float    left_remote_y;

        float    right_remote_x;
        float    right_remote_y;

        int16_t  doubel_switch_1;
        int16_t  doubel_switch_2;
        int16_t  doubel_switch_3;
        int16_t  doubel_switch_4;

        float    knob_up_left;
        float    knob_up_right;

        int16_t  doubel_switch_5;
        int16_t  doubel_switch_6;

        bool     left_bool_switch;
        bool     right_bool_switch;

        float    knob_down_left;
        float    knob_down_right;
    } WFLY_Data_t;

    void WFLY_Data_Process(uint8_t *Buffer, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_WFLY_H