//
// Created by ASUS on 2026/8/7.
//
#include "../Inc/Drv_WFLY.h"

WFLY_Data_t WFLY_Data;

void WFLY_Data_Process(uint8_t *Buffer, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++) {
        if (Buffer[i] != 0x0F) {
            continue;
        }

        if (i + WFLY_DATA_LEN > Length) {
            break;
        }

        /**
         * Set Data
         */
        /* Channel 1 - 8 */
        WFLY_Data.Channel_Data[0] = ((Buffer[i + 1]  | (Buffer[i + 2]  << 8)) & 0x07FF);
        WFLY_Data.Channel_Data[1] = ((Buffer[i + 2]  >> 3) | (Buffer[i + 3]  << 5)) & 0x07FF;
        WFLY_Data.Channel_Data[2] = ((Buffer[i + 3]  >> 6) | (Buffer[i + 4]  << 2) | (Buffer[i + 5]  << 10)) & 0x07FF;
        WFLY_Data.Channel_Data[3] = ((Buffer[i + 5]  >> 1) | (Buffer[i + 6]  << 7)) & 0x07FF;
        WFLY_Data.Channel_Data[4] = ((Buffer[i + 6]  >> 4) | (Buffer[i + 7]  << 4)) & 0x07FF;
        WFLY_Data.Channel_Data[5] = ((Buffer[i + 7]  >> 7) | (Buffer[i + 8]  << 1) | (Buffer[i + 9]  << 9)) & 0x07FF;
        WFLY_Data.Channel_Data[6] = ((Buffer[i + 9]  >> 2) | (Buffer[i + 10] << 6)) & 0x07FF;
        WFLY_Data.Channel_Data[7] = ((Buffer[i + 10] >> 5) | (Buffer[i + 11] << 3)) & 0x07FF;

        /* Channel 9 - 16 */
        WFLY_Data.Channel_Data[8]  = ((Buffer[i + 12] | (Buffer[i + 13] << 8)) & 0x07FF);
        WFLY_Data.Channel_Data[9]  = ((Buffer[i + 13] >> 3) | (Buffer[i + 14] << 5)) & 0x07FF;
        WFLY_Data.Channel_Data[10] = ((Buffer[i + 14] >> 6) | (Buffer[i + 15] << 2) | (Buffer[i + 16] << 10)) & 0x07FF;
        WFLY_Data.Channel_Data[11] = ((Buffer[i + 16] >> 1) | (Buffer[i + 17] << 7)) & 0x07FF;
        WFLY_Data.Channel_Data[12] = ((Buffer[i + 17] >> 4) | (Buffer[i + 18] << 4)) & 0x07FF;
        WFLY_Data.Channel_Data[13] = ((Buffer[i + 18] >> 7) | (Buffer[i + 19] << 1) | (Buffer[i + 20] << 9)) & 0x07FF;
        WFLY_Data.Channel_Data[14] = ((Buffer[i + 20] >> 2) | (Buffer[i + 21] << 6)) & 0x07FF;
        WFLY_Data.Channel_Data[15] = ((Buffer[i + 21] >> 5) | (Buffer[i + 22] << 3)) & 0x07FF;

        WFLY_Data.left_remote_x   = (((float)WFLY_Data.Channel_Data[3]  - 1031.0f) / (1695.0f - 353.0f)) *  2.0f;
        WFLY_Data.left_remote_y   = (((float)WFLY_Data.Channel_Data[1]  - 1024.0f) / (1695.0f - 353.0f)) * -2.0f;
        WFLY_Data.right_remote_x  = (((float)WFLY_Data.Channel_Data[0]  - 1024.0f) / (1695.0f - 353.0f)) *  2.0f;
        WFLY_Data.right_remote_y  = (((float)WFLY_Data.Channel_Data[2]  - 1024.0f) / (1695.0f - 353.0f)) *  2.0f;

        WFLY_Data.knob_up_left    = (((float)WFLY_Data.Channel_Data[8]  - 1024.0f) / (1695.0f - 353.0f)) * 2.0f + 1.0f;
        WFLY_Data.knob_up_right   = (((float)WFLY_Data.Channel_Data[9]  - 1024.0f) / (1695.0f - 353.0f)) * 2.0f + 1.0f;

        WFLY_Data.knob_down_left  = (((float)WFLY_Data.Channel_Data[14] - 1024.0f) / (1695.0f - 353.0f)) * 2.0f;
        WFLY_Data.knob_down_right = (((float)WFLY_Data.Channel_Data[15] - 1024.0f) / (1695.0f - 353.0f)) * 2.0f;

        WFLY_Data.doubel_switch_1 = (int16_t)(WFLY_Data.Channel_Data[4]  > 1124 ? 2 : (WFLY_Data.Channel_Data[4]  < 924 ? 0 : 1));
        WFLY_Data.doubel_switch_2 = (int16_t)(WFLY_Data.Channel_Data[5]  > 1124 ? 2 : (WFLY_Data.Channel_Data[5]  < 924 ? 0 : 1));
        WFLY_Data.doubel_switch_3 = (int16_t)(WFLY_Data.Channel_Data[6]  > 1124 ? 2 : (WFLY_Data.Channel_Data[6]  < 924 ? 0 : 1));
        WFLY_Data.doubel_switch_4 = (int16_t)(WFLY_Data.Channel_Data[7]  > 1124 ? 2 : (WFLY_Data.Channel_Data[7]  < 924 ? 0 : 1));
        WFLY_Data.doubel_switch_5 = (int16_t)(WFLY_Data.Channel_Data[10] > 1124 ? 2 : (WFLY_Data.Channel_Data[10] < 924 ? 0 : 1));
        WFLY_Data.doubel_switch_6 = (int16_t)(WFLY_Data.Channel_Data[12] > 1124 ? 2 : (WFLY_Data.Channel_Data[12] < 924 ? 0 : 1));

        WFLY_Data.left_bool_switch  = (WFLY_Data.Channel_Data[11] > 1024 ? true : false); //SF
        WFLY_Data.right_bool_switch = (WFLY_Data.Channel_Data[13] > 1024 ? true : false); //SH
    }
}