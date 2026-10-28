//
// Created by ASUS on 2026/7/30.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_CRC_H
#define JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../Inc/RoboMaster_Referee_Info_Define.h"

    //////////////////////////////////////////////////////////////CRC8////////////////////////////////////////////////////////////////
    /**
     * @brief CRC8校验
     *
     * @param pchMessage
     * @param dwLength
     * @param ucCRC8
     * @return Referee_uchar
     */
    extern Referee_uchar Get_CRC8_Check_Sum(Referee_uchar *pchMessage, Referee_uint dwLength, Referee_uchar ucCRC8);

    /**
     * @brief CRC8校验
     *
     * @param pchMessage
     * @param dwLength
     * @return Referee_uint
     */
    extern Referee_uint Verify_CRC8_Check_Sum(Referee_uchar *pchMessage, Referee_uint dwLength);

    /**
     * @brief CRC8校验
     *
     * @param pchMessage
     * @param dwLength
     * @return Referee_action
     */
    extern Referee_action Append_CRC8_Check_Sum(Referee_uchar *pchMessage, Referee_uint dwLength);
    extern Referee_uint8 Append_CRC8_Check_Sum2(Referee_uchar *pchMessage, Referee_uint dwLength);

    /////////////////////////////////////////////////////////////////////////CRC16////////////////////////////////////////////////////////////////////////
    /**
     * @brief CRC16校验
     *
     * @param pchMessage
     * @param dwLength
     * @param wCRC
     * @return Referee_uint16
     */
    extern Referee_uint16 Get_CRC16_Check_Sum(Referee_uint8 *pchMessage, Referee_uint32 dwLength, Referee_uint16 wCRC);

    /**
     * @brief CRC16校验
     *
     * @param pchMessage
     * @param dwLength
     * @return Referee_uint32
     */
    extern Referee_uint32 Verify_CRC16_Check_Sum(Referee_uint8 *pchMessage, Referee_uint32 dwLength);

    /**
     * @brief CRC16校验
     *
     * @param pchMessage
     * @param dwLength
     * @return Referee_action
     */
    extern Referee_action Append_CRC16_Check_Sum(Referee_uint8 *pchMessage, Referee_uint32 dwLength);
    extern Referee_uint16 Append_CRC16_Check_Sum2(Referee_uint8 *pchMessage, Referee_uint32 dwLength);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_CRC_H