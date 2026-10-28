//
// Created by ASUS on 2026/8/28.
//
#include "../Inc/drv_HIPNUC_Hl14S3.h"

/* CRC Check Begin */
uint16_t hipnuc_crc16_init = 0x0000;
uint16_t hipnuc_crc16_poly = 0x1021;

const uint16_t hipnuc_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * Descriptions: CRC16 checksum function
 * Input:  pchMessage - Data to check
 *         dwLength   - Stream length
 *         wCRC       - Initialized checksum (use CRC16_INIT)
 * Output: CRC checksum
 */
static uint16_t HIPNUC_Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
        return 0xFFFF;

    while (dwLength--)
    {
        chData = *pchMessage++;
        wCRC = (uint16_t)(wCRC << 8) ^ hipnuc_crc16_table[((uint16_t)(wCRC >> 8) ^ (uint16_t)chData) & 0x00FF];
    }
    return wCRC;
}

/**
 * Descriptions: CRC16 Verify function
 *               CRC 字段在偏移4-5，计算时跳过
 * Input:  pchMessage - 完整数据帧指针
 *                       [SOF:2B][LEN:2B][CRC:2B][DATA:LEN bytes]
 * Output: True or False (CRC Verify Result)
 */
static uint32_t HIPNUC_Verify_CRC16_Check_Sum(uint8_t *pchMessage)
{
    uint16_t wExpected = 0;
    uint16_t data_len;

    if (pchMessage == NULL)
        return 0;

    data_len = (uint16_t)pchMessage[2] | ((uint16_t)pchMessage[3] << 8);

    wExpected = HIPNUC_Get_CRC16_Check_Sum(pchMessage    , 4, hipnuc_crc16_init);
    wExpected = HIPNUC_Get_CRC16_Check_Sum(pchMessage + 6, data_len, wExpected);

    return (wExpected & 0xFF) == pchMessage[4] && ((wExpected >> 8) & 0xFF) == pchMessage[5];
}
/* CRC Check End */

void HIPNUC_Hl14S3_UART_Hl91_DATA_Process(HIPNUC_data_info_t *hipnuc, uint8_t *Buffer, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++) {
        /**
         * 帧头1不对
         */
        if (Buffer[i] != HIPNUC_DATA_FRAME_HEADER1) {
            continue;
        }

        hipnuc->frame_Header1 = Buffer[i];

        /**
         * 帧头2不对
         */
        if (Buffer[i + 1] != HIPNUC_DATA_FRAME_HEADER2) {
            continue;
        }

        hipnuc->frame_Header2 = Buffer[i + 1];

        /**
         * 帧头都没问题但是不够校验CRC16了
         */
        if (i + 4 > Length) {
            break;
        }

        /**
         * CRC16没过
         */
        if (HIPNUC_Verify_CRC16_Check_Sum(Buffer + i) == false) {
            continue;
        }

        hipnuc->CRC16_code    = (Buffer[i + 5] << 8) | Buffer[i + 4];

        hipnuc->data_length   = (Buffer[i + 3] << 8) | Buffer[i + 2];

        uint16_t hipnuc_Hl14S3_hl91_data_frame_beginning_addr = 6;

        /**
         * 超长度了
         */
        if (i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + hipnuc->data_length > Length) {
            break;
        }

        /**
         * TAG错误
         */
        if (Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr] != TAG_Hl91) {
            continue;
        }

        hipnuc->data_frame.tag = Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr];

        /* 上述过程都通过，则搬运数据。 */
        hipnuc->data_frame.main_status = (Buffer[i + 8] << 8) | Buffer[i + 7];

        hipnuc->data_frame.wb_conv    = hipnuc->data_frame.main_status >>  3;
        hipnuc->data_frame.mag_dist   = hipnuc->data_frame.main_status >>  4;
        hipnuc->data_frame.acc_sat    = hipnuc->data_frame.main_status >>  5;
        hipnuc->data_frame.gyro_sat   = hipnuc->data_frame.main_status >>  6;
        hipnuc->data_frame.att_conv   = hipnuc->data_frame.main_status >>  7;

        hipnuc->data_frame.mag_aiding = hipnuc->data_frame.main_status >> 10;
        hipnuc->data_frame.utc_synced = hipnuc->data_frame.main_status >> 11;
        hipnuc->data_frame.sout_pulse = hipnuc->data_frame.main_status >> 12;

        hipnuc->data_frame.temperature = Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr+ 3];

        hipnuc->data_frame.air_pressure = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr +  4]);

        hipnuc->data_frame.sys_time     = (uint32_t)(Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr +  8]       |
                                                     Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr +  9] <<  8 |
                                                     Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 10] << 16 |
                                                     Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 11] << 24 );

        /* acc */
        hipnuc->data_frame.acc_x        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 12]);

        hipnuc->data_frame.acc_y        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 16]);

        hipnuc->data_frame.acc_z        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 20]);

        /* gyro */
        hipnuc->data_frame.gyro_x       = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 24]);

        hipnuc->data_frame.gyro_y       = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 28]);

        hipnuc->data_frame.gyro_z       = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 32]);

        /* mag */
        hipnuc->data_frame.mag_x        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 36]);

        hipnuc->data_frame.mag_y        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 40]);

        hipnuc->data_frame.mag_z        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 44]);

        /* Eular Angle */
        hipnuc->data_frame.roll         = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 48]);

        hipnuc->data_frame.pitch        = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 52]);

        hipnuc->data_frame.yaw          = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 56]);

        /* quaternion */
        hipnuc->data_frame.quaternion_w = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 60]);

        hipnuc->data_frame.quaternion_x = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 64]);

        hipnuc->data_frame.quaternion_y = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 68]);

        hipnuc->data_frame.quaternion_z = F4(&Buffer[i + hipnuc_Hl14S3_hl91_data_frame_beginning_addr + 72]);
    }
}
