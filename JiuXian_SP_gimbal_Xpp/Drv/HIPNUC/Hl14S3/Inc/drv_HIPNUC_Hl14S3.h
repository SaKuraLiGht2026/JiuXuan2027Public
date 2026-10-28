//
// Created by ASUS on 2026/8/28.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_HIPNUC_HL14S3_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_HIPNUC_HL14S3_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define F4(p) (*(float *)(p))

#define HIPNUC_DATA_FRAME_HEADER1  0x5A
#define HIPNUC_DATA_FRAME_HEADER2  0xA5

#define TAG_Hl91                   0x91
#define TAG_Hl83                   0x83

    typedef struct {
        uint8_t  tag;
        uint16_t main_status;

        /* main status */ /* 如果数值为1则为报警，反之正常。 */
        /* 0~7bit */
        /* bit0~2 reserve   */
        bool     wb_conv;        /* bit3  */
        bool     mag_dist;       /* bit4  */
        bool     acc_sat;        /* bit5  */
        bool     gyro_sat;       /* bit6  */
        bool     att_conv;       /* bit7  */

        /* 8~15bit */
        /* bit8~9 reserve   */
        bool     mag_aiding;     /* bit10 */
        bool     utc_synced;     /* bit11 */
        bool     sout_pulse;     /* bit12 */
        /* bit13~15 reserve */

        uint8_t  temperature;
        float    air_pressure;
        uint32_t sys_time;

        /* acc */
        float    acc_x;
        float    acc_y;
        float    acc_z;

        /* gyr */
        float    gyro_x;
        float    gyro_y;
        float    gyro_z;

        /* mag */
        float    mag_x;
        float    mag_y;
        float    mag_z;

        /* Eular angle */
        float    roll;
        float    pitch;
        float    yaw;

        /* Quarters */
        float    quaternion_w;
        float    quaternion_x;
        float    quaternion_y;
        float    quaternion_z;
    }HIPNUC_Hl14S3_Hl91_Data_INFO_t;

    typedef struct {
        uint8_t                           frame_Header1;
        uint8_t                           frame_Header2;
        uint16_t                          data_length;
        uint16_t                          CRC16_code;

        HIPNUC_Hl14S3_Hl91_Data_INFO_t    data_frame;
    }HIPNUC_data_info_t;

    void HIPNUC_Hl14S3_UART_Hl91_DATA_Process(HIPNUC_data_info_t *hipnuc, uint8_t *Buffer, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_HIPNUC_HL14S3_H