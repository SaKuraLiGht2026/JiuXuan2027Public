//
// Created by ASUS on 2026/10/3.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_DRV_XIAOHE_H
#define JIUXIAN_SP_CHASSIS_XPP_DRV_XIAOHE_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取缓冲区数据的宏定义
 * 这些宏定义用于从缓冲区中按大端序或小端序读取 16 位或 32 位整数。它们将缓冲区中的字节组合
 */
    // 从缓冲区按大端序读取 16 位无符号整数
#define READ_U16_BE(p)  ((uint16_t)(((p)[0] << 8) | (p)[1]))
    // 从缓冲区按大端序读取 16 位有符号整数
#define READ_S16_BE(p)  ((int16_t)(((p)[0] << 8) | (p)[1]))
    // 从缓冲区按小端序读取 16 位无符号整数
#define READ_U16_LE(p)  ((uint16_t)(((p)[1] << 8) | (p)[0]))
    // 从缓冲区按小端序读取 16 位有符号整数
#define READ_S16_LE(p)  ((int16_t)(((p)[1] << 8) | (p)[0]))
    // 从缓冲区按大端序读取 32 位有符号整数
#define READ_S32_BE(p)  ((int32_t)(((uint32_t)(p)[0] << 24) | \
                                    ((uint32_t)(p)[1] << 16) | \
                                    ((uint32_t)(p)[2] << 8)  | \
                                     (uint32_t)(p)[3]))
    // 从缓冲区按小端序读取 32 位有符号整数
#define READ_S32_LE(p)  ((int32_t)(((uint32_t)(p)[3] << 24) | \
                                   ((uint32_t)(p)[2] << 16) | \
                                   ((uint32_t)(p)[1] << 8)  | \
                                   (uint32_t)(p)[0]))

#define REQUEST_MSG_SIZE         8

#define XIAOHE_READ_CODE      0x03
#define XIAOHE_WRITE_CODE     0x06
#define XIAOHE_ERROR_CODE     0x80

#define VOLTAGE_REG_ADDR      0x0000
#define CURRENT_REG_ADDR      0x0001
#define POWER_REG_ADDR        0x0003
#define ID_REG_ADDR           0x0005
#define GAIN_REG_ADDR         0x0006
#define OFFSET_REG_ADDR       0x0007

#define XIAOHE_DATA_TEST         0

#define VOLTAGE_WHAT_TO_BE_UNPACKED     0b00000001
#define CURRENT_WHAT_TO_BE_UNPACKED     0b00000010
#define POWER_WHAT_TO_BE_UNPACKED       0b00000100
#define ID_WHAT_TO_BE_UNPACKED          0b00001000
#define GAIN_WHAT_TO_BE_UNPACKED        0b00010000
#define OFFSET_WHAT_TO_BE_UNPACKED      0b00100000

#define XIAOHE_CONNECTED_TIMEOUT_MS           1000

    typedef enum {
        Read_Voltage,
        Read_Current,
        Read_Power,
        Read_ID,
        Read_Gain,
        Read_Offset,
        Read_Power_Param,
        Read_Calibration_Param,
        Read_All
    }XiaoHE_Read_Data_Type_t;

    typedef enum {
        Write_Gain,
        Write_Offset
    }XiaoHE_Write_Data_Type_t;

    typedef struct {
        uint8_t   id;           // 萧何一式的ID

        uint8_t   Data_type;    // 需要解包的数据

        uint8_t   error_code;   // 错误码

        uint8_t   tx_requset_msg[REQUEST_MSG_SIZE];

        float     voltage;
        float     current;
        float     power;
        uint16_t  id_fdb;
        float     gain;
        float     offset;

        bool      is_Write_OK;

        uint64_t  connect_tick;   // 连接计时器
        bool      is_Connected;   // 连接状态标志
    } XiaoHE_Msg_INFO_t;

    void XiaoHE_INIT(void);
    void XiaoHE_Data_Clear(uint8_t id);

#if XIAOHE_DATA_TEST
    void XiaoHE_Tx_Test_Read_Req(uint8_t id);
#endif

    void XiaoHE_Read_Cmd(uint8_t id, XiaoHE_Read_Data_Type_t type);

    void XiaoHE_Data_Process(uint8_t *Buffer, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_CHASSIS_XPP_DRV_XIAOHE_H