//
// Created by ASUS on 2026/10/3.
//
#include "../Inc/drv_XiaoHe.h"

XiaoHE_Msg_INFO_t XiaoHE[8];

static uint16_t XiaoHE_Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;   /* 发送时低字节在前！ */
}

static uint16_t Data_Unpack(uint8_t id, const uint8_t *data, uint8_t Tag) {
    if (Tag == 0) {
        XiaoHE[id - 1].voltage = static_cast<float>READ_U16_BE(data) / 1000.0f;
        return 2;
    }else if (Tag == 1) {
        XiaoHE[id - 1].current = static_cast<float>READ_S32_BE(data) / 1000.0f;
        return 4;
    }else if (Tag == 2) {
        XiaoHE[id - 1].power   = static_cast<float>READ_S32_BE(data) / 1000.0f;
        return 4;
    }else if (Tag == 3) {
        XiaoHE[id - 1].id_fdb  =                   READ_U16_BE(data);
        return 2;
    }else if (Tag == 4) {
        XiaoHE[id - 1].gain    = static_cast<float>READ_U16_BE(data) / 1.0f;
        return 2;
    }else if (Tag == 5) {
        XiaoHE[id - 1].offset  = static_cast<float>READ_S16_BE(data) / 1.0f;
        return 2;
    }else {
        return 0;
    }
}

/* ============================================================================================================================================ */

/* Read */

static void XiaoHE_Read_Voltage_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = VOLTAGE_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (VOLTAGE_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = VOLTAGE_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x01;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Current_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = CURRENT_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (CURRENT_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = CURRENT_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x02;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Power_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = POWER_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (POWER_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = POWER_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x02;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_ID_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = ID_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (ID_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = ID_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x01;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Gain_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = GAIN_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (GAIN_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = GAIN_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x01;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Offset_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = OFFSET_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (OFFSET_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = OFFSET_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x01;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Power_Param_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = VOLTAGE_WHAT_TO_BE_UNPACKED | CURRENT_WHAT_TO_BE_UNPACKED | POWER_WHAT_TO_BE_UNPACKED | ID_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (VOLTAGE_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = VOLTAGE_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x06;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Read_Calibration_Param_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = GAIN_WHAT_TO_BE_UNPACKED | OFFSET_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (GAIN_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = GAIN_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x02;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void Xiaohe_Read_All_Cmd(uint8_t id) {
    XiaoHE[id - 1].Data_type = VOLTAGE_WHAT_TO_BE_UNPACKED | CURRENT_WHAT_TO_BE_UNPACKED | POWER_WHAT_TO_BE_UNPACKED | ID_WHAT_TO_BE_UNPACKED | GAIN_WHAT_TO_BE_UNPACKED | OFFSET_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (VOLTAGE_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = VOLTAGE_REG_ADDR & 0xFF;

    XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
    XiaoHE[id - 1].tx_requset_msg[5] = 0x08;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

/* Write */

static void XiaoHE_Write_Gain_Cmd(uint8_t id, float gain) {
    XiaoHE[id - 1].Data_type = GAIN_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_WRITE_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (GAIN_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = GAIN_REG_ADDR & 0xFF;

    uint16_t gain_value = static_cast<uint16_t>(gain * 1.0f);
    gain_value = (gain_value > 1500 ? 1500 : (gain_value < 500 ? 500 : gain_value));
    XiaoHE[id - 1].tx_requset_msg[4] = (gain_value >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[5] = gain_value & 0xFF;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

static void XiaoHE_Write_Offset_Cmd(uint8_t id, float offset) {
    XiaoHE[id - 1].Data_type = OFFSET_WHAT_TO_BE_UNPACKED;

    XiaoHE[id - 1].tx_requset_msg[0] = id;
    XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_WRITE_CODE;

    XiaoHE[id - 1].tx_requset_msg[2] = (OFFSET_REG_ADDR >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[3] = OFFSET_REG_ADDR & 0xFF;

    int16_t offset_value = static_cast<int16_t>(offset * 1.0f);
    offset_value = (offset_value > 500 ? 500 : (offset_value < -500 ? -500 : offset_value));
    XiaoHE[id - 1].tx_requset_msg[4] = (offset_value >> 8) & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[5] = offset_value & 0xFF;

    uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

    XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
    XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
}

/* ============================================================================================================================================ */

void XiaoHE_INIT(void) {
    for (uint8_t i = 0; i < 8; i++) {
        XiaoHE[i].id         = i + 1;

        XiaoHE[i].Data_type  = 0x00;
        XiaoHE[i].error_code = 0x00;

        XiaoHE[i].tx_requset_msg[0] = 0xAA;
        XiaoHE[i].tx_requset_msg[1] = 0x55;
        XiaoHE[i].tx_requset_msg[2] = 0x01;
        XiaoHE[i].tx_requset_msg[3] = 0x00;
        XiaoHE[i].tx_requset_msg[4] = 0x00;
        XiaoHE[i].tx_requset_msg[5] = 0x00;
        XiaoHE[i].tx_requset_msg[6] = 0x00;
        XiaoHE[i].tx_requset_msg[7] = 0x00;

        XiaoHE[i].voltage = 0.0f;
        XiaoHE[i].current = 0.0f;
        XiaoHE[i].power   = 0.0f;
        XiaoHE[i].id_fdb  = 0x0000;
        XiaoHE[i].gain    = 0.0f;
        XiaoHE[i].offset  = 0.0f;

        XiaoHE[i].is_Write_OK = false;

        XiaoHE[i].connect_tick = 0;
        XiaoHE[i].is_Connected = false;
    }
}

void XiaoHE_Data_Clear(uint8_t id) {
    XiaoHE[id - 1].Data_type  = 0x00;
    XiaoHE[id - 1].error_code = 0x00;

    XiaoHE[id - 1].voltage = 0.0f;
    XiaoHE[id - 1].current = 0.0f;
    XiaoHE[id - 1].power   = 0.0f;
    XiaoHE[id - 1].id_fdb  = 0x0000;
    XiaoHE[id - 1].gain    = 0.0f;
    XiaoHE[id - 1].offset  = 0.0f;

    XiaoHE[id - 1].is_Write_OK = false;

    XiaoHE[id - 1].connect_tick = XIAOHE_CONNECTED_TIMEOUT_MS;
    XiaoHE[id - 1].is_Connected = false;
}

#if XIAOHE_DATA_TEST
    void XiaoHE_Tx_Test_Read_Req(uint8_t id) {
        XiaoHE[id - 1].Data_type = VOLTAGE_WHAT_TO_BE_UNPACKED | CURRENT_WHAT_TO_BE_UNPACKED | POWER_WHAT_TO_BE_UNPACKED | ID_WHAT_TO_BE_UNPACKED | GAIN_WHAT_TO_BE_UNPACKED | OFFSET_WHAT_TO_BE_UNPACKED;

        XiaoHE[id - 1].tx_requset_msg[0] = id;
        XiaoHE[id - 1].tx_requset_msg[1] = XIAOHE_READ_CODE;

        XiaoHE[id - 1].tx_requset_msg[2] = 0x00;
        XiaoHE[id - 1].tx_requset_msg[3] = 0x00;

        XiaoHE[id - 1].tx_requset_msg[4] = 0x00;
        XiaoHE[id - 1].tx_requset_msg[5] = 0x08;

        uint16_t crc = XiaoHE_Modbus_CRC16(XiaoHE[id - 1].tx_requset_msg, REQUEST_MSG_SIZE - 2);

        XiaoHE[id - 1].tx_requset_msg[6] = crc & 0xFF;
        XiaoHE[id - 1].tx_requset_msg[7] = (crc >> 8) & 0xFF;
    }
#endif

/**
 * @brief 萧何一式读取命令
 * @param id
 * @param type
 */
void XiaoHE_Read_Cmd(uint8_t id, XiaoHE_Read_Data_Type_t type) {
    for (uint8_t i = 0; i < 7; i++) {
        XiaoHE[i].connect_tick++;
        if (XiaoHE[i].connect_tick > XIAOHE_CONNECTED_TIMEOUT_MS) {
            XiaoHE[i].connect_tick = XIAOHE_CONNECTED_TIMEOUT_MS;
            XiaoHE[i].is_Connected = false;
        }

        if (XiaoHE[i].is_Connected == false) {
            XiaoHE_Data_Clear(i + 1);
        }
    }

    switch (type) {
        case Read_Voltage:
            XiaoHE_Read_Voltage_Cmd(id);
            break;
        case Read_Current:
            XiaoHE_Read_Current_Cmd(id);
            break;
        case Read_Power:
            XiaoHE_Read_Power_Cmd(id);
            break;
        case Read_ID:
            XiaoHE_Read_ID_Cmd(id);
            break;
        case Read_Gain:
            XiaoHE_Read_Gain_Cmd(id);
            break;
        case Read_Offset:
            XiaoHE_Read_Offset_Cmd(id);
            break;
        case Read_Power_Param:
            XiaoHE_Read_Power_Param_Cmd(id);
            break;
        case Read_Calibration_Param:
            XiaoHE_Read_Calibration_Param_Cmd(id);
            break;
        case Read_All:
            Xiaohe_Read_All_Cmd(id);
            break;
        default:
            break;
    }
}

/**
 * @brief 萧何一式写入命令
 * @param id
 * @param type
 * @param value
 */
void XiaoHE_Write_Cmd(uint8_t id, XiaoHE_Write_Data_Type_t type, float value) {
    switch (type) {
        case Write_Gain:
            XiaoHE_Write_Gain_Cmd(id, value);
            break;
        case Write_Offset:
            XiaoHE_Write_Offset_Cmd(id, value);
            break;
        default:
            break;
    }
}

/**
 * @brief 解析萧何一式数据
 * @param Buffer
 * @param Length
 */
void XiaoHE_Data_Process(uint8_t *Buffer, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++) {
        // 检查从机地址是否合法
        if (Buffer[i] != 0x01 && Buffer[i] != 0x02 && Buffer[i] != 0x03 && Buffer[i] != 0x04 && Buffer[i] != 0x05 && Buffer[i] != 0x06 && Buffer[i] != 0x07 && Buffer[i] != 0x08) {
            continue;
        }

        uint8_t slave_id = Buffer[i];

        // 检查功能码是否合法
        if (Buffer[i + 1] != XIAOHE_READ_CODE) {
            if (Buffer[i + 1] == XIAOHE_ERROR_CODE) {
                // 错误帧长度不足以进行CRC校验，直接跳过
                if (i + 3 > Length) {
                    break;
                }

                uint16_t crc16_md = Buffer[i + 3] | (Buffer[i + 4] << 8);

                if (crc16_md != XiaoHE_Modbus_CRC16(&Buffer[i], 3)) {
                    // CRC校验失败，跳过该帧
                    continue;
                }

                XiaoHE[slave_id - 1].error_code = Buffer[i + 2];

            } else if (Buffer[i + 1] == XIAOHE_WRITE_CODE) {

                if (i + 6 > Length) {
                    XiaoHE[slave_id - 1].is_Write_OK = false;
                    break;
                }

                uint16_t WregAddr = Buffer[i + 2] << 8 | Buffer[i + 3];

                if (WregAddr != GAIN_REG_ADDR && WregAddr != OFFSET_REG_ADDR) {
                    // 非法寄存器地址，跳过该帧
                    XiaoHE[slave_id - 1].is_Write_OK = false;
                    continue;
                }

                uint16_t crc16_md = Buffer[i + 6] | (Buffer[i + 7] << 8);

                if (crc16_md != XiaoHE_Modbus_CRC16(&Buffer[i], 6)) {
                    // CRC校验失败，跳过该帧
                    XiaoHE[slave_id - 1].is_Write_OK = false;
                    continue;
                }

                XiaoHE[slave_id - 1].is_Write_OK = true;

            } else {
                // 非法功能码，跳过该帧
                continue;
            }
        }

        uint16_t len = 3 + Buffer[i + 2];
        uint16_t total_len = len + 2; // 数据长度 + CRC长度

        if (i + len > Length) {
            // 数据长度不足以进行CRC校验，直接跳过
            break;
        }

        uint16_t crc16_md = Buffer[i + total_len - 2] | (Buffer[i + total_len - 1] << 8);

        if (crc16_md != XiaoHE_Modbus_CRC16(&Buffer[i], len)) {
            // CRC校验失败，跳过该帧
            continue;
        }

        if (Buffer[i + 2] != (XiaoHE[slave_id -1].Data_type & VOLTAGE_WHAT_TO_BE_UNPACKED ? 2 : 0) +
                             (XiaoHE[slave_id -1].Data_type & CURRENT_WHAT_TO_BE_UNPACKED ? 4 : 0) +
                             (XiaoHE[slave_id -1].Data_type & POWER_WHAT_TO_BE_UNPACKED   ? 4 : 0) +
                             (XiaoHE[slave_id -1].Data_type & ID_WHAT_TO_BE_UNPACKED      ? 2 : 0) +
                             (XiaoHE[slave_id -1].Data_type & GAIN_WHAT_TO_BE_UNPACKED    ? 2 : 0) +
                             (XiaoHE[slave_id -1].Data_type & OFFSET_WHAT_TO_BE_UNPACKED  ? 2 : 0)
                                                                                                     ) {
            // 数据长度不匹配，跳过该帧
            continue;
        }

        uint16_t byte_index = 3; // 数据起始索引

        for (uint8_t j = 0; j < 8; j++) {
            if (XiaoHE[slave_id - 1].Data_type & (1 << j)) {
                byte_index += Data_Unpack(slave_id, &Buffer[i + byte_index], j);
            }
        }

        if (XiaoHE[slave_id -1].id == XiaoHE[slave_id -1].id_fdb) {
            XiaoHE[slave_id - 1].connect_tick = 0;
            XiaoHE[slave_id - 1].is_Connected = true;
        }
    }
}
