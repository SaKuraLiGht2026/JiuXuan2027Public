//
// Created by ASUS on 2026/7/16.
//
#include "../Inc/drv_DJI.h"
/* USER Private include BEGIN */
#include "../../Bsp/FDCAN/Inc/Bsp_fdcan.h"
/* USER Private include END */

/* USER Private variables BEGIN */
float M3508_P19_rotor_angle_accum[8] = {0};
float M2006_P36_rotor_angle_accum[8] = {0};

uint16_t M3508_P19_last_raw_angle[8] = {0};
uint16_t M2006_P36_last_raw_angle[8] = {0};

uint8_t M3508_P19_first_msg[8]       = {0};
uint8_t M2006_P36_first_msg[8]       = {0};

/**
 * @brief 电机期望输出扭矩数组包
 */
float DJI_M2006_Torq_1_to_4[4];
float DJI_M2006_Torq_5_to_8[4];
float DJI_M3508_Torq_1_to_4[4];
float DJI_M3508_Torq_5_to_8[4];
float DJI_GM6020_Torq_1_to_4[4];
float DJI_GM6020_Torq_5_to_7[4];
float DJI_GM6020_Vol_1_to_4[4];
float DJI_GM6020_Vol_5_to_7[4];

DJI_Msg_INFO_t DJI_M2006_P36_G1;
DJI_Msg_INFO_t DJI_M2006_P36_G2;
DJI_Msg_INFO_t DJI_M3508_P19_G1;
DJI_Msg_INFO_t DJI_M3508_P19_G2;
DJI_Msg_INFO_t DJI_GM6020_G1;
DJI_Msg_INFO_t DJI_GM6020_G2;

/* USER Private variables END */

/* DRV DJI MOTOR INFO INIT BEGIN */
DRV_DJI_StatusTypedef DJI_INIT() {
    DJI_M2006_P36_G1.motor_model = DJI_M2006_P36;
    DJI_M2006_P36_G2.motor_model = DJI_M2006_P36;

    DJI_M3508_P19_G1.motor_model = DJI_M3508_P19;
    DJI_M3508_P19_G2.motor_model = DJI_M3508_P19;

    DJI_GM6020_G1.motor_model    = DJI_GM6020   ;
    DJI_GM6020_G2.motor_model    = DJI_GM6020   ;

    for (uint8_t i = 0; i < 8; i++) {
        M2006_P36_first_msg[i] = 1;
        M3508_P19_first_msg[i] = 1;
    }

    return DRV_DJI_OK;
}
/* DRV DJI MOTOR INFO INIT END */

/* USER Private static function BEGIN */
/* 通用电流数据类型转换 ------------------------------------------------------------------------ */
/**
 * @brief 将电机电流从浮点数转换为int16_t
 *
 * @param motor
 * @param current
 * @return int16_t
 */
inline int16_t DJI_Motor_current_float_to_int16(DJI_MOTOR_MODEL motor, float current) {
    int16_t _int16_temp_current;
    switch (motor)
    {
        case DJI_M2006_P36 :
        {
            _int16_temp_current = (int16_t)(current * DJI_M2006_P36_MAX_CTRL_CURRENT_ENCOUNTER / DJI_M2006_P36_MAX_CTRL_CURRENT);
            _int16_temp_current = DJI_Ctrl_Limit(_int16_temp_current, DJI_M2006_P36_MAX_CTRL_CURRENT_ENCOUNTER);
            break;
        }
        case DJI_M3508_P19 :
        {
            _int16_temp_current = (int16_t)(current * DJI_M3508_P19_MAX_CTRL_CURRENT_ENCOUNTER / DJI_M3508_P19_MAX_CTRL_CURRENT);
            _int16_temp_current = DJI_Ctrl_Limit(_int16_temp_current, DJI_M3508_P19_MAX_CTRL_CURRENT_ENCOUNTER);
            break;
        }
        case DJI_GM6020    :
        {
            _int16_temp_current = (int16_t)(current * DJI_GM6020_MAX_CTRL_CURRENT_ENCOUNTER   / DJI_GM6020_MAX_CTRL_CURRENT    );
            _int16_temp_current = DJI_Ctrl_Limit(_int16_temp_current, DJI_GM6020_MAX_CTRL_CURRENT_ENCOUNTER   );
            break;
        }
        default            :
        {
            _int16_temp_current = 0;
            break;
        }
    }
    return _int16_temp_current;
}

/**
 * @brief 将电机电流从int16_t转换为浮点数
 *
 * @param motor
 * @param current
 * @return float
 */
inline float DJI_Motor_current_int16_to_float(DJI_MOTOR_MODEL motor, int16_t current) {
    float _float_temp_current;
    switch (motor)
    {
        case DJI_M2006_P36 :
        {
            _float_temp_current = (float)(current * DJI_M2006_P36_MAX_CTRL_CURRENT / DJI_M2006_P36_MAX_CTRL_CURRENT_ENCOUNTER);
            _float_temp_current = DJI_Ctrl_Limit(_float_temp_current, DJI_M2006_P36_MAX_CTRL_CURRENT);
            break;
        }
        case DJI_M3508_P19 :
        {
            _float_temp_current = (float)(current * DJI_M3508_P19_MAX_CTRL_CURRENT / DJI_M3508_P19_MAX_CTRL_CURRENT_ENCOUNTER);
            _float_temp_current = DJI_Ctrl_Limit(_float_temp_current, DJI_M3508_P19_MAX_CTRL_CURRENT);
            break;
        }
        case DJI_GM6020    :
        {
            _float_temp_current = (float)(current * DJI_GM6020_MAX_CTRL_CURRENT    / DJI_GM6020_MAX_CTRL_CURRENT_ENCOUNTER   );
            _float_temp_current = DJI_Ctrl_Limit(_float_temp_current, DJI_GM6020_MAX_CTRL_CURRENT    );
            break;
        }
        default            :
        {
            _float_temp_current = 0;
            break;
        }
    }
    return _float_temp_current;
}

/* GM6020压控数据类型转换 ---------------------------------------------------------------------- */
/**
 * @brief 将电机电压从浮点数转换为int16_t(仅限GM6020)
 *
 * @param motor
 * @param voltage
 * @return int16_t
 */
inline int16_t DJI_GM6020_voltage_float_to_int16(DJI_MOTOR_MODEL motor, float voltage) {
    int16_t _int16_temp_voltage;
    _int16_temp_voltage = (int16_t)(voltage * DJI_GM6020_MAX_CTRL_VOLTAGE_ENCOUNTER / DJI_GM6020_MAX_CTRL_VOLTAGE);
    _int16_temp_voltage = (motor == DJI_GM6020 ? DJI_Ctrl_Limit(_int16_temp_voltage, DJI_GM6020_MAX_CTRL_VOLTAGE_ENCOUNTER) : 0);
    return _int16_temp_voltage;
}

/**
 * @brief 将电机电压从int16_t转换为浮点数(仅限GM6020，不过一般用不到)
 *
 * @param motor
 * @param voltage
 * @return float
 */
inline float DJI_GM6020_voltage_int16_to_float(DJI_MOTOR_MODEL motor, int16_t voltage) {
    float _float_temp_voltage;
    _float_temp_voltage = (float)(voltage * DJI_GM6020_MAX_CTRL_VOLTAGE / DJI_GM6020_MAX_CTRL_VOLTAGE_ENCOUNTER);
    _float_temp_voltage = (motor == DJI_GM6020 ? DJI_Ctrl_Limit(_float_temp_voltage, DJI_GM6020_MAX_CTRL_VOLTAGE)           : 0);
    return _float_temp_voltage;
}

/* USER Private static function END */

/* USER Private extern function BEGIN */
/**
 * @brief 电机控制扭矩数据内存拷贝
 *
 * @param dji
 * @param in_torque_data
 * @param len
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_OUTPUT_Torq_MEMCPY(DJI_Msg_INFO_t *dji, float *in_torque_data , uint8_t len) {
    if(len > 4)       return DRV_DJI_ERROR;
    else if (len < 4) return DRV_DJI_ERROR;
    else {
        for(uint8_t i = 0; i < len; i++) {
            dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Torq[i] = in_torque_data[i];
        }
        return DRV_DJI_OK;
    }
}

/**
 * @brief 电机控制电流数据内存拷贝
 *
 * @param dji
 * @param in_torque_data
 * @param len
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_OUTPUT_Cur_MEMCPY(DJI_Msg_INFO_t *dji, float *in_torque_data , uint8_t len) {
    if(DJI_OUTPUT_Torq_MEMCPY(dji, in_torque_data, len) != DRV_DJI_OK) return DRV_DJI_ERROR;
    if (len > 4)                                                       return DRV_DJI_ERROR;
    else if (len < 4)                                                  return DRV_DJI_ERROR;
    else {
        switch (dji->motor_model)
        {
            case DJI_M2006_P36 :
            {
                for(uint8_t i = 0; i < len; i++) {
                    dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[i] = dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Torq[i] / DJI_M2006_P36_k_torque;
                }
                break;
            }
            case DJI_M3508_P19 :
            {
                for(uint8_t i = 0; i < len; i++) {
                    dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[i] = dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Torq[i] / DJI_M3508_P19_k_torque;
                }
                break;
            }
            case DJI_GM6020    :
            {
                for(uint8_t i = 0; i < len; i++) {
                    dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[i] = dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Torq[i] / DJI_GM6020_k_torque;
                }
                break;
            }
            default            : return DRV_DJI_ERROR;
        }
        return DRV_DJI_OK;
    }
}

/**
 * @brief 电机控制电压数据内存拷贝
 *
 * @param dji
 * @param in_voltage_data
 * @param len
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_OUTPUT_Vol_MEMCPY(DJI_Msg_INFO_t *dji, float *in_voltage_data , uint8_t len) {
    if(len > 4)       return DRV_DJI_ERROR;
    else if (len < 4) return DRV_DJI_ERROR;
    else {
        for(uint8_t i = 0; i < len; i++) {
            dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Vol[i] = in_voltage_data[i];
        }
        return DRV_DJI_OK;
    }
}

/**
 * @brief 电机控制报文打包
 *
 * @param dji
 * @param data   目标发送数组
 * @param len    目标发送数组长度
 * @param range  发送范围
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_Ctrl_Msg_data_pack(DJI_Msg_INFO_t *dji, uint8_t *data, uint8_t len, DJI_TRANSMIT_RANGE range) {
    // 判断数据长度是否合法
    if(len < 8)      return DRV_DJI_ERROR;
    else if(len > 8) return DRV_DJI_ERROR;
    else {
        // 判断电机型号
        switch (dji->motor_model)
        {
            case DJI_M2006_P36 :
            {
                switch (range)
                {
                    case DJI_M2006_P36_GROUP_1      : dji->DJI_Ctrl_Msg.tx_ID = DJI_M2006_P36_TX_STANDARD_ID_1_to_4     ; break;
                    case DJI_M2006_P36_GROUP_2      : dji->DJI_Ctrl_Msg.tx_ID = DJI_M2006_P36_TX_STANDARD_ID_5_to_8     ; break;
                    default                         : return DRV_DJI_ERROR;
                }
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[0] = DJI_Motor_current_float_to_int16(DJI_M2006_P36, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[0]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[1] = DJI_Motor_current_float_to_int16(DJI_M2006_P36, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[1]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[2] = DJI_Motor_current_float_to_int16(DJI_M2006_P36, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[2]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[3] = DJI_Motor_current_float_to_int16(DJI_M2006_P36, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[3]);
                break;
            }

            case DJI_M3508_P19 :
            {
                switch (range)
                {
                    case DJI_M3508_P19_GROUP_1      : dji->DJI_Ctrl_Msg.tx_ID = DJI_M3508_P19_TX_STANDARD_ID_1_to_4     ; break;
                    case DJI_M3508_P19_GROUP_2      : dji->DJI_Ctrl_Msg.tx_ID = DJI_M3508_P19_TX_STANDARD_ID_5_to_8     ; break;
                    default                         : return DRV_DJI_ERROR;
                }
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[0] = DJI_Motor_current_float_to_int16(DJI_M3508_P19, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[0]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[1] = DJI_Motor_current_float_to_int16(DJI_M3508_P19, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[1]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[2] = DJI_Motor_current_float_to_int16(DJI_M3508_P19, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[2]);
                dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[3] = DJI_Motor_current_float_to_int16(DJI_M3508_P19, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[3]);
                break;
            }

            case DJI_GM6020    :
            {
                switch (range)
                {
                    case DJI_GM6020_Vol_GROUP_1     : dji->DJI_Ctrl_Msg.tx_ID = DJI_GM6020_TX_STANDARD_ID_1_to_4_Voltage; break;
                    case DJI_GM6020_Vol_GROUP_2     : dji->DJI_Ctrl_Msg.tx_ID = DJI_GM6020_TX_STANDARD_ID_5_to_7_Voltage; break;
                    case DJI_GM6020_Cur_GROUP_1     : dji->DJI_Ctrl_Msg.tx_ID = DJI_GM6020_TX_STANDARD_ID_1_to_4_Current; break;
                    case DJI_GM6020_Cur_GROUP_2     : dji->DJI_Ctrl_Msg.tx_ID = DJI_GM6020_TX_STANDARD_ID_5_to_7_Current; break;
                    default                         : return DRV_DJI_ERROR;
                }

                if (range == DJI_GM6020_Vol_GROUP_1 || range == DJI_GM6020_Vol_GROUP_2) {
                    dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[0] = DJI_GM6020_voltage_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Vol[0]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[1] = DJI_GM6020_voltage_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Vol[1]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[2] = DJI_GM6020_voltage_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Vol[2]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[3] = DJI_GM6020_voltage_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Vol[3]);
                }
                else if (range == DJI_GM6020_Cur_GROUP_1 || range == DJI_GM6020_Cur_GROUP_2) {
                    dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[0] = DJI_Motor_current_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[0]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[1] = DJI_Motor_current_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[1]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[2] = DJI_Motor_current_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[2]);
                    dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[3] = DJI_Motor_current_float_to_int16(DJI_GM6020, dji->DJI_Ctrl_Msg.DJI_Motor_Ctrl_Cur[3]);
                }
                else {
                    return DRV_DJI_ERROR;
                }
                break;
            }
            default            : return DRV_DJI_ERROR;
        }

        // 大端整合 + 位对齐
        if (range == DJI_GM6020_Vol_GROUP_1 || range == DJI_GM6020_Vol_GROUP_2) {
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[0] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[0] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[1] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[0])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[2] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[1] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[3] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[1])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[4] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[2] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[5] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[2])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[6] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[3] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[7] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Vol_Ctrl_Msg[3])      & 0xFF;
        }else {
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[0] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[0] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[1] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[0])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[2] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[1] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[3] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[1])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[4] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[2] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[5] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[2])      & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[6] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[3] >> 8) & 0xFF;
            dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[7] = (uint8_t)(dji->DJI_Ctrl_Msg.DJI_Motor_Cur_Ctrl_Msg[3])      & 0xFF;
        }

        // 打包数据
        for (uint8_t i = 0; i < len; i++) {
            data[i] = dji->DJI_Ctrl_Msg.DJI_Ctrl_Msg[i];
        }
    }
    return DRV_DJI_OK;
}

/**
 * @brief 2006电机反馈报文解包
 * @param data
 * @param len
 * @param motor_ID
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_M2006_P36_FeedBack_Msg_UpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID) {
    if(len < 8) return DRV_DJI_ERROR;
    if(len > 8) return DRV_DJI_ERROR;
    switch(motor_ID)
    {
        case DJI_M2006_P36_ID1:
        {
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_tor_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_temp_enc =                                                         0 ;

            /***/
            if (M2006_P36_first_msg[0] == 1) {
                M2006_P36_last_raw_angle[0] = DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos_enc;
                M2006_P36_first_msg[0] = 2;
            }else if (M2006_P36_first_msg[0] == 2) {
                float delta_deg = (float)(DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos_enc - M2006_P36_last_raw_angle[0]) *360.0f / 8192.0f;

                if (delta_deg > 180.0f)          delta_deg -= 360.0f;
                else if (delta_deg < -180.0f)    delta_deg += 360.0f;

                M2006_P36_rotor_angle_accum[0] += delta_deg;
            }

            M2006_P36_last_raw_angle[0] = DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos_enc;

            float output_shaft_angle = fmodf(M2006_P36_rotor_angle_accum[0] / 36.0f, 360.0f);

            if(output_shaft_angle < 0.0f) output_shaft_angle += 360.0f;

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_pos = output_shaft_angle;

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel      =  DJI_LPF(DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel_enc * DJI_M2006_P36_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel_last, 0.6);

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel_last =  DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_vel;

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M2006_P36, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_cur_enc);

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_tor      =  DJI_LPF(DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_cur * DJI_M2006_P36_k_torque * DJI_M2006_P36_Speed_Ratio_inv, DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_tor_last, 0.6);

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_tor_last =  DJI_M2006_P36_G1.DJI_FeedBack_Msg[0].Motor_tor;

            break;
        }
        case DJI_M2006_P36_ID2:
        {
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].Motor_pos_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].Motor_vel_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].Motor_tor_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[1].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID3:
        {
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].Motor_pos_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].Motor_vel_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].Motor_tor_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[2].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID4:
        {
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].Motor_pos_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].Motor_vel_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].Motor_tor_enc  = (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G1.DJI_FeedBack_Msg[3].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID5:
        {
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].Motor_pos_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].Motor_vel_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].Motor_tor_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID6:
        {
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].Motor_pos_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].Motor_vel_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].Motor_tor_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[1].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID7:
        {
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].Motor_pos_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].Motor_vel_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].Motor_tor_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[2].Motor_temp_enc =                                                         0 ;

            break;
        }
        case DJI_M2006_P36_ID8:
        {
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].Motor_pos_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[1]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].Motor_vel_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[3]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].Motor_tor_enc  = (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[5]);
            DJI_M2006_P36_G2.DJI_FeedBack_Msg[3].Motor_temp_enc =                                                         0 ;

            break;
        }

        default            : return DRV_DJI_ERROR;
    }
    return DRV_DJI_OK;
}

/**
 * @brief 3508电机反馈报文解包
 * @param data
 * @param len
 * @param motor_ID
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_M3508_P19_FeedBack_Msg_UpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID) {
    if(len < 8) return DRV_DJI_ERROR;
    if(len > 8) return DRV_DJI_ERROR;
    switch(motor_ID) {
        case DJI_M3508_P19_ID1:
        {
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_pos_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_vel_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_cur_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_temp_enc =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[6];

            // DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_pos      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_vel      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_tor      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_temp     =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID2:
        {
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_pos_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_vel_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_cur_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_temp_enc =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[6];

            // DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_pos      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[0].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_vel      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_tor      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_temp     =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[1].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID3:
        {
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_pos_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_vel_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_cur_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_temp_enc =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[6];

            /**
            * @brief 电机转速映射
            * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
            */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_vel      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_tor      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_temp     =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[2].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID4:
        {
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_pos_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_vel_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_cur_enc  = (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_temp_enc =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[6];

            /**
            * @brief 电机转速映射
            * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
            */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_vel      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_tor      =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_temp     =  DJI_M3508_P19_G1.DJI_FeedBack_Msg[3].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID5:
        {
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M2006_P36_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_pos_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_vel_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_cur_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_temp_enc =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_vel      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_tor      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_temp     =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[0].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID6:
        {
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_pos_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_vel_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_cur_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_temp_enc =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_vel      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_tor      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_temp     =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[1].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID7:
        {
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_pos_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_vel_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_cur_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_temp_enc =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_vel      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_tor      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_temp     =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[2].Motor_temp_enc;

            break;
        }
        case DJI_M3508_P19_ID8:
        {
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_pos_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[1]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_vel_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[3]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_cur_enc  = (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[5]);
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_temp_enc =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机转速映射
             * @attention 注意这个是M3508 P9电机原装减速箱的相关映射！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_vel      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_vel_enc * DJI_M3508_P19_Speed_Ratio * DJI_MOTOR_RPM_TO_RADPS;
            /**
             * @brief 电机转矩电流映射
             * @attention 这个不是输出轴的扭矩！
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_cur      =  DJI_Motor_current_int16_to_float(DJI_M3508_P19, DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_cur_enc);
            /**
             * @brief 电机输出轴转速映射
             */
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_tor      =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_cur * DJI_M3508_P19_k_torque * DJI_M3508_P19_Speed_Ratio_inv;
            DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_temp     =  DJI_M3508_P19_G2.DJI_FeedBack_Msg[3].Motor_temp_enc;

            break;
        }
        default            : return DRV_DJI_ERROR;
    }
    return DRV_DJI_OK;
}

/**
 * @brief 6020电机反馈报文解包
 * @param data
 * @param len
 * @param motor_ID
 * @return DRV_DJI_StatusTypedef
 */
DRV_DJI_StatusTypedef DJI_GM6020_FeedBack_Msg_UnpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID) {
    if(len < 8) return DRV_DJI_ERROR;
    if(len > 8) return DRV_DJI_ERROR;
    switch(motor_ID) {
        case DJI_GM6020_ID1:
        {
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_cur_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp_enc    =  DJI_GM6020_G1.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos         =  DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_vel;

            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_cur_enc);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_tor         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_tor_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_tor_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_tor;
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp        =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp_enc, DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp_last   =  DJI_GM6020_G1.DJI_FeedBack_Msg[0].Motor_temp;

            break;
        }
        case DJI_GM6020_ID2:
        {
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_cur_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp_enc    =  DJI_GM6020_G1.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos         =  DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_cur_enc);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_tor         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_tor_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp        =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp_enc, DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_vel;
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_tor_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_tor;
            DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp_last   =  DJI_GM6020_G1.DJI_FeedBack_Msg[1].Motor_temp;

            break;
        }
        case DJI_GM6020_ID3:
        {
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_cur_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp_enc    =  DJI_GM6020_G1.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos         =  DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_cur_enc);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_tor         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_tor_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp        =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp_enc, DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_vel;
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_tor_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_tor;
            DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp_last   =  DJI_GM6020_G1.DJI_FeedBack_Msg[2].Motor_temp;

            break;
        }
        case DJI_GM6020_ID4:
        {
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[i] = data[i];
            }
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_cur_enc     = (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp_enc    =  DJI_GM6020_G1.DJI_FeedBack_Msg[3].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos         =  DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_cur_enc);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_tor         =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_tor_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp        =  DJI_LPF(DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp_enc, DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp_last, 0.6);
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_vel;
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_tor_last    =  DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_tor;
            DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp_last   =  DJI_GM6020_G1.DJI_FeedBack_Msg[3].Motor_temp;

            break;
        }
        case DJI_GM6020_ID5:
        {
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_pos_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_cur_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp_enc    =  DJI_GM6020_G2.DJI_FeedBack_Msg[0].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_pos         =  DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_cur_enc);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp        =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp_enc, DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_vel;
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_tor;
            DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp_last   =  DJI_GM6020_G2.DJI_FeedBack_Msg[0].Motor_temp;

            break;
        }
        case DJI_GM6020_ID6:
        {
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_cur_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp_enc    =  DJI_GM6020_G2.DJI_FeedBack_Msg[1].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos         =  DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_cur_enc);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_tor         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_tor_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp        =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp_enc, DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_vel;
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_tor_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_tor;
            DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp_last   =  DJI_GM6020_G2.DJI_FeedBack_Msg[1].Motor_temp;

            break;
        }
        case DJI_GM6020_ID7:
        {
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].motor_ID = motor_ID;
            for(uint8_t i = 0; i < len; i++) {
                DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[i] = data[i];
            }

            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[0] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[1]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[2] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[3]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_cur_enc     = (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[4] << 8) |
                                                                  (DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[5]);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp_enc    =  DJI_GM6020_G2.DJI_FeedBack_Msg[2].DJI_FeedBack_Msg[6];

            /**
             * @brief 电机编码器值转浮点型
             */
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos         =  DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_pos_enc * 360.0f / (DJI_MOTOR_POS_MAX_ENCOUNTER - DJI_MOTOR_POS_MIN_ENCOUNTER);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel_enc * DJI_MOTOR_RPM_TO_RADPS, DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_cur         =  DJI_Motor_current_int16_to_float(DJI_GM6020, DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_cur_enc);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_tor         =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_cur     * DJI_GM6020_k_torque, DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_tor_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp        =  DJI_LPF(DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp_enc, DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp_last, 0.6);
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_vel;
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_tor_last    =  DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_tor;
            DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp_last   =  DJI_GM6020_G2.DJI_FeedBack_Msg[2].Motor_temp;

            break;
        }
        default            : return DRV_DJI_ERROR;
    }
    return DRV_DJI_OK;
}

/**
 * @brief 简单发送报文拷贝测试
 * @param dji
 * @param tx_ID
 * @param data
 * @param IN1
 * @param IN2
 * @param IN3
 * @param IN4
 * @return
 */
DRV_DJI_StatusTypedef Simple_CAN_data_pack(DJI_Msg_INFO_t *dji, uint32_t tx_ID, uint8_t *data, int16_t IN1, int16_t IN2, int16_t IN3, int16_t IN4) {
	dji->DJI_Ctrl_Msg.tx_ID = tx_ID;

	data[0] = (uint8_t)(IN1 >> 8) & 0xFF;
	data[1] = (uint8_t)(IN1     ) & 0xFF;
	data[2] = (uint8_t)(IN2 >> 8) & 0xFF;
	data[3] = (uint8_t)(IN2     ) & 0xFF;
	data[4] = (uint8_t)(IN3 >> 8) & 0xFF;
	data[5] = (uint8_t)(IN3     ) & 0xFF;
	data[6] = (uint8_t)(IN4 >> 8) & 0xFF;
	data[7] = (uint8_t)(IN4     ) & 0xFF;

	return DRV_DJI_OK;
}

/* USER Private extern function END */

