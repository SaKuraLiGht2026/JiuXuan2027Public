//
// Created by ASUS on 2026/7/16.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_DJI_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_DJI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>

/*  */
extern float DJI_M2006_Torq_1_to_4[4];
extern float DJI_M2006_Torq_5_to_8[4];
extern float DJI_M3508_Torq_1_to_4[4];
extern float DJI_M3508_Torq_5_to_8[4];
extern float DJI_GM6020_Torq_1_to_4[4];
extern float DJI_GM6020_Torq_5_to_7[4];
extern float DJI_GM6020_Vol_1_to_4[4];
extern float DJI_GM6020_Vol_5_to_7[4];
/*  */

#define DJI_LPF(x, x_last, alpha)   (x_last + alpha * (x - x_last))

/* USER Private define BEGIN */
/**
 * @brief 安全限制
 */
#define DJI_Ctrl_Limit(x, limit)            (x > limit ? limit : (x < -limit ? -limit : x))

#define DJI_MOTOR_POS_MAX_ENCOUNTER                     8192
#define DJI_MOTOR_POS_MIN_ENCOUNTER                     0

#define DJI_MOTOR_RPM_TO_RADPS                          0.1047197551f
#define DJI_MOTOR_RADPS_TO_RPM                          9.5492965855f

/**
 * @brief info of M2006 motor
 */
#define DJI_M2006_P36_TX_STANDARD_ID_1_to_4             0x200
#define DJI_M2006_P36_TX_STANDARD_ID_5_to_8             0x1FF

#define DJI_M2006_P36_MAX_CTRL_CURRENT_ENCOUNTER        10000
#define DJI_M2006_P36_MAX_CTRL_CURRENT                  10.0f

#define DJI_M2006_P36_k_torque                          0.18                  //  N*m/A
#define DJI_M2006_P36_k_torque_inv                      5.56

#define DJI_M2006_P36_Speed_Ratio                       0.0277777777777777778f //  从电机本体到输出轴
#define DJI_M2006_P36_Speed_Ratio_inv                   36.0f                  //  从输出轴到电机本体

#define DJI_M2006_P36_ID1                               0x201
#define DJI_M2006_P36_ID2                               0x202
#define DJI_M2006_P36_ID3                               0x203
#define DJI_M2006_P36_ID4                               0x204
#define DJI_M2006_P36_ID5                               0x205
#define DJI_M2006_P36_ID6                               0x206
#define DJI_M2006_P36_ID7                               0x207
#define DJI_M2006_P36_ID8                               0x208

/**
 * @brief info of M3508 motor
 */
#define DJI_M3508_P19_TX_STANDARD_ID_1_to_4             0x200
#define DJI_M3508_P19_TX_STANDARD_ID_5_to_8             0x1FF

#define DJI_M3508_P19_MAX_CTRL_CURRENT_ENCOUNTER        16384
#define DJI_M3508_P19_MAX_CTRL_CURRENT                  20.0f

#define DJI_M3508_P19_k_torque                          0.30                  //  N*m/A
#define DJI_M3508_P19_k_torque_inv                      3.33

#define DJI_M3508_P19_Speed_Ratio                       0.0526315789473684210f //  从电机本体到输出轴
#define DJI_M3508_P19_Speed_Ratio_inv                   19.0f                  //  从输出轴到电机本体

#define DJI_M3508_P19_ID1                               0x201
#define DJI_M3508_P19_ID2                               0x202
#define DJI_M3508_P19_ID3                               0x203
#define DJI_M3508_P19_ID4                               0x204
#define DJI_M3508_P19_ID5                               0x205
#define DJI_M3508_P19_ID6                               0x206
#define DJI_M3508_P19_ID7                               0x207
#define DJI_M3508_P19_ID8                               0x208

/**
 * @brief info of GM6020 motor
 */
/* GM6020 Voltage Ctrl */
#define DJI_GM6020_TX_STANDARD_ID_1_to_4_Voltage        0x1ff
#define DJI_GM6020_TX_STANDARD_ID_5_to_7_Voltage        0x2ff

#define DJI_GM6020_MAX_CTRL_VOLTAGE_ENCOUNTER           25000
#define DJI_GM6020_MAX_CTRL_VOLTAGE                     25.0f

/* GM6020 Current Ctrl */
#define DJI_GM6020_TX_STANDARD_ID_1_to_4_Current        0x1fe
#define DJI_GM6020_TX_STANDARD_ID_5_to_7_Current        0x2fe

#define DJI_GM6020_MAX_CTRL_CURRENT_ENCOUNTER           16384
#define DJI_GM6020_MAX_CTRL_CURRENT                     3.0f

#define DJI_GM6020_k_torque                             0.741  //  N*m/A
#define DJI_GM6020_k_torque_inv                         1.345

#define DJI_GM6020_ID1                                  0x205
#define DJI_GM6020_ID2                                  0x206
#define DJI_GM6020_ID3                                  0x207
#define DJI_GM6020_ID4                                  0x208
#define DJI_GM6020_ID5                                  0x209
#define DJI_GM6020_ID6                                  0x20A
#define DJI_GM6020_ID7                                  0x20B
/* USER Private define END */

/* USER Private enum BEGIN */

typedef enum{
    DRV_DJI_OK              = 0x00U,
    DRV_DJI_ERROR           = 0x01U,
    DRV_DJI_BUSY            = 0x02U,
    DRV_DJI_TIMEOUT         = 0x03U
}DRV_DJI_StatusTypedef;

/**
 * @brief 电机类型
 */
typedef enum{
    DJI_M2006_P36           = 0x00U,
    DJI_M3508_P19           = 0x01U,
    DJI_GM6020              = 0x02U
}DJI_MOTOR_MODEL;

typedef enum{
    DJI_M2006_P36_GROUP_1   = 0x00U,
    DJI_M2006_P36_GROUP_2   = 0x01U,

    DJI_M3508_P19_GROUP_1   = 0x02U,
    DJI_M3508_P19_GROUP_2   = 0x03U,

    DJI_GM6020_Vol_GROUP_1  = 0x04U,
    DJI_GM6020_Vol_GROUP_2  = 0x05U,

    DJI_GM6020_Cur_GROUP_1  = 0x06U,
    DJI_GM6020_Cur_GROUP_2  = 0x07U
}DJI_TRANSMIT_RANGE;

/**
 * @brief     是否有减速箱
 * @attention 仅针对M2006 P36和M3508 P19
 */
typedef enum{
    MOTOR_WITH_GEARBOX      = 0x00U,
    MOTOR_WITHOUT_GEARBOX   = 0x01U
}DJI_GEARBOX_STATUS;

/* USER Private enum END */

/* USER Private struct BEGIN */

typedef struct{
    uint32_t              tx_ID;
    float                 DJI_Motor_Ctrl_Torq[4];
    float                 DJI_Motor_Ctrl_Cur[4];
    float                 DJI_Motor_Ctrl_Vol[4];
    int16_t               DJI_Motor_Cur_Ctrl_Msg[4];
    int16_t               DJI_Motor_Vol_Ctrl_Msg[4];
    uint8_t               DJI_Ctrl_Msg[8];
}DJI_Ctrl_Msg_INFO_t;

typedef struct
{
    uint32_t        motor_ID;
    uint8_t         DJI_FeedBack_Msg[8];

    int16_t         Motor_pos_enc;
    int16_t         Motor_vel_enc;
    int16_t         Motor_cur_enc;
    int16_t         Motor_tor_enc;    //M2006_P36
    int16_t         Motor_temp_enc;

    float           Motor_pos;
    float           Motor_vel;
    float           Motor_cur;
    float           Motor_tor;
    float           Motor_temp;

    float           Motor_vel_last;
    float           Motor_tor_last;
    float           Motor_temp_last;
}DJI_FeedBack_Msg_INFO_t;

typedef struct{
    DJI_MOTOR_MODEL            motor_model;

    DJI_Ctrl_Msg_INFO_t        DJI_Ctrl_Msg;
    DJI_FeedBack_Msg_INFO_t    DJI_FeedBack_Msg[4];
}DJI_Msg_INFO_t;

/* USER Private struct END */

/* USER Private function BEGIN */
extern DRV_DJI_StatusTypedef DJI_INIT();

int16_t DJI_Motor_current_float_to_int16(DJI_MOTOR_MODEL motor, float current);
float DJI_Motor_current_int16_to_float(DJI_MOTOR_MODEL motor, int16_t current);

int16_t DJI_GM6020_voltage_float_to_int16(DJI_MOTOR_MODEL motor, float voltage);
float DJI_GM6020_voltage_int16_to_float(DJI_MOTOR_MODEL motor, int16_t voltage);

extern DRV_DJI_StatusTypedef DJI_OUTPUT_Torq_MEMCPY(DJI_Msg_INFO_t *dji, float *in_torque_data , uint8_t len);
extern DRV_DJI_StatusTypedef DJI_OUTPUT_Cur_MEMCPY(DJI_Msg_INFO_t *dji, float *in_torque_data , uint8_t len);
extern DRV_DJI_StatusTypedef DJI_OUTPUT_Vol_MEMCPY(DJI_Msg_INFO_t *dji, float *in_voltage_data , uint8_t len);

extern DRV_DJI_StatusTypedef DJI_Ctrl_Msg_data_pack(DJI_Msg_INFO_t *dji, uint8_t *data, uint8_t len, DJI_TRANSMIT_RANGE range);

extern DRV_DJI_StatusTypedef Simple_CAN_data_pack(DJI_Msg_INFO_t *dji, uint32_t tx_ID, uint8_t *data, int16_t IN1, int16_t IN2, int16_t IN3, int16_t IN4);

extern DRV_DJI_StatusTypedef DJI_M2006_P36_FeedBack_Msg_UpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID);
extern DRV_DJI_StatusTypedef DJI_M3508_P19_FeedBack_Msg_UpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID);
extern DRV_DJI_StatusTypedef DJI_GM6020_FeedBack_Msg_UnpackingCallBack(uint8_t *data, uint8_t len, uint32_t motor_ID);

/* USER Private function END */

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_DJI_H