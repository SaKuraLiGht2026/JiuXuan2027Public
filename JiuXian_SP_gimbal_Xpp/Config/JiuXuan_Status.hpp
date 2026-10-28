//
// Created by ASUS on 2026/8/18.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_JIUXUAN_STATUS_HPP
#define JIUXIAN_SP_GIMBAL_XPP_JIUXUAN_STATUS_HPP

#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define IsTrue(input, compare, output) (input == compare ? output : 0)

/* ==================================================================== */
/**
 * @brief 九玄语音包
 */
typedef enum {
    Waiting,
    Soldier,
    Idol
}JiuXuan_Voice_Mode_t;

/**
 * @brief 九玄语音模式
 */
typedef enum {
    No_Voice,
    IDLE_Voice,
    War_Voice
}JiuXuan_Voice_t;
/* ==================================================================== */

/**
 * @brief 九玄工作模式
 */
typedef enum {
    Work_Mode_Default,
    Free,
    War
}JiuXuan_Work_Mode_t;

/**
 * @brief 九玄兵种选择
 */
typedef enum {
    Troop_Default,
    Infantry,
    Sentry
}JiuXuan_Troop_t;

/**
 * @brief 九玄步兵模式控制端选择
 */
typedef enum {
    NULL_Ctrl,
    JiuXuan_Remote_Ctrl,
    JiuXuan_RM_Client_Ctrl
}JiuXuan_Infantry_Ctrl_Mode_t;

/**
 * @brief 九玄摩擦轮开关
 */
typedef enum {
    Friction_OFF,
    Friction_ON
}JiuXuan_Friction_Switch_t;

/**
 * @brief 九玄火控开关
 */
typedef enum {
    Fire_OFF,
    Fire_ON
}JiuXuan_Fire_Switch_t;

/**
 * @brief 九玄发射模式
 */
typedef enum {
    Disable_Fire,
    Semi_Auto,
    Full_Auto
}JiuXuan_Fire_Mode_t;

typedef struct {
    bool isRMClientFricON;
    bool isRMClientGyroON;
    bool isRMClientSemiON;

    float speed_gain_x;
    float speed_gain_y;
}JiuXuan_RM_Client_Info_t;

/*  */
typedef struct {
    JiuXuan_Voice_Mode_t            jiu_xuan_voice_mode;
    JiuXuan_Voice_t                 jiu_xuan_voice;
    JiuXuan_Work_Mode_t             jiu_xuan_work_mode;
    JiuXuan_Troop_t                 jiu_xuan_troop;
    JiuXuan_Infantry_Ctrl_Mode_t    jiu_xuan_infantry_ctrl_mode;
    JiuXuan_Friction_Switch_t       jiu_xuan_friction_switch;
    JiuXuan_Fire_Switch_t           jiu_xuan_fire_switch;
    JiuXuan_Fire_Mode_t             jiu_xuan_fire_mode;

    JiuXuan_RM_Client_Info_t        jiu_xuan_rm_client_info;
}JiuXuan_Menu_Info_t;
/*  */

#endif //JIUXIAN_SP_GIMBAL_XPP_JIUXUAN_STATUS_HPP