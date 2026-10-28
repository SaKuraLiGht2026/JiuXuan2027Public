//
// Created by ASUS on 2026/7/30.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_REFEREE_INFO_DEFINE_H
#define JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_REFEREE_INFO_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define REFEREE_STANDARD_LINK_HEADER_SOF     0xA5
#define REFEREE_VTM_LINK_HEADER_241225_SOF   0xA5
#define REFEREE_VTM_LINK_HEADER_SOF          0xA9

#define VTM_VT02_Ver_20241225 1

/* Robot ID */
#define RED_HERO                                1
#define RED_ENGINEER                            2
#define RED_INFANTRY_3                          3
#define RED_INFANTRY_4                          4
#define RED_INFANTRY_5                          5
#define RED_AERIAL                              6
#define RED_SENTRY                              7
#define RED_DART                                8
#define RED_RADAR                               9
#define RED_BASE                               10
#define RED_OUTPOST                            11
#define BLUE_HERO                             101
#define BLUE_ENGINEER                         102
#define BLUE_INFANTRY_3                       103
#define BLUE_INFANTRY_4                       104
#define BLUE_INFANTRY_5                       105
#define BLUE_AERIAL                           106
#define BLUE_SENTRY                           107
#define BLUE_DART                             108
#define BLUE_RADAR                            109
#define BLUE_BASE                             110
#define BLUE_OUTPOST                          111

/* Client ID */
#define RED_HERO_CLIENT                      0x0101
#define RED_ENGINEER_CLIENT                  0x0102
#define RED_INFANTRY_3_CLIENT                0x0103
#define RED_INFANTRY_4_CLIENT                0x0104
#define RED_INFANTRY_5_CLIENT                0x0105
#define RED_AERIAL_CLIENT                    0x0106
#define BLUE_HERO_CLIENT                     0x0165
#define BLUE_ENGINEER_CLIENT                 0x0166
#define BLUE_INFANTRY_3_CLIENT               0x0167
#define BLUE_INFANTRY_4_CLIENT               0x0168
#define BLUE_INFANTRY_5_CLIENT               0x0169
#define BLUE_AERIAL_CLIENT                   0x016A
#define REFEREE_SYSTEM_SERVER                0x8080


#define Referee_uint                         unsigned int
#define Referee_uint8                        uint8_t
#define Referee_uint16                       uint16_t
#define Referee_uint32                       uint32_t
#define Referee_int                          int
#define Referee_int8                         int8_t
#define Referee_int16                        int16_t
#define Referee_int32                        int32_t
#define Referee_float                        float
#define Referee_uchar                        unsigned char
#define Referee_action                       void
#define Referee_ushort                       unsigned short

#define  __FALSE                   (uint32_t)0x00000000
#define  __TRUE                    (uint32_t)0x00000001

/* RoboMaster cmd code */
/* Standard Link ------------------------------------------------------------- */
#define GAME_STATUS                                 0x0001  /* RX */
#define GAME_RESULT                                 0x0002  /* RX */
#define GAME_ROBOT_HP                               0x0003  /* RX */
#define EVENT_DATA                                  0x0101  /* RX */
#define REFEREE_WARNING                             0x0104  /* RX */
#define DART_INFO                                   0x0105  /* RX */
#define ROBOT_STATUS                                0x0201  /* RX */
#define POWER_HEAT_DATA                             0x0202  /* RX */
#define ROBOT_POS                                   0x0203  /* RX */
#define BUFF                                        0x0204  /* RX */
#define HURT_DATA                                   0x0206  /* RX */
#define SHOOT_DATA                                  0x0207  /* RX */
#define PROJECTILE_ALLOWANCE                        0x0208  /* RX */
#define RFID_STATUS                                 0x0209  /* RX */
#define DART_CLIENT_CMD                             0x020A  /* RX */
#define GROUND_ROBOT_POSITION                       0x020B  /* RX */
#define RADAR_MARK_DATA                             0x020C  /* RX */
#define SENTRY_INFO                                 0x020D  /* RX */
#define RADAR_INFO                                  0x020E  /* RX */
#define ROBOT_INTERACTION_DATA                      0x0301  /* TX */  /* RX */
#define MAP_COMMAND                                 0x0303  /* RX */
#define MAP_ROBOT_DATA                              0x0305  /* TX */  /* Radar */
#define MAP_DATA                                    0x0307  /* TX */
#define CUSTOM_INFO                                 0x0308  /* TX */
/* 0x0301 Subcontent ID */
#define ROBOT_INTERACTION_ID_MIN                    0x0200  /* TX */  /* RX */
#define ROBOT_INTERACTION_ID_MAX                    0x02FF  /* TX */  /* RX */
#define DELETE_CLIENT_UI_LAYER                      0x0100  /* TX */
#define CREATE_ONE_CLIENT_UI_LAYER                  0x0101  /* TX */
#define CREATE_TWO_CLIENT_UI_LAYER                  0x0102  /* TX */
#define CREATE_FIVE_CLIENT_UI_LAYER                 0x0103  /* TX */
#define CREATE_SEVEN_CLIENT_UI_LAYER                0x0104  /* TX */
#define CREATE_CHARACTER_CLIENT_UI_LAYER            0x0110  /* TX */
#define SENTRY_SELF_DECISIONS_ID                    0x0120  /* TX */
#define RADAR_SELF_DECISIONS_ID                     0x0121  /* TX */

/* VTM Link ------------------------------------------------------------------ */
#define CUSTOM_ROBOT_DATA                           0x0302  /* RX */
#define REMOTE_CONTROL_DATA_241225                  0x0304  /* RX */
#define ROBOT_CUSTOM_DATA                           0x0309  /* TX */
#define SELF_DEFINE_CLIENT_ROBOT_CUSTOM_DATA        0x0310  /* TX */
#define SELF_DEFINE_CLIENT_CUSTOM_ROBOT_DATA        0x0311  /* RX */
/* --------------------------------------------------------------------------- */

/* RoboMaster cmd data frame length */
/* Standard Link ------------------------------------------------------------- */
#define DATA_LEN_0x0001                                11
#define DATA_LEN_0x0002                                 1
#define DATA_LEN_0x0003                                20
#define DATA_LEN_0x0101                                 4
#define DATA_LEN_0x0104                                 3
#define DATA_LEN_0x0105                                 3
#define DATA_LEN_0x0201                                17
#define DATA_LEN_0x0202                                14
#define DATA_LEN_0x0203                                16
#define DATA_LEN_0x0204                                 8
#define DATA_LEN_0x0206                                 1
#define DATA_LEN_0x0207                                 7
#define DATA_LEN_0x0208                                 8
#define DATA_LEN_0x0209                                 5
#define DATA_LEN_0x020A                                 6
#define DATA_LEN_0x020B                                40
#define DATA_LEN_0x020C                                 2
#define DATA_LEN_0x020D                                14
#define DATA_LEN_0x020E                                 1
#define DATA_LEN_0x0301_MAX                           118
#define DATA_LEN_0x0303                                15
#define DATA_LEN_0x0305                                48
#define DATA_LEN_0x0307                               103
#define DATA_LEN_0x0308                                34

/* VTM Link ------------------------------------------------------------------ */
#define DATA_LEN_0x0302                                30
#define DATA_LEN_0x0304                                12  // VT02 Info
#define DATA_LEN_0x0309                                30
#define DATA_LEN_0x0310                               300
#define DATA_LEN_0x0311                                30
/* --------------------------------------------------------------------------- */

/* RoboMaster Referee System Info Enum --------------------------------------- */
    typedef enum {
        RMUC               = 1,
        RMUL_3V3           = 2,
        RMUL_INFANTRY_SOLO = 3,
        TYPE_DEFAULT
    }ROBOMASTER_GAME_TYPE_t;

    typedef enum {
        RED  = 0x01U,
        BLUE = 0x02U,
        FACTION_DEFAULT
    }ROBOT_FACTION_t;

    typedef enum {
        HERO_ROBOT     = 0x01U,
        ENGINEER_ROBOT = 0x02U,
        INFANTRY_ROBOT        ,
        AERIAL_ROBOT   = 0x06U,
        SENTRY_ROBOT   = 0x07U,
        DART_SYSTEM    = 0x08U,
        RADAR_SYSTEM   = 0x09U,
        ROBOT_DEFAULT
    }ROBOT_ARMS;

    typedef enum {
        DRAW,
        WIN,
        LOST,
        GAME_DEFAULT
    }GAME_RESULT_INFO_t;

/* RoboMaster Referee System Info Struct ------------------------------------- */
    typedef struct __attribute__((packed))
    {
        uint8_t  game_type : 4;
        uint8_t  game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    }game_status_t;

    typedef struct __attribute__((packed))
    {
        uint8_t  winner;
    }game_result_t;

    typedef struct __attribute__((packed))
    {
        uint16_t ally_1_robot_HP;
        uint16_t ally_2_robot_HP;
        uint16_t ally_3_robot_HP;
        uint16_t ally_4_robot_HP;
        int16_t  damage_difference;
        uint16_t ally_7_robot_HP;
        uint16_t ally_outpost_HP;
        uint16_t ally_base_HP;
        uint16_t enemy_outpost_HP;
        uint16_t enemy_base_HP;
    }game_robot_HP_t;

    typedef struct __attribute__((packed))
    {
        uint32_t event_data;
    }event_data_t;

    typedef struct __attribute__((packed))
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    }referee_warning_t;

    typedef struct __attribute__((packed))
    {
        uint8_t  dart_remaining_time;
        uint16_t dart_info;
    }dart_info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        float   bullet_speed_limit;
        uint8_t power_management_gimbal_output  : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    }robot_status_t;

    typedef struct __attribute__((packed))
    {
        uint16_t reserve_1;
        uint16_t reserve_2;
        float    reserve_3;
        uint16_t buffer_energy;
        uint16_t shooter_17mm_barrel_heat;
        uint16_t shooter_42mm_barrel_heat;
    }power_heat_data_t;

    typedef struct __attribute__((packed))
    {
        float x;
        float y;
        float angle;
    }robot_pos_t;

    typedef struct __attribute__((packed))
    {
        uint8_t recovery_buff;
        uint16_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
    }buff_t;

    typedef struct __attribute__((packed))
    {
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
    }hurt_data_t;

    typedef struct __attribute__((packed))
    {
        uint8_t bullet_type;
        uint8_t shooter_number;
        uint8_t launching_frequency;
        float initial_speed;
    }shoot_data_t;

    typedef struct __attribute__((packed))
    {
        uint16_t projectile_allowance_17mm;
        uint16_t projectile_allowance_42mm;
        uint16_t remaining_gold_coin;
        uint16_t projectile_allowance_fortress;
    }projectile_allowance_t;

    typedef struct __attribute__((packed))
    {
        uint32_t rfid_status;
        uint8_t rfid_status_2;
    }rfid_status_t;

    typedef struct __attribute__((packed))
    {
        uint8_t dart_launch_opening_status;
        uint8_t reserve_1;
        uint16_t target_change_time;
        uint16_t latest_launch_cmd_time;
    }dart_client_cmd_t;

    typedef struct __attribute__((packed))
    {
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float reserve_1;
        float reserve_2;
    }ground_robot_position_t;

    typedef struct __attribute__((packed))
    {
        uint16_t mark_progress;
    }radar_mark_data_t;

    typedef struct __attribute__((packed))
    {
        uint32_t sentry_info;
        uint16_t sentry_info_2;
        uint64_t sentry_info_3;
    }sentry_info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t radar_info;
    }radar_info_t;

    /* ================================== */
    /* The struct of 0x0301 Subcontent ID */
    typedef struct __attribute__((packed))
    {
        uint8_t delete_type;
        uint8_t layer;
    }interaction_layer_delete_t;

    typedef struct __attribute__((packed))
    {
        uint8_t figure_name[3];
        uint32_t operate_tpye:3;
        uint32_t figure_tpye:3;
        uint32_t layer:4;
        uint32_t color:4;
        uint32_t details_a:9;
        uint32_t details_b:9;
        uint32_t details_c:10;
        uint32_t width:10;
        uint32_t start_x:11;
        uint32_t start_y:11;
        uint32_t details_d:11;
        uint32_t details_e:11;
    }interaction_figure_t;

    typedef struct __attribute__((packed))
    {
        interaction_figure_t interaction_figure[2];
    }interaction_figure_2_t;

    typedef struct __attribute__((packed))
    {
        interaction_figure_t interaction_figure[5];
    }interaction_figure_5_t;

    typedef struct __attribute__((packed))
    {
        interaction_figure_t interaction_figure[7];
    }interaction_figure_7_t;

    // typedef struct
    // {
    //     graphic_data_struct_t grapic_data_struct;
    //     uint8_t data[30];
    // }ext_client_custom_character_t;

    typedef struct __attribute__((packed))
    {
        uint32_t sentry_cmd;
    } sentry_cmd_t;

    typedef struct __attribute__((packed))
    {
        uint8_t radar_cmd;
        uint8_t password_cmd;
        uint8_t password_1;
        uint8_t password_2;
        uint8_t password_3;
        uint8_t password_4;
        uint8_t password_5;
        uint8_t password_6;
    }radar_cmd_t;
    /* ================================== */

    typedef struct __attribute__((packed))
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t *user_data;
    }robot_interaction_data_t;

    typedef struct __attribute__((packed))
    {
        float target_position_x;
        float target_position_y;
        uint8_t cmd_keyboard;
        uint8_t target_robot_id;
        uint16_t cmd_source;
    }map_command_t;

    typedef struct __attribute__((packed))
    {
        uint16_t opponent_hero_position_x;
        uint16_t opponent_hero_position_y;
        uint16_t opponent_engineer_position_x;
        uint16_t opponent_engineer_position_y;
        uint16_t opponent_infantry_3_position_x;
        uint16_t opponent_infantry_3_position_y;
        uint16_t opponent_infantry_4_position_x;
        uint16_t opponent_infantry_4_position_y;
        uint16_t opponent_aerial_position_x;
        uint16_t opponent_aerial_position_y;
        uint16_t opponent_sentry_position_x;
        uint16_t opponent_sentry_position_y;
        uint16_t ally_hero_position_x;
        uint16_t ally_hero_position_y;
        uint16_t ally_engineer_position_x;
        uint16_t ally_engineer_position_y;
        uint16_t ally_infantry_3_position_x;
        uint16_t ally_infantry_3_position_y;
        uint16_t ally_infantry_4_position_x;
        uint16_t ally_infantry_4_position_y;
        uint16_t ally_aerial_position_x;
        uint16_t ally_aerial_position_y;
        uint16_t ally_sentry_position_x;
        uint16_t ally_sentry_position_y;
    }map_robot_data_t;

    typedef struct __attribute__((packed))
    {
        uint8_t intention;
        uint16_t start_position_x;
        uint16_t start_position_y;
        int8_t delta_x[49];
        int8_t delta_y[49];
        uint16_t sender_id;
    }map_data_t;

    typedef struct __attribute__((packed))
    {
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[30];
    }custom_info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t data[30];
    }custom_robot_data_t;

    typedef struct __attribute__((packed))
    {
        int16_t  mouse_x;
        int16_t  mouse_y;
        int16_t  mouse_z;
        int8_t   left_button_down;
        int8_t   right_button_down;
        uint16_t keyboard_value;
        uint16_t reserved;
    }remote_control_vt02_241225_t;

    typedef struct __attribute__((packed))
    {
        uint8_t data[30];
    }robot_custom_data_t;

    typedef struct __attribute__((packed))
    {
        uint8_t data[300];
    }robot_custom_data_2_t;

    typedef struct __attribute__((packed))
    {
        uint8_t data[30];
    }custom_robot_data_2_t;

/**
 * @brief RoboMaster Referee System Info
 */
typedef struct {
    /* Original Data Frame */
    game_status_t            game_status;
    game_result_t            game_result;
    game_robot_HP_t          game_robot_hp;
    event_data_t             event_data;
    referee_warning_t        referee_warning;
    dart_info_t              dart_info;
    robot_status_t           robot_status;
    power_heat_data_t        power_heat_data;
    robot_pos_t              robot_pos;
    buff_t                   buff;
    hurt_data_t              hurt_data;
    shoot_data_t             shoot_data;
    projectile_allowance_t   projectile_allowance;
    rfid_status_t            rfid_status;
    dart_client_cmd_t        dart_client_cmd;
    ground_robot_position_t  ground_robot_position;
    radar_mark_data_t        radar_mark_data;
    sentry_info_t            sentry_info;
    radar_info_t             radar_info;
    robot_interaction_data_t robot_interaction_data;
    map_command_t            map_command;
    map_robot_data_t         map_robot_data;
    map_data_t               map_data;
    custom_info_t            custom_info;

    /* Visual Info */
    ROBOMASTER_GAME_TYPE_t   RoboMaster_Game_Type;
    ROBOT_FACTION_t          Robot_Faction;
    ROBOT_ARMS               RoboMaster_ARMS;
    GAME_RESULT_INFO_t       Game_Result_Info;

    /* Current CMD ID */
    uint16_t                 rx_cmd_id;

    /* Tx Msg Info */
    uint16_t                 tx_cmd_id;
    uint8_t                  tx_seq;
}RoboMAster_Referee_System_Standard_Link_Info_t;

typedef struct {
    /* Original Data Frame */
    /* Special VTM Link Data */ //
    bool W;
    bool S;
    bool A;
    bool D;
    bool Shift;
    bool Ctrl;
    bool Q;
    bool E;
    bool R;
    bool F;
    bool G;
    bool Z;
    bool X;
    bool C;
    bool V;
    bool B;

    /* VTM Link Data */

    custom_robot_data_t custom_robot_data;

    remote_control_vt02_241225_t remote_control_vt02_241225;

    robot_custom_data_t robot_custom_data;

    custom_robot_data_2_t custom_robot_data_2;
    robot_custom_data_2_t robot_custom_data_2;

    /* Current CMD ID */
    uint16_t                 rx_cmd_id;

}RoboMaster_Referee_System_VTM_Link_Info_t;
/* --------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_ROBOMASTER_REFEREE_INFO_DEFINE_H