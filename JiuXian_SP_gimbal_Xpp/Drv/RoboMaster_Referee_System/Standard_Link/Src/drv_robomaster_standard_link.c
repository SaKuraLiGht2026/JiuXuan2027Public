//
// Created by ASUS on 2026/7/30.
//
#include "../Inc/drv_robomaster_standard_link.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"

RoboMAster_Referee_System_Standard_Link_Info_t RM_Referee_Standard_Link_Info;

void Standard_Link_Init(void) {
    RM_Referee_Standard_Link_Info.RoboMaster_Game_Type = TYPE_DEFAULT;
    RM_Referee_Standard_Link_Info.Robot_Faction        = FACTION_DEFAULT;
    RM_Referee_Standard_Link_Info.RoboMaster_ARMS      = ROBOT_DEFAULT;
    RM_Referee_Standard_Link_Info.Game_Result_Info     = GAME_DEFAULT;

    RM_Referee_Standard_Link_Info.tx_cmd_id            = 0x0000;
    RM_Referee_Standard_Link_Info.rx_cmd_id            = 0x0000;

    RM_Referee_Standard_Link_Info.tx_seq               = 0;
}

void RoboMaster_Interaction_UI_Design_One_Graphic(interaction_figure_t *figure) {
    uint8_t UI_tx_buffer[30];
    memset(UI_tx_buffer, 0, sizeof(UI_tx_buffer));

    /* ---------- 帧头 ---------- */
    UI_tx_buffer[0] = 0xA5;

    uint16_t data_len = 23;   // 2 + 6 + 15
    UI_tx_buffer[1] = data_len & 0xFF;
    UI_tx_buffer[2] = (data_len >> 8) & 0xFF;

    UI_tx_buffer[3] = RM_Referee_Standard_Link_Info.tx_seq++;

    /* CRC8 */
    UI_tx_buffer[4] = Append_CRC8_Check_Sum2(UI_tx_buffer, 5);

    /* ---------- 命令码 ---------- */
    UI_tx_buffer[5] = ROBOT_INTERACTION_DATA & 0xFF;        // 0x0301
    UI_tx_buffer[6] = (ROBOT_INTERACTION_DATA >> 8) & 0xFF;

    /* ---------- 交互头 ---------- */
    uint16_t sender   = RM_Referee_Standard_Link_Info.robot_status.robot_id;
    uint16_t receiver = sender + 0x100;

    UI_tx_buffer[7]  = CREATE_ONE_CLIENT_UI_LAYER & 0xFF;
    UI_tx_buffer[8]  = (CREATE_ONE_CLIENT_UI_LAYER >> 8) & 0xFF;
    UI_tx_buffer[9]  = sender & 0xFF;
    UI_tx_buffer[10] = (sender >> 8) & 0xFF;
    UI_tx_buffer[11] = receiver & 0xFF;
    UI_tx_buffer[12] = (receiver >> 8) & 0xFF;

    /* ---------- 图形数据 ---------- */
    memcpy(UI_tx_buffer + 13, figure, 15);

    /* ---------- CRC16 ---------- */
    uint16_t crc16 = Append_CRC16_Check_Sum2(UI_tx_buffer, 30);
    UI_tx_buffer[28] = crc16 & 0xFF;
    UI_tx_buffer[29] = (crc16 >> 8) & 0xFF;

    /* ---------- 发送 ---------- */
    HAL_UART_Transmit(&huart1, UI_tx_buffer, 30, 100);
}

void RoboMaster_Standard_Link_Rx_Data_Process(Referee_uint8 *data, Referee_uint16 len) {
    for (Referee_int i = 0; i < len; i++) {
        /**
         * @brief 帧头不对
         *
         */
        if (data[i] != REFEREE_STANDARD_LINK_HEADER_SOF) {
            continue;
        }
        /**
         * @brief 帧头对了但是数据长度不够校验CRC8了
         *
         */
        if (i + 4 > len) {
            break;
        }
        /**
         * @brief 帧头数据CRC8校验错误（帧头数据 = 帧头（1byte） + 数据帧长度（2byte） + seq（1byte） + crc8（1byte））
         *
         */
        if (Verify_CRC8_Check_Sum(data + i, 5) == __FALSE) {
            continue;
        }

        // 计算数据长度（小端序）
        Referee_uint16 data_len = (data[i + 2] << 8) | data[i + 1];
        Referee_uint16 total_frame_len = data_len + 7 + 2;  // 总长度

        /**
         * @brief 数据长度不够校验CRC16了
         *
         */
        if (i + total_frame_len > len) {
            break;
        }

        /**
         * @brief 数据CRC16校验错误
         *
         */
        if (Verify_CRC16_Check_Sum(data + i, total_frame_len) == __FALSE) {
            i += total_frame_len - 1;  // 跳过整个数据包
            continue;
        }

        // 记录数据……
        Referee_uint16 CMD_ID_CODE = (data[i + 6] << 8) | data[i + 5];
        RM_Referee_Standard_Link_Info.rx_cmd_id = CMD_ID_CODE;
        switch (CMD_ID_CODE) {
            case GAME_STATUS : {
                memcpy(&RM_Referee_Standard_Link_Info.game_status, data + i + 7, DATA_LEN_0x0001);
                break;
            }
            case GAME_RESULT : {
                memcpy(&RM_Referee_Standard_Link_Info.game_result, data + i + 7, DATA_LEN_0x0002);
                break;
            }
            case GAME_ROBOT_HP: {
                memcpy(&RM_Referee_Standard_Link_Info.game_robot_hp, data + i + 7, DATA_LEN_0x0003);
                break;
            }
            case EVENT_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.event_data, data + i + 7, DATA_LEN_0x0101);
                break;
            }
            case REFEREE_WARNING : {
                memcpy(&RM_Referee_Standard_Link_Info.referee_warning, data + i + 7, DATA_LEN_0x0104);
                break;
            }
            case DART_INFO : {
                memcpy(&RM_Referee_Standard_Link_Info.dart_info, data + i + 7, DATA_LEN_0x0105);
                break;
            }
            case ROBOT_STATUS : {
                memcpy(&RM_Referee_Standard_Link_Info.robot_status, data + i + 7, DATA_LEN_0x0201);
                break;
            }
            case POWER_HEAT_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.power_heat_data, data + i + 7, DATA_LEN_0x0202);
                break;
            }
            case ROBOT_POS : {
                memcpy(&RM_Referee_Standard_Link_Info.robot_status, data + i + 7, DATA_LEN_0x0203);
                break;
            }
            case BUFF : {
                memcpy(&RM_Referee_Standard_Link_Info.buff, data + i + 7, DATA_LEN_0x0204);
                break;
            }
            case HURT_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.hurt_data, data + i + 7, DATA_LEN_0x0206);
                break;
            }
            case SHOOT_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.shoot_data, data + i + 7, DATA_LEN_0x0207);
                break;
            }
            case PROJECTILE_ALLOWANCE : {
                memcpy(&RM_Referee_Standard_Link_Info.projectile_allowance, data + i + 7, DATA_LEN_0x0208);
                break;
            }
            case RFID_STATUS : {
                memcpy(&RM_Referee_Standard_Link_Info.rfid_status, data + i + 7, DATA_LEN_0x0209);
                break;
            }
            case DART_CLIENT_CMD : {
                memcpy(&RM_Referee_Standard_Link_Info.dart_client_cmd, data + i + 7, DATA_LEN_0x020A);
                break;
            }
            case GROUND_ROBOT_POSITION : {
                memcpy(&RM_Referee_Standard_Link_Info.ground_robot_position, data + i + 7, DATA_LEN_0x020B);
                break;
            }
            case RADAR_MARK_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.radar_mark_data, data + i + 7, DATA_LEN_0x020C);
                break;
            }
            case SENTRY_INFO : {
                memcpy(&RM_Referee_Standard_Link_Info.sentry_info, data + i + 7, DATA_LEN_0x020D);
                break;
            }
            case RADAR_INFO : {
                memcpy(&RM_Referee_Standard_Link_Info.radar_info, data + i + 7, DATA_LEN_0x020E);
                break;
            }
            case ROBOT_INTERACTION_DATA : {
                memcpy(&RM_Referee_Standard_Link_Info.robot_interaction_data, data + i + 7, DATA_LEN_0x0301_MAX);
                break;
            }
            case MAP_COMMAND : {
                memcpy(&RM_Referee_Standard_Link_Info.map_command, data + i + 7, DATA_LEN_0x0303);
                break;
            }
            default: {
                return;
            }
        }
        Standard_Link_Rx_Data_Parse();
    }
}

void Standard_Link_Rx_Data_Parse(void) {
    switch(RM_Referee_Standard_Link_Info.rx_cmd_id) {
            case GAME_STATUS : {

                break;
            }
            case GAME_RESULT : {

                break;
            }
            case GAME_ROBOT_HP: {

                break;
            }
            case EVENT_DATA : {

                break;
            }
            case REFEREE_WARNING : {

                break;
            }
            case DART_INFO : {

                break;
            }
            case ROBOT_STATUS : {
                if (RM_Referee_Standard_Link_Info.robot_status.robot_id >= 1 && RM_Referee_Standard_Link_Info.robot_status.robot_id <= 9) {
                    RM_Referee_Standard_Link_Info.Robot_Faction = RED;
                    if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_HERO)             RM_Referee_Standard_Link_Info.RoboMaster_ARMS = HERO_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_ENGINEER)    RM_Referee_Standard_Link_Info.RoboMaster_ARMS = ENGINEER_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_INFANTRY_3)  RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_INFANTRY_4)  RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_INFANTRY_5)  RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_AERIAL)      RM_Referee_Standard_Link_Info.RoboMaster_ARMS = AERIAL_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_SENTRY)      RM_Referee_Standard_Link_Info.RoboMaster_ARMS = SENTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_DART)        RM_Referee_Standard_Link_Info.RoboMaster_ARMS = DART_SYSTEM;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == RED_RADAR)       RM_Referee_Standard_Link_Info.RoboMaster_ARMS = RADAR_SYSTEM;
                    else                                                                             RM_Referee_Standard_Link_Info.RoboMaster_ARMS = ROBOT_DEFAULT;
                }else if (RM_Referee_Standard_Link_Info.robot_status.robot_id >= 101 && RM_Referee_Standard_Link_Info.robot_status.robot_id <= 109) {
                    RM_Referee_Standard_Link_Info.Robot_Faction = BLUE;
                    if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_HERO)            RM_Referee_Standard_Link_Info.RoboMaster_ARMS = HERO_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_ENGINEER)   RM_Referee_Standard_Link_Info.RoboMaster_ARMS = ENGINEER_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_INFANTRY_3) RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_INFANTRY_4) RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_INFANTRY_5) RM_Referee_Standard_Link_Info.RoboMaster_ARMS = INFANTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_AERIAL)     RM_Referee_Standard_Link_Info.RoboMaster_ARMS = AERIAL_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_SENTRY)     RM_Referee_Standard_Link_Info.RoboMaster_ARMS = SENTRY_ROBOT;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_DART)       RM_Referee_Standard_Link_Info.RoboMaster_ARMS = DART_SYSTEM;
                    else if (RM_Referee_Standard_Link_Info.robot_status.robot_id == BLUE_RADAR)      RM_Referee_Standard_Link_Info.RoboMaster_ARMS = RADAR_SYSTEM;
                    else                                                                             RM_Referee_Standard_Link_Info.RoboMaster_ARMS = ROBOT_DEFAULT;
                }
                break;
            }
            case POWER_HEAT_DATA : {

                break;
            }
            case ROBOT_POS : {

                break;
            }
            case BUFF : {

                break;
            }
            case HURT_DATA : {

                break;
            }
            case SHOOT_DATA : {

                break;
            }
            case PROJECTILE_ALLOWANCE : {

                break;
            }
            case RFID_STATUS : {

                break;
            }
            case DART_CLIENT_CMD : {

                break;
            }
            case GROUND_ROBOT_POSITION : {

                break;
            }
            case RADAR_MARK_DATA : {

                break;
            }
            case SENTRY_INFO : {

                break;
            }
            case RADAR_INFO : {

                break;
            }
            case ROBOT_INTERACTION_DATA : {
                /* To be continue... */
                break;
            }
            case MAP_COMMAND : {

                break;
            }
            default : {
                break;
            }
    }
}
