//
// Created by ASUS on 2026/7/30.
//
#include "../Inc/drv_robomaster_vtm_link.h"

RoboMaster_Referee_System_VTM_Link_Info_t RM_Referee_VTM_Link_Info;

void VTM_Link_Init(void) {
    RM_Referee_VTM_Link_Info.rx_cmd_id = 0x0000;

    RM_Referee_VTM_Link_Info.W     = false;     // 前
    RM_Referee_VTM_Link_Info.S     = false;     // 后
    RM_Referee_VTM_Link_Info.A     = false;     // 左
    RM_Referee_VTM_Link_Info.D     = false;     // 右
    RM_Referee_VTM_Link_Info.Shift = false;     // 加速
    RM_Referee_VTM_Link_Info.Ctrl  = false;     // 是否附加遥控器数据
    RM_Referee_VTM_Link_Info.Q     = false;     // 小陀螺速度方向
    RM_Referee_VTM_Link_Info.E     = false;     // 变速小陀螺开启或关闭
    RM_Referee_VTM_Link_Info.R     = false;     // 单发连发切换
    RM_Referee_VTM_Link_Info.F     = false;     // 开摩擦轮
    RM_Referee_VTM_Link_Info.G     = false;
    RM_Referee_VTM_Link_Info.Z     = false;
    RM_Referee_VTM_Link_Info.X     = false;
    RM_Referee_VTM_Link_Info.C     = false;     // 开关超电
    RM_Referee_VTM_Link_Info.V     = false;
    RM_Referee_VTM_Link_Info.B     = false;
}

void VTM_Link_Clear(void) {
    RM_Referee_VTM_Link_Info.rx_cmd_id = 0x0000;

    RM_Referee_VTM_Link_Info.W     = false;     // 前
    RM_Referee_VTM_Link_Info.S     = false;     // 后
    RM_Referee_VTM_Link_Info.A     = false;     // 左
    RM_Referee_VTM_Link_Info.D     = false;     // 右
    RM_Referee_VTM_Link_Info.Shift = false;     // 加速
    RM_Referee_VTM_Link_Info.Ctrl  = false;     // 是否附加遥控器数据
    RM_Referee_VTM_Link_Info.Q     = false;     // 小陀螺速度方向
    RM_Referee_VTM_Link_Info.E     = false;     // 变速小陀螺开启或关闭
    RM_Referee_VTM_Link_Info.R     = false;     // 单发连发切换
    RM_Referee_VTM_Link_Info.F     = false;     // 开摩擦轮
    RM_Referee_VTM_Link_Info.G     = false;
    RM_Referee_VTM_Link_Info.Z     = false;
    RM_Referee_VTM_Link_Info.X     = false;
    RM_Referee_VTM_Link_Info.C     = false;     // 开关超电
    RM_Referee_VTM_Link_Info.V     = false;
    RM_Referee_VTM_Link_Info.B     = false;

    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.mouse_x = 0;
    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.mouse_y = 0;
    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.mouse_z = 0;
    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.left_button_down  = 0;
    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.right_button_down = 0;
    RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value    = 0;
}

void RoboMaster_VTM_Link_Rx_Data_Process(Referee_uint8 *data, Referee_uint16 len) {
    for (Referee_int i = 0; i < len; i++) {
        /**
         * @brief 帧头不对(这是或命题！！！)
         *
         */
#if VTM_VT02_Ver_20241225

        if (data[i] != REFEREE_VTM_LINK_HEADER_241225_SOF) {
            continue;;
        }

#else

        if (data[i] !=REFEREE_VTM_LINK_HEADER_SOF) {
            continue;
        }

#endif
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
        RM_Referee_VTM_Link_Info.rx_cmd_id = CMD_ID_CODE;
        switch (CMD_ID_CODE) {
            case REMOTE_CONTROL_DATA_241225 : {
                memcpy(&RM_Referee_VTM_Link_Info.remote_control_vt02_241225, data + i + 7, DATA_LEN_0x0304);
                break;
            }
            default: {
                return;
            }
        }
        VTM_Link_Rx_Data_Parse();
    }
}

void VTM_Link_Rx_Data_Parse(void) {
    switch(RM_Referee_VTM_Link_Info.rx_cmd_id) {
            case REMOTE_CONTROL_DATA_241225 : {
                RM_Referee_VTM_Link_Info.W     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x01  ) ? true : false;
                RM_Referee_VTM_Link_Info.S     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x02  ) ? true : false;
                RM_Referee_VTM_Link_Info.A     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x04  ) ? true : false;
                RM_Referee_VTM_Link_Info.D     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x08  ) ? true : false;
                RM_Referee_VTM_Link_Info.Shift = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x10  ) ? true : false;
                RM_Referee_VTM_Link_Info.Ctrl  = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x20  ) ? true : false;
                RM_Referee_VTM_Link_Info.Q     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x40  ) ? true : false;
                RM_Referee_VTM_Link_Info.E     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x80  ) ? true : false;
                RM_Referee_VTM_Link_Info.R     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x100 ) ? true : false;
                RM_Referee_VTM_Link_Info.F     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x200 ) ? true : false;
                RM_Referee_VTM_Link_Info.G     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x400 ) ? true : false;
                RM_Referee_VTM_Link_Info.Z     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x800 ) ? true : false;
                RM_Referee_VTM_Link_Info.X     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x1000) ? true : false;
                RM_Referee_VTM_Link_Info.C     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x2000) ? true : false;
                RM_Referee_VTM_Link_Info.V     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x4000) ? true : false;
                RM_Referee_VTM_Link_Info.B     = (RM_Referee_VTM_Link_Info.remote_control_vt02_241225.keyboard_value & 0x8000) ? true : false;
                break;
            }
            default : {
                break;
            }
    }
}