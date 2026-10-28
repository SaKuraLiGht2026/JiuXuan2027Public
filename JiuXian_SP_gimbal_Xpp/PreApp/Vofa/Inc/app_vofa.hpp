//
// Created by ASUS on 2026/7/22.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_APP_VOFA_HPP
#define JIUXIAN_SP_CHASSIS_XPP_APP_VOFA_HPP

#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include <usart.h>
#include <dma.h>

#define Vofa_Rx_Buffer_Size         256

/**
 * @brief 控件类型
 */
#define SIMPLE_FLOAT_CONTROL      0x01U
#define BOOL_CONTROL              0x02U
#define REMOTE_CONTROL            0x03U

/**
 * @brief 需要发送的浮点型数据数量
 */
#define VOFA_SHOW_NUM_1               1
#define VOFA_SHOW_NUM_2               2
#define VOFA_SHOW_NUM_3               3
#define VOFA_SHOW_NUM_4               4
#define VOFA_SHOW_NUM_5               5
#define VOFA_SHOW_NUM_6               6
#define VOFA_SHOW_NUM_7               7
#define VOFA_SHOW_NUM_8               8
#define VOFA_SHOW_NUM_9               9
#define VOFA_SHOW_NUM_10             10
#define VOFA_SHOW_NUM_11             11
#define VOFA_SHOW_NUM_12             12
#define VOFA_SHOW_NUM_13             13
#define VOFA_SHOW_NUM_14             14
#define VOFA_SHOW_NUM_15             15
#define VOFA_SHOW_NUM_16             16
#define VOFA_SHOW_NUM_17             17
#define VOFA_SHOW_NUM_18             18
#define VOFA_SHOW_NUM_19             19
#define VOFA_SHOW_NUM_20             20

/**
 * @brief Vofa数据处理回调函数数据类型
 */
typedef void (*Vofa_Data_Process_Callback)();

typedef struct {
    uint8_t  name[256];
    uint8_t  value[256];

    uint16_t name_length;
    uint16_t value_length;

    bool     isMemcpyOK;
}VOFA_Rx_Info_t;

namespace VOFA {
    class VOFA_FireWater_Manager {
        public:
        VOFA_FireWater_Manager(UART_HandleTypeDef *huart);

        UART_HandleTypeDef *Gain_Vofa_FireWater_Manager_UART_Handle();

        bool Gain_Vofa_FireWater_Manager_IsInit();

        void Set_Tx_Buffer_Addr(uint32_t addr);

        uint32_t Gain_Tx_Buffer_Addr();

        void Write_data(float *Data, uint16_t Length);

        private:
        UART_HandleTypeDef *UART_Handler;

        uint8_t             show_method_;

        uint8_t            *vofa_tx_buffer_;

        bool                isVofaManagerInit;

        /* 工具函数 */
        static char *append_float(char *p, float value);
        static char *itoa_rev(char *p, int32_t val, uint8_t min_width);
    };

    class VOFA_Justfloat_Manager {
        public:
        VOFA_Justfloat_Manager(UART_HandleTypeDef *huart);

        bool Gain_Vofa_Justfloat_Manager_IsInit();

        void Set_Tx_Buffer_Addr(uint32_t addr);

        uint32_t Gain_Tx_Buffer_Addr();

        void Write_data(float *Data, uint16_t Length);

        bool Gain_Tx_State();

        private:
        UART_HandleTypeDef *UART_Handler;

        uint8_t            *vofa_tx_buffer_;

        uint8_t             Vofa_Justfloat_frame_tail[4];

        bool                isVofaManagerInit;
    };

    class VOFA_UI_Manager {
        public:
        VOFA_UI_Manager(const char* name, uint8_t ui_type, Vofa_Data_Process_Callback callback_function, VOFA_Rx_Info_t *link_buffer);

        bool Set_Param();

        float Gain_value();

        float Gain_Remote_value_x();

        float Gain_Remote_value_y();

        private:
        char                        name_[64];
        uint16_t                    name_length_;

        uint8_t                     temp_value_[64];

        // simple float
        float                       UI_value_;
        // 2DOF remote float
        float                       UI_value_x_;
        float                       UI_value_y_;
        // bool
        bool                        UI_state_;

        uint8_t                     UI_type_;

        VOFA_Rx_Info_t             *info_;

        Vofa_Data_Process_Callback  UI_Trigger_Callback_Function;
    };
}

#endif //JIUXIAN_SP_CHASSIS_XPP_APP_VOFA_HPP