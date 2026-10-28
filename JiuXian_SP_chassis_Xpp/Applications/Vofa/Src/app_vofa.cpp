//
// Created by ASUS on 2026/7/22.
//
#include "../Inc/app_vofa.hpp"
#include "../Inc/vofa_ascii.h"
#include <cassert>

namespace VOFA {
    /**
     * @attention : 建议优先使用JustFloat协议，FireWater协议其实开销很大，对于那些频率要求极高的场景是很吃力的。
     * @attention : 这里只写JustFloat协议发送和FireWater协议发送，因为RawData没有协议，单纯用来当串口助手使用。
     */
    /* FireWater TX frame -------------------------------------------------------------------------- */
    char *VOFA_FireWater_Manager::itoa_rev(char *p, int32_t val, uint8_t min_width) {
        char *start = p;

        if (val == 0) {
            *p++ = '0';
        } else {
            while (val > 0) {
                *p++ = '0' + (val % 10);
                val /= 10;
            }
        }

        /* 补零（小数部分用） */
        while ((uint8_t)(p - start) < min_width) {
            *p++ = '0';
        }

        /* 反转 */
        for (char *a = start, *b = p - 1; a < b; ++a, --b) {
            char t = *a;
            *a = *b;
            *b = t;
        }

        return p;
    }

    char *VOFA_FireWater_Manager::append_float(char *p, float value) {
        /* 符号 */
        if (value < 0.0f) {
            *p++ = '-';
            value = -value;
        }

        /* 四舍五入到 6 位小数 */
        int32_t scaled    = (int32_t)(value * 1000000.0f + 0.5f);
        int32_t int_part  = scaled / 1000000;
        int32_t frac_part = scaled % 1000000;

        /* 整数 */
        p = itoa_rev(p, int_part, 1);

        /* 小数点 */
        *p++ = '.';

        /* 小数（固定 6 位，自动补零） */
        p = itoa_rev(p, frac_part, 6);

        return p;
    }

    VOFA_FireWater_Manager::VOFA_FireWater_Manager(UART_HandleTypeDef *huart) : UART_Handler(huart),
                                                                                isVofaManagerInit(true)
    {

    }

    UART_HandleTypeDef *VOFA_FireWater_Manager::Gain_Vofa_FireWater_Manager_UART_Handle() {
        return this->UART_Handler;
    }

    bool VOFA_FireWater_Manager::Gain_Vofa_FireWater_Manager_IsInit() {
        return this->isVofaManagerInit;
    }

    void VOFA_FireWater_Manager::Set_Tx_Buffer_Addr(uint32_t addr) {
        this->vofa_tx_buffer_ = (uint8_t*)addr;
    }

    uint32_t VOFA_FireWater_Manager::Gain_Tx_Buffer_Addr() {
        return (uint32_t)this->vofa_tx_buffer_;
    }

    void VOFA_FireWater_Manager::Write_data(float *Data, uint16_t Length) {
        /* Empty data cancel send. */
        if (Data == nullptr || Length == 0) return;

        char *p = reinterpret_cast<char*>(this->vofa_tx_buffer_);

        for (uint16_t i = 0; i < Length; i++) {
            p = append_float(p, Data[i]);
            if (i < Length - 1) *p++ = ',';
        }

        *p++ = '\r';
        *p++ = '\n';

        uint16_t byte_len = static_cast<uint16_t>(p - reinterpret_cast<char*>(this->vofa_tx_buffer_));

        HAL_UART_Transmit_DMA(this->UART_Handler, this->vofa_tx_buffer_, byte_len);
    }

    /* JustFloat TX frame -------------------------------------------------------------------------- */
    VOFA_Justfloat_Manager::VOFA_Justfloat_Manager(UART_HandleTypeDef *huart) : UART_Handler(huart)
    {
        this->Vofa_Justfloat_frame_tail[0] = 0x00;
        this->Vofa_Justfloat_frame_tail[1] = 0x00;
        this->Vofa_Justfloat_frame_tail[2] = 0x80;
        this->Vofa_Justfloat_frame_tail[3] = 0x7f;

        this->isVofaManagerInit            = true;
    }

    bool VOFA_Justfloat_Manager::Gain_Vofa_Justfloat_Manager_IsInit() {
        return this->isVofaManagerInit;
    }

    void VOFA_Justfloat_Manager::Set_Tx_Buffer_Addr(uint32_t addr) {
        this->vofa_tx_buffer_ = (uint8_t*)addr;
    }

    uint32_t VOFA_Justfloat_Manager::Gain_Tx_Buffer_Addr() {
        return (uint32_t)this->vofa_tx_buffer_;
    }

    void VOFA_Justfloat_Manager::Write_data(float *Data, uint16_t Length) {
        /* Empty data cancel send. */
        if (Data == nullptr || Length == 0) return;

        uint16_t byte_len = Length * sizeof(float);

        memcpy(this->vofa_tx_buffer_           , Data                            , byte_len);
        memcpy(this->vofa_tx_buffer_ + byte_len, this->Vofa_Justfloat_frame_tail ,        4);

        if (this->vofa_tx_buffer_ != nullptr) SCB_CleanDCache_by_Addr((uint32_t*)this->vofa_tx_buffer_, byte_len + 4);

        HAL_UART_Transmit_DMA(this->UART_Handler, this->vofa_tx_buffer_, byte_len + 4);
    }

    /* VOFA UI Design --------------------------------------------------------------------- */
    VOFA_UI_Manager::VOFA_UI_Manager(const char *name, uint8_t ui_type, Vofa_Data_Process_Callback callback_function, VOFA_Rx_Info_t *link_buffer) : UI_value_(0.0f),
                                                                                                                                                     UI_state_(false),
                                                                                                                                                     UI_type_(ui_type),
                                                                                                                                                     info_(link_buffer),
                                                                                                                                                     UI_Trigger_Callback_Function(callback_function)
    {
        assert(name        != nullptr);
        assert(link_buffer != nullptr);

        this->name_length_ = strlen(name);
        memcpy(this->name_, name, this->name_length_);
    }

    bool VOFA_UI_Manager::Set_Param() {
        // 字数不对，退出！
        if (this->name_length_ != this->info_->name_length) return false;

        // 字数对了但是名字对不上，退出！
        for (uint16_t i = 0; i < this->name_length_; i++) {
            if (this->name_[i] != this->info_->name[i]) return false;
        }

        // 比对与设置参数
        if (this->UI_type_ == BOOL_CONTROL) {
            if (this->info_->value_length != 1) {
                return false;
            }else if (this->info_->value[0] == 0x30){
                this->UI_state_ = false;
            }else if (this->info_->value[0] == 0x31) {
                this->UI_state_ = true;
            }else {
                return false;
            }

        }else if (this->UI_type_ == SIMPLE_FLOAT_CONTROL) {
            const uint16_t val_len = this->info_->value_length;
            const uint8_t* val_buf = this->info_->value;

            bool isNegative = false;
            bool has_dot = false;
            float val = 0.0f;
            float scale = 1.0f;  // 整数位权重：1,10,100... 遇到小数点后切为0.1,0.01...

            for (uint16_t i = 0; i < val_len; i++) {
                const uint8_t c = val_buf[i];

                if (c == '-') {
                    // 和原逻辑完全一致：负号只能在开头
                    if (i == 0) isNegative = true;
                    else return false;
                }
                else if (c == '.') {
                    // 和原逻辑完全一致：不能有多个小数点
                    if (has_dot) return false;
                    has_dot = true;
                    scale = 0.1f;
                }
                else if (c >= '0' && c <= '9') {
                    const uint8_t digit = c - '0';
                    if (!has_dot) {
                        val = val * 10.0f + digit;
                    } else {
                        val += digit * scale;
                        scale *= 0.1f;
                    }
                }
                else if (c == '\r' || c == '\n') {
                    break;  // 和原逻辑完全一致：忽略帧尾
                }
                else {
                    return false;  // 和原逻辑完全一致：非法字符
                }
            }

            this->UI_value_ = isNegative ? -val : val;

        }else if (this->UI_type_ == REMOTE_CONTROL) {
            float val = 0.0f;
            bool isNegative = false;
            bool has_dot = false;
            float scale = 1.0f;
            uint8_t state = 0; // 0:解析X, 1:解析Y

            // 预取长度，减少结构体访问次数
            uint16_t len = this->info_->value_length;
            uint8_t *data = this->info_->value;

            for (uint16_t i = 0; i < len; ++i) {
                uint8_t c = data[i];

                if (c == '-') {
                    if (scale != 1.0f) return false; // 负号不在开头
                    isNegative = true;
                } else if (c == '.') {
                    if (has_dot) return false;
                    has_dot = true;
                    scale = 0.1f; // 遇到小数点，切换为除法模式
                } else if (c >= '0' && c <= '9') {
                    if (!has_dot) {
                        // 整数部分：乘10累加
                        val = val * 10.0f + (c - '0');
                    } else {
                        // 小数部分：乘0.1累加
                        val += (c - '0') * scale;
                        scale *= 0.1f;
                    }
                } else if (c == ',') {
                    // X轴结算
                    this->UI_value_x_ = isNegative ? -val : val;
                    // 重置状态，准备解析Y轴
                    val = 0.0f;
                    isNegative = false;
                    has_dot = false;
                    scale = 1.0f;
                    state = 1; // 标记为已进入Y轴解析
                } else if (c == '\r' || c == '\n') {
                    break;
                } else {
                    return false; // 非法字符
                }
            }

            // 循环结束，结算Y轴（必须已经解析过逗号）
            if (state == 1) {
                this->UI_value_y_ = isNegative ? -val : val;
                return true;
            }

            return false; // 格式错误，没找到逗号
        }else {
            return false;
        }

        return true;
    }

    float VOFA_UI_Manager::Gain_value() {
       if (this->UI_type_ == BOOL_CONTROL) {
           return this->UI_state_;
       }else if (this->UI_type_ == SIMPLE_FLOAT_CONTROL) {
           return this->UI_value_;
       }else {
           return 0.0f;
       }
    }

    float VOFA_UI_Manager::Gain_Remote_value_x() {
        if (this->UI_type_ == REMOTE_CONTROL) {
            return this->UI_value_x_;
        }else {
            return 0.0f;
        }
    }

    float VOFA_UI_Manager::Gain_Remote_value_y() {
        if (this->UI_type_ == REMOTE_CONTROL) {
            return this->UI_value_y_;
        }else {
            return 0.0f;
        }
    }
}
