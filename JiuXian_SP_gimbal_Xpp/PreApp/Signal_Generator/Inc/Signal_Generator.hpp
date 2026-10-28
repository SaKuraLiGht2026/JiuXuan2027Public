//
// Created by ASUS on 2026/8/22.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_SIGNAL_GENERATOR_HPP
#define JIUXIAN_SP_GIMBAL_XPP_SIGNAL_GENERATOR_HPP

#include <algorithm>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

/* ================================================================================================================ */
/**
 * 脉冲信号发生器
 */
/* ================================================================================================================ */
typedef void (*Pulse_Start_Callback)();

class Pulse_Generator {
public:

    Pulse_Generator(uint64_t tick, Pulse_Start_Callback Callback_Function) : input(false),
                                                                             current_trigger_flag_(false),
                                                                             last_trigger_flag_(false),
                                                                             is_trigger_(false),
                                                                             trigger_tick_(0),
                                                                             trigger_tick_max_(tick),
                                                                             pulse_generator_output_(false),
                                                                             Callback_Function_(Callback_Function) {}

    void Set_Pulse_Generato_Input(bool input) {this->input = input;}

    void Pulse_Rising_Edge();

    void Pulse_Falling_Edge();

    bool Get_Current_flag() {return this->current_trigger_flag_;}

    bool Get_last_flag() {return this->last_trigger_flag_;}

    bool Get_is_trigger() {return this->is_trigger_;}

    bool Get_Pulse_Output() {return this->pulse_generator_output_;}

private:
    bool                 input;

    bool                 current_trigger_flag_;
    bool                 last_trigger_flag_;

    bool                 is_trigger_;

    uint64_t             trigger_tick_;
    uint64_t             trigger_tick_max_;

    bool                 pulse_generator_output_;

    Pulse_Start_Callback Callback_Function_;

};
/* ================================================================================================================ */
/**
 * @brief 三角波发生器
 */
/* ================================================================================================================ */
class Triangle_Generator {
public:
    Triangle_Generator(bool start_rising, float up_gain, float down_gain,
                       float top, float bottom, float offset)
        : is_rising_(start_rising),
          up_gain_(up_gain),
          down_gain_(down_gain),
          top_(top),
          bottom_(bottom),
          output_(offset)
    {
        assert(up_gain_ > 0.0f && "up_gain must be positive");
        assert(down_gain_ > 0.0f && "down_gain must be positive");

        assert(top_ > bottom_ && "top must be greater than bottom");

        if (top_ < bottom_) std::swap(top_, bottom_);

        if (output_ > top_) output_ = top_;
        if (output_ < bottom_) output_ = bottom_;
    }

    void update() {
        if (is_rising_) {
            output_ += up_gain_;
            if (output_ >= top_) {
                output_ = top_;
                is_rising_ = false;
            }
        } else {
            output_ -= down_gain_;
            if (output_ <= bottom_) {
                output_ = bottom_;
                is_rising_ = true;
            }
        }
    }

    void reset(float offset = 0.0f) {
        output_ = std::clamp(offset, bottom_, top_);
    }

    [[nodiscard]] float Get_Triangle_Generator_Output() const { return output_; }

private:
    bool  is_rising_;
    float up_gain_;
    float down_gain_;
    float top_;
    float bottom_;
    float output_;
};
/* ================================================================================================================ */

#endif //JIUXIAN_SP_GIMBAL_XPP_SIGNAL_GENERATOR_HPP