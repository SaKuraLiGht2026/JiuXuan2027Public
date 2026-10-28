//
// Created by ASUS on 2026/8/22.
//
#include "../Inc/Signal_Generator.hpp"

void Pulse_Generator::Pulse_Rising_Edge() {
    if (!this->pulse_generator_output_) {
        this->current_trigger_flag_ = input;

        if (this->current_trigger_flag_ == true && this->last_trigger_flag_ == false) {
            this->is_trigger_ = true;
        }else {
            this->is_trigger_ = false;
        }

        if (this->is_trigger_ == true) {
            this->pulse_generator_output_ = true;
            if (this->Callback_Function_ != nullptr) {
                this->Callback_Function_();
            }
        }

        this->last_trigger_flag_ = this->current_trigger_flag_;
    }

    if (this->pulse_generator_output_) {
        this->trigger_tick_++;
        if (this->trigger_tick_ >= this->trigger_tick_max_) {
            this->trigger_tick_ = 0;
            this->pulse_generator_output_ = false;
        }
    }
}

void Pulse_Generator::Pulse_Falling_Edge() {
    if (!this->pulse_generator_output_) {
        this->current_trigger_flag_ = input;

        if (this->current_trigger_flag_ == false && this->last_trigger_flag_ == true) {
            this->is_trigger_ = true;
        }else {
            this->is_trigger_ = false;
        }

        if (this->is_trigger_ == true) {
            this->pulse_generator_output_ = true;
            if (this->Callback_Function_ != nullptr) {
                this->Callback_Function_();
            }
        }

        this->last_trigger_flag_ = this->current_trigger_flag_;
    }

    if (this->pulse_generator_output_) {
        this->trigger_tick_++;
        if (this->trigger_tick_ >= this->trigger_tick_max_) {
            this->trigger_tick_ = 0;
            this->pulse_generator_output_ = false;
        }
    }
}
