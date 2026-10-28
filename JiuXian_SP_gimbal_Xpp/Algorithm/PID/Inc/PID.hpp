//
// Created by ASUS on 2026/7/16.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_PID_HPP
#define JIUXIAN_SP_GIMBAL_XPP_PID_HPP

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief PID控制器(1.0版本)
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float kf, float I_Limit, float u_Limit, bool isAnglePID);

    void set_params(float kp, float ki, float kd, float kf){kp_ = kp; ki_ = ki; kd_ = kd; kf_ = kf;}

    bool update(float reference, float measure);

    /**
     * @brief 自适应调整kp值
     * @param max_kp
     * @param min_kp
     * @param adaptive_rate
     */
    void AdaptiveKp(float max_kp, float min_kp, float adaptive_rate) {
        if ((error_ - pre_error_) > 0) {
            kp_ *= (1 + adaptive_rate);
            if (kp_ > max_kp) kp_ = max_kp;
        }else {
            kp_ *= (1 - adaptive_rate);
            if (kp_ < min_kp) kp_ = min_kp;
        }
    }



    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    float getKf() const { return kf_; }

    float getError() const { return error_; }
    float getPreError() const { return pre_error_; }

    float getOutput() const;

    void Clear();

private:
    float kp_;
    float ki_;
    float kd_;
    float kf_;

    float error_;
    float pre_error_;
    float pre_pre_error_;

    float I_Limit_;
    float u_Limit_;

    float pO_;
    float iO_;
    float dO_;
    float dO_pre_;
    float dO_lpf_;
    float fO_;

    float output_;

    bool  isAnglePID_;
};

#endif //JIUXIAN_SP_GIMBAL_XPP_PID_HPP