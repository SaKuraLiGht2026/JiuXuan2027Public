//
// Created by ASUS on 2026/4/5.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_ALG_PID_HPP
#define JIUXIAN_SP_CHASSIS_XPP_ALG_PID_HPP

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief PID控制器(1.0版本)
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float kf, float I_Limit, float u_Limit, bool isAnglePID);

    bool update(float reference, float measure);

    [[nodiscard]] float getError() const {return this->error_;}

    [[nodiscard]] float getOutput() const;

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

/* ===================================================Version 2.0==================================================== */
/**
 * @brief PID控制器(2.0版本)——位置环&速度环串级
 */
class PIDControllerV2 {
    public:
    PIDControllerV2(float Kp_pos, float Ki_pos, float Kd_pos,
                    float Kp_vel, float Ki_vel, float Kd_vel,
                    float Kf_cur,
                    float pos_I_limit, float vel_I_limit,
                    float pos_u_limit, float vel_u_limit);

    void  update(float pos_ref, float pos_fdb, float vel_fdb);

    float getOutput() const;

    void  Clear();

    private:
    float Kp_pos_;
    float Ki_pos_;
    float Kd_pos_;

    float Kp_vel_;
    float Ki_vel_;
    float Kd_vel_;

    float Kf_cur_;

    float pos_I_limit_;
    float vel_I_limit_;

    float pos_u_limit_;
    float vel_u_limit_;

    float pos_pO_;
    float pos_iO_;
    float pos_dO_;

    float vel_pO_;
    float vel_iO_;
    float vel_dO_;

    float cur_fO_;

    float vel_ref_;
    float vel_pre_ref_;

    float pos_error_;
    float vel_error_;

    float pos_pre_error_;
    float vel_pre_error_;

    float u_;
};
/* ===================================================Version 2.0==================================================== */

/* ===================================================Version 3.0==================================================== */
/**
 * @brief PID3.0 理想型PID
 */
class IdealPIDController {
    public:
    IdealPIDController(float kp, float Ti, float Td, float kf, float e_limit, float I_limit, float u_limit, bool isAngle) : kp_(kp),
                                                                                                                            ti_(Ti),
                                                                                                                            td_(Td),
                                                                                                                            kf_(kf),
                                                                                                                            e_Limit_(e_limit),
                                                                                                                            I_Limit_(I_limit),
                                                                                                                            u_Limit_(u_limit),
                                                                                                                            pO_(0.0f),
                                                                                                                            iO_(0.0f),
                                                                                                                            dO_(0.0f),
                                                                                                                            dO_pre_(0.0f),
                                                                                                                            dO_lpf_(0.0f),
                                                                                                                            fO_(0.0f),
                                                                                                                            output_(0.0f),
                                                                                                                            isAnglePID_(isAngle) {
        error_ = 0.0f;
        pre_error_ = 0.0f;
        pre_pre_error_ = 0.0f;
    }

    private:
    float kp_;
    float ti_;
    float td_;
    float kf_;

    float error_;
    float pre_error_;
    float pre_pre_error_;

    float e_Limit_;
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

/* ===================================================Version 3.0==================================================== */

#endif //JIUXIAN_SP_CHASSIS_XPP_ALG_PID_HPP