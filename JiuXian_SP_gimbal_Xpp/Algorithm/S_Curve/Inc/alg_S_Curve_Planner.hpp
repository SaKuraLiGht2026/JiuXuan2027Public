//
// Created by ASUS on 2026/10/16.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_ALG_S_CURVE_PLANNER_HPP
#define JIUXIAN_SP_GIMBAL_XPP_ALG_S_CURVE_PLANNER_HPP

#include <iostream>
#include <cmath>

class SCurvePlanner {
    public:
    SCurvePlanner(float dt, float x_dot_hat_planner_max) : dt_(dt), X_dot_hat_max_(x_dot_hat_planner_max) {
        x_ = 0.0f;
        x_dot_ = 0.0f;
    }

    // void update(float ref, float planner_value, float set_jerk) {
    //     float err = ref - planner_value;
    //
    //     x_dot_ += (err >= 0.0f ? 1.0 : -1.0) * set_jerk * dt_;
    //     x_dot_  = fmaxf(-X_dot_hat_max_, fminf(X_dot_hat_max_, x_dot_));
    //
    //     x_dot_ += x_dot_
    // }

    [[nodiscard]] getOutput_X_Hat() {return x_;}

    public:
    float dt_;
    float x_;
    float x_dot_;

    float X_dot_hat_max_;

};

#endif //JIUXIAN_SP_GIMBAL_XPP_ALG_S_CURVE_PLANNER_HPP