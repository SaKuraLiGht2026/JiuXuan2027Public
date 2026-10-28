//
// Created by ASUS on 2026/9/28.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_RAMP_HPP
#define JIUXIAN_SP_CHASSIS_XPP_RAMP_HPP

class Ramp_Planner {
public:

    Ramp_Planner(float ramp, float dt) : ramp_(ramp), dt_(dt), planner_speed_(0.0f) {}

    float update(float target_speed) {
        float speed_diff = target_speed - planner_speed_;
        float max_change = ramp_ * dt_;

        if (speed_diff > max_change) {
            planner_speed_ += max_change;
        } else if (speed_diff < -max_change) {
            planner_speed_ -= max_change;
        } else {
            planner_speed_ = target_speed;
        }

        return planner_speed_;
    }

private:

    float ramp_;
    float dt_;
    float planner_speed_;
};

#endif //JIUXIAN_SP_CHASSIS_XPP_RAMP_HPP