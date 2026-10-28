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


/**
 * @brief 速度斜坡规划器（Header-only）
 *
 * 按照加速度/减速度限制，从当前速度平滑过渡到目标速度。
 * 每个控制周期调用 update() 获取速度指令。
 */
class VelocityRampPlanner {
public:
    /**
     * @param max_accel 最大加速度 (正值, 单位/秒²)
     * @param max_decel 最大减速度 (正值, 单位/秒²)
     * @param dt        控制周期 (秒)
     */
    VelocityRampPlanner(double max_accel, double max_decel, double dt)
        : max_accel_(fabs(max_accel))
        , max_decel_(fabs(max_decel))
        , dt_(dt)
        , current_vel_(0.0)
        , target_vel_(0.0)
        , ramping_(false) {}

    /// 设置目标速度（可正可负）
    void setTarget(double target_vel) {
        target_vel_ = target_vel;
        ramping_ = true;
    }

    /// 每个控制周期调用，返回当前速度指令
    double update() {
        double error = target_vel_ - current_vel_;

        if (fabs(error) <= 1e-9) {
            current_vel_ = target_vel_;
            ramping_ = false;
            return current_vel_;
        }

        double accel_lim = (error > 0) ? max_accel_ : max_decel_;
        double step = accel_lim * dt_;

        if (fabs(error) <= step) {
            current_vel_ = target_vel_;
            ramping_ = false;
        } else {
            current_vel_ += (error > 0) ? step : -step;
        }

        return current_vel_;
    }

    double getCurrentVel() const { return current_vel_; }
    double getTargetVel()  const { return target_vel_; }
    bool   isRamping()    const { return ramping_; }

    /// 重置（可指定初始速度）
    void reset(double init_vel = 0.0) {
        current_vel_ = init_vel;
        target_vel_  = init_vel;
        ramping_     = false;
    }

private:
    double max_accel_;
    double max_decel_;
    double dt_;
    double current_vel_;
    double target_vel_;
    bool   ramping_;
};

#endif //JIUXIAN_SP_CHASSIS_XPP_RAMP_HPP