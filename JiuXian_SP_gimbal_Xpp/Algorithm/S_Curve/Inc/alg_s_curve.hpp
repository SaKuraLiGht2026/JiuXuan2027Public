//
// Created by ASUS on 2026/8/24.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP
#define JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP

#include <iostream>
#include <cmath>
#include <algorithm>

class YawSimple {
public:
    YawSimple(float dt, float max_vel, float max_accel,
              float raw_min, float raw_max)
        : dt_(dt), max_vel_(max_vel), max_accel_(max_accel)
        , raw_min_(raw_min), raw_max_(raw_max)
        , plan_pos_(0), plan_vel_(0) {}

    void reset(float cur) { plan_pos_ = cur; plan_vel_ = 0; }

    float update(float raw, float fb) {
        // 直接映射到 -1 ~ 1
        float x = (raw - raw_min_) / (raw_max_ - raw_min_) * 2.0f - 1.0f;

        // 死区（映射后的 0.05）
        if (fabsf(x) < 0.05f) x = 0.0f;

        // 指数曲线
        float mag = fabsf(x);
        mag = mag * 0.7f + mag * mag * mag * 0.3f;
        float target_vel = mag * (x > 0 ? 1 : -1) * max_vel_;

        // 加速度限幅
        float dv = target_vel - plan_vel_;
        float max_dv = max_accel_ * dt_;
        dv = (dv > max_dv) ? max_dv : (dv < -max_dv ? -max_dv : dv);
        plan_vel_ += dv;

        // 积分，不夹紧
        plan_pos_ += plan_vel_ * dt_;

        return plan_pos_;
    }

private:
    float dt_, max_vel_, max_accel_;
    float raw_min_, raw_max_;
    float plan_pos_, plan_vel_;
};

//
class SCurveAnglePlanner {
public:
    SCurveAnglePlanner(float max_speed, float max_accel)
        : max_speed(max_speed), max_accel(max_accel),
          angle(0), speed(0), target_angle(0) {}

    void set_target(float tangle) {
        // ★ 把外部目标 unwrap 到当前角度所在的"圈"
        float diff = tangle - angle;
        // 归一化 diff 到 -180~+180
        if (diff > 180.0f) diff -= 360.0f;
        else if (diff < -180.0f) diff += 360.0f;
        target_angle = angle + diff;  // 保证 target 和 angle 在同一圈
    }

    void update(float dt) {
        float angle_err = target_angle - angle;

        float v_des = angle_err * (max_speed / 5.0f);
        v_des = fmaxf(-max_speed, fminf(max_speed, v_des));

        float v_err = v_des - speed;
        float max_dv = max_accel * dt;
        if (v_err > max_dv) v_err = max_dv;
        if (v_err < -max_dv) v_err = -max_dv;
        speed += v_err;

        angle += speed * dt;
    }

    float get_angle() const { return angle; }
    float get_speed() const { return speed; }

    void reset(float a = 0.0f) {
        angle = target_angle = a;
        speed = 0.0f;
    }

private:
    float max_speed;
    float max_accel;
    float angle;
    float speed;
    float target_angle;
};

// 将角度归一化到 [-180, 180]
static inline float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 计算从 from 到 to 的最短角度差（结果在 [-180, 180]）
static inline float angleDifference(float from, float to) {
    return normalizeAngle(to - from);
}

class YawSmoother {
public:
    /**
     * @brief 构造函数
     * @param max_vel  最大跟踪速度 (°/s)，推荐 200~400
     * @param max_acc  最大加速度 (°/s²)，推荐 1000~2000
     * @param kp       PD 刚度系数（越大跟得越紧）
     */
    YawSmoother(float max_vel = 300.0f, float max_acc = 1800.0f, float kp = 100.0f)
        : max_vel_(max_vel), max_acc_(max_acc), kp_(kp) {
        kd_ = 2.0f * sqrtf(kp_);  // 临界阻尼
        reset(0.0f);
    }

    // 重置状态
    void reset(float current_yaw_deg) {
        smooth_pos_ = normalizeAngle(current_yaw_deg);
        smooth_vel_ = 0.0f;
    }

    /**
     * @brief 每个控制周期调用
     * @param target_yaw_deg  视觉/预测给出的目标角度（度）
     * @param dt               控制周期（秒），比如 0.001 表示 1kHz
     */
    void update(float target_yaw_deg, float dt) {
        // 1. 计算最短角度误差（度）
        float error = angleDifference(smooth_pos_, target_yaw_deg);

        float acc_limit = max_acc_;
        if (fabsf(error) > 5.0f) {       // 偏差 > 5° 判定为"需要拉枪"
            acc_limit = max_acc_ * 1.5f;  // 瞬间放宽到 1.5 倍
        }

        float acc = kp_ * error - kd_ * smooth_vel_;
        if (acc > acc_limit) acc = acc_limit;
        if (acc < -acc_limit) acc = -acc_limit;

        // 4. 积分速度（°/s）
        smooth_vel_ += acc * dt;

        // 5. 速度限幅
        if (smooth_vel_ > max_vel_) smooth_vel_ = max_vel_;
        if (smooth_vel_ < -max_vel_) smooth_vel_ = -max_vel_;

        // 6. 积分位置（°）
        smooth_pos_ += smooth_vel_ * dt;

        // 7. 归一化到 [-180, 180]
        smooth_pos_ = normalizeAngle(smooth_pos_);
    }

    // 获取平滑后的期望角度（°）
    float getSmoothPosition() const { return smooth_pos_; }

    // 获取平滑后的期望速度（°/s），可用于前馈
    float getSmoothVelocity() const { return smooth_vel_; }

    // 运行时调参
    void setGains(float kp, float max_vel, float max_acc) {
        kp_ = kp;
        kd_ = 2.0f * sqrtf(kp_);
        max_vel_ = max_vel;
        max_acc_ = max_acc;
    }

private:
    float max_vel_;   // °/s
    float max_acc_;   // °/s²
    float kp_;        // 刚度
    float kd_;        // 阻尼

    float smooth_pos_; // 平滑后的期望位置（°）
    float smooth_vel_; // 平滑后的期望速度（°/s）
};

#endif //JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP