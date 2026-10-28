//
// Created by ASUS on 2026/8/24.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP
#define JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP

#include <iostream>
#include <cmath>
#include <algorithm>

//
class SCurvePlanner {
public:
    SCurvePlanner(float max_accel, float max_jerk, float zero_th = 0.005f)
        : max_accel(max_accel), max_jerk(max_jerk), zero_th(zero_th),
          speed(0), accel(0), jerk(0), target_speed(0),
          dir_x(1), dir_y(0), out_vx(0), out_vy(0), frozen(true) {}

    void set_target(float vx, float vy) {
        float mag_sq = vx * vx + vy * vy;

        if (mag_sq < 1e-12f) {
            target_speed = 0.0f;
            return;
        }

        float mag = sqrtf(mag_sq);

        // ★★★ 核心修复：目标速度太小，不更新方向 ★★★
        // 防止回中噪声把 dir_x/dir_y 污染
        // 0.05 是你最大速度的 5% 左右，根据你的摇杆量程调
        if (mag < 0.05f) {
            target_speed = mag;
            frozen = false;
            return;
        }

        float nx = vx / mag;
        float ny = vy / mag;

        float dot = dir_x * nx + dir_y * ny;

        if (dot < 0.9997f) {
            speed = speed * dot;
            if (speed < 0.0f) speed = 0.0f;
            dir_x = nx;
            dir_y = ny;
        }

        target_speed = mag;
        frozen = false;
    }

    void update(float dt) {
        if (frozen) return;

        if (target_speed < 1e-6f) {
            if (speed < zero_th && fabsf(accel) < zero_th * 2.0f) {
                speed = accel = jerk = 0.0f;
                target_speed = 0.0f;
                out_vx = out_vy = 0.0f;
                dir_x = 1.0f;
                dir_y = 0.0f;
                frozen = true;
                return;
            }
        }

        float err = target_speed - speed;
        float a_des = err * (max_accel / 0.15f);
        a_des = fmaxf(-max_accel, fminf(max_accel, a_des));

        float a_err = a_des - accel;
        jerk = fmaxf(-max_jerk, fminf(max_jerk, a_err / dt));

        accel += jerk * dt;
        accel = fmaxf(-max_accel, fminf(max_accel, accel));

        speed += accel * dt;
        speed = fmaxf(0.0f, speed);

        out_vx = speed * dir_x;
        out_vy = speed * dir_y;
    }

    float get_vx() const { return out_vx; }
    float get_vy() const { return out_vy; }
    bool  is_stopped() const { return frozen; }

    void force_zero() {
        speed = accel = jerk = target_speed = 0.0f;
        out_vx = out_vy = 0.0f;
        dir_x = 1.0f;
        dir_y = 0.0f;
        frozen = true;
    }

private:
    float max_accel, max_jerk, zero_th;
    float speed, accel, jerk;
    float target_speed;
    float dir_x, dir_y;
    float out_vx, out_vy;
    bool frozen;
};

#endif //JIUXIAN_SP_CHASSIS_XPP_ALG_S_CURVE_HPP