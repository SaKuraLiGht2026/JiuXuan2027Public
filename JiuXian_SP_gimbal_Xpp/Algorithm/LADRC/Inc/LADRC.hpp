//
// Created by ASUS on 2026/9/27.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_LADRC_HPP
#define JIUXIAN_SP_GIMBAL_XPP_LADRC_HPP

class LADRC {
public:
    LADRC(float J, float wc, float wo, float u_min, float u_max, float Ts) :
        wc_(wc), wo_(wo), u_min_(u_min), u_max_(u_max), Ts_(Ts) {

        if (J <= 0.0f) {
            J = 1.0f; // 避免除以零
        }
        b0_ = 1.0f / J;

        kp_ = wc_ * wc_;
        kd_ = 2.0f * wc_;

        beta1_ = 3.0f * wo_;
        beta2_ = 3.0f * wo_ * wo_;
        beta3_ = wo_ * wo_ * wo_;

        z1_ = 0.0f;
        z2_ = 0.0f;
        z3_ = 0.0f;

        u_raw_ = 0.0f;
        angle_meas_ = 0.0f;
        vel_meas_ = 0.0f;
    };

    void set_J(float J){
        if (J <= 0.0f) {
            J = 1.0f; // 避免除以零
        }
        b0_ = 1.0f / J;
    }

    void set_wc(float wc){wc_ = wc; kp_ = wc_ * wc_; kd_ = 2.0f * wc_;}

    void set_wo(float wo){wo_ = wo; beta1_ = 3.0f * wo_; beta2_ = 3.0f * wo_ * wo_; beta3_ = wo_ * wo_ * wo_;}

    void set_u_limits(float u_min, float u_max){u_min_ = u_min; u_max_ = u_max;}

    void set_Ts(float Ts){Ts_ = Ts;}

    void update(float angle_ref, float angle_meas, float vel_meas) {
        angle_meas_ = angle_meas;
        vel_meas_   = vel_meas;

        float y   = angle_meas;
        float err = z1_ - y;

        z1_ += Ts_ * (z2_ - beta1_ * err);
        z2_ += Ts_ * (z3_ - beta2_ * err + b0_ * u_raw_);
        z3_ -= Ts_ * (beta3_ * err);

        float angle_err = z1_ - angle_ref > 180.0f ? z1_ - angle_ref - 360.0f : (z1_ - angle_ref < -180.0f ? z1_ - angle_ref + 360.0f : z1_ - angle_ref);
        u_raw_ = (-kp_ * angle_err + kd_ * z2_ - z3_) / b0_;

        if (u_raw_ > u_max_) {
            u_raw_ = u_max_;
        } else if (u_raw_ < u_min_) {
            u_raw_ = u_min_;
        }
    }

    [[nodiscard]] float get_control_output() const { return u_raw_;}

    [[nodiscard]] float get_vel_meas() const { return vel_meas_;}

    [[nodiscard]] float get_observer_state_z1() const { return z1_;}

    [[nodiscard]] float get_observer_state_z2() const { return z2_;}

    [[nodiscard]] float get_observer_state_z3() const { return z3_;}

    void reset() {
        z1_ = 0.0f;
        z2_ = 0.0f;
        z3_ = 0.0f;
        u_raw_ = 0.0f;
    }

private:
    /* ---------- 状态变量 ---------- */
    float z1_;          // 观测角度 (rad)
    float z2_;          // 观测角速度 (rad/s)
    float z3_;          // 观测总扰动

    /* ---------- 控制参数 ---------- */
    float b0_;          // 控制增益，b0 ≈ 1/J_nominal (J: 标称转动惯量)
    float kp_;          // 比例增益 = wc^2
    float kd_;          // 微分增益 = 2*zeta*wc
    float wc_;          // 控制器带宽 (rad/s)

    /* ---------- LESO 参数 ---------- */
    float beta1_;       // LESO 增益1 = 3*wo
    float beta2_;       // LESO 增益2 = 3*wo^2
    float beta3_;       // LESO 增益3 = wo^3
    float wo_;          // 观测器带宽 (rad/s)

    /* ---------- 输出限幅 ---------- */
    float u_min_;       // 控制量下限 (给速度环的指令或力矩)
    float u_max_;       // 控制量上限

    /* ---------- 采样时间 ---------- */
    float Ts_;          // 控制周期 (s)

    /* ---------- 中间变量（调试用） ---------- */
    float u_raw_;       // 限幅前控制量
    float angle_meas_;  // 当前测量角度 (rad, 连续)
    float vel_meas_;    // 当前测量角速度 (rad/s, 差分)
};

#endif //JIUXIAN_SP_GIMBAL_XPP_LADRC_HPP