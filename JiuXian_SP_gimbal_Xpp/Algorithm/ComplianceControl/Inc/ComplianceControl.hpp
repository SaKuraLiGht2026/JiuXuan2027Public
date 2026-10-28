//
// Created by ASUS on 2026/9/27.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_COMPLIANCECONTROL_HPP
#define JIUXIAN_SP_GIMBAL_XPP_COMPLIANCECONTROL_HPP

class ComplianceControl {
public:
    ComplianceControl(float K_spring, float B_damper, float compliance_torque_max, float compliance_torque_min) : K_spring_(K_spring), B_damper_(B_damper), err_(0.0f), err_dot_(0.0f), compliance_torque_(0.0f), compliance_torque_max_(compliance_torque_max), compliance_torque_min_(compliance_torque_min) {}
    void update(float ref, float fdb, float fdb_dot) {
        float err = ref - fdb > 180.0f ? ref - fdb - 360.0f : (ref - fdb < -180.0f ? ref - fdb + 360.0f : ref - fdb);
        float err_dot = -fdb_dot;
        err_ = err;
        err_dot_ = err_dot;
        compliance_torque_ = K_spring_ * err_ + B_damper_ * err_dot_;
        if (compliance_torque_ > compliance_torque_max_) {
            compliance_torque_ = compliance_torque_max_;
        } else if (compliance_torque_ < compliance_torque_min_) {
            compliance_torque_ = compliance_torque_min_;
        }
    }

    float getComplianceTorque() const { return compliance_torque_; }

private:
    float K_spring_;
    float B_damper_;

    float err_;
    float err_dot_;

    float compliance_torque_;

    float compliance_torque_max_;
    float compliance_torque_min_;
};

#endif //JIUXIAN_SP_GIMBAL_XPP_COMPLIANCECONTROL_HPP