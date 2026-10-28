//
// Created by ASUS on 2026/4/5.
//
#include "../Inc/alg_pid.hpp"
#include <assert.h>

    PIDController::PIDController(float kp, float ki, float kd, float kf, float I_Limit, float u_Limit, bool isAnglePID) : kp_(kp),
                                                                                                                          ki_(ki),
                                                                                                                          kd_(kd),
                                                                                                                          kf_(kf),
                                                                                                                          error_(0.0f),
                                                                                                                          pre_error_(0.0f),
                                                                                                                          pre_pre_error_(0.0f),
                                                                                                                          I_Limit_(I_Limit),
                                                                                                                          u_Limit_(u_Limit),
                                                                                                                          pO_(0.0f),
                                                                                                                          iO_(0.0f),
                                                                                                                          dO_(0.0f),
                                                                                                                          dO_pre_(0.0f),
                                                                                                                          dO_lpf_(0.0f),
                                                                                                                          fO_(0.0f),
                                                                                                                          output_(0.0f),
                                                                                                                          isAnglePID_(isAnglePID)
    {

    }

    bool PIDController::update(float reference, float measure) {
        this->error_ = reference - measure;
        if (this->isAnglePID_) {
            this->error_ = (this->error_ > 180.0f ? this->error_ - 360.0f : (this->error_ < -180.0f ? this->error_ + 360.0f : this->error_));
        }
        this->pO_  = this->kp_ *  this->error_;
        this->iO_ += this->ki_ *  this->error_;
        this->dO_pre_ = this->dO_;
        this->dO_  = this->kd_ * (this->error_ - this->pre_error_);
        this->dO_lpf_ = 0.1f * this->dO_ + 0.9f * this->dO_pre_;
        this->fO_  = this->kf_ *  this->pre_pre_error_;

        this->iO_  = (this->iO_ > this->I_Limit_ ? this->I_Limit_ : (this->iO_ < -this->I_Limit_ ? -this->I_Limit_ : this->iO_));

        this->output_ = this->pO_ + this->iO_ + this->dO_lpf_ + this->fO_;

        this->output_ = (this->output_ > u_Limit_ ? u_Limit_ : (this->output_ < -u_Limit_ ? -u_Limit_ : this->output_));

        this->pre_pre_error_ = this->pre_error_;
        this->pre_error_     = this->error_;

        return true;
    }

    float PIDController::getOutput() const {
        return this->output_;
    }

    void PIDController::Clear() {
        this->iO_            = 0.0f;
        this->output_        = 0.0f;
        this->error_         = 0.0f;
        this->pre_error_     = 0.0f;
        this->pre_pre_error_ = 0.0f;
    }

/* ===================================================Version 2.0==================================================== */
/**
 *
 * @param Kp_pos
 * @param Ki_pos
 * @param Kd_pos
 * @param Kp_vel
 * @param Ki_vel
 * @param Kd_vel
 * @param Kf_cur
 * @param pos_I_limit
 * @param vel_I_limit
 * @param pos_u_limit
 * @param vel_u_limit
 */
PIDControllerV2::PIDControllerV2(float Kp_pos, float Ki_pos, float Kd_pos,
                                 float Kp_vel, float Ki_vel, float Kd_vel,
                                 float Kf_cur,
                                 float pos_I_limit, float vel_I_limit,
                                 float pos_u_limit, float vel_u_limit)
                                                                       : Kp_pos_(Kp_pos),
                                                                         Ki_pos_(Ki_pos),
                                                                         Kd_pos_(Kd_pos),
                                                                         Kp_vel_(Kp_vel),
                                                                         Ki_vel_(Ki_vel),
                                                                         Kd_vel_(Kd_vel),
                                                                         Kf_cur_(Kf_cur),
                                                                         pos_I_limit_(pos_I_limit),
                                                                         vel_I_limit_(vel_I_limit),
                                                                         pos_u_limit_(pos_u_limit),
                                                                         vel_u_limit_(vel_u_limit)
{
    assert(pos_I_limit_ >= 0.0f);
    assert(vel_I_limit_ >= 0.0f);
    assert(pos_u_limit_ >= 0.0f);
    assert(vel_u_limit_ >= 0.0f);
}

void PIDControllerV2::update(float pos_ref, float pos_fdb, float vel_fdb) {
    // 位置环
    this->pos_error_ = (pos_ref - pos_fdb > 180.0f ? pos_ref - pos_fdb - 360.0f : (pos_ref - pos_fdb < -180.0f ? pos_ref - pos_fdb + 360.0f : pos_ref - pos_fdb));

    this->pos_pO_  = this->Kp_pos_ *  this->pos_error_;
    this->pos_iO_ +=(this->Ki_pos_ *  this->pos_error_ > this->pos_I_limit_ ? this->pos_I_limit_ : (this->Ki_pos_ * this->pos_error_ < -this->pos_I_limit_ ? -this->pos_I_limit_ : this->Ki_pos_ * this->pos_error_));
    this->pos_dO_  = this->Kd_pos_ * (this->pos_error_ - this->pos_pre_error_);

    this->vel_ref_ = (this->pos_pO_ + this->pos_iO_ + this->pos_dO_ > this->vel_u_limit_ ? this->vel_u_limit_ : (this->pos_pO_ + this->pos_iO_ + this->pos_dO_ < -this->vel_u_limit_ ? -this->vel_u_limit_ : this->pos_pO_ + this->pos_iO_ + this->pos_dO_));
    this->pos_pre_error_ = this->pos_error_;

    // 速度环
    this->vel_error_ = this->vel_ref_ - vel_fdb;

    this->vel_pO_  = this->Kp_vel_ *  this->vel_error_;
    this->vel_iO_ +=(this->Ki_vel_ *  this->vel_error_ > this->vel_I_limit_ ? this->vel_I_limit_ : (this->Ki_vel_ * this->vel_error_ < -this->vel_I_limit_ ? -this->vel_I_limit_ : this->Ki_vel_ * this->vel_error_));
    this->vel_dO_  = this->Kd_vel_ * (this->vel_error_ - this->vel_pre_error_);

    this->cur_fO_  = this->Kf_cur_ * (this->vel_ref_ - this->vel_pre_ref_);

    this->u_       =(this->vel_pO_ + this->vel_iO_ + this->vel_dO_ + this->cur_fO_ > this->pos_u_limit_ ? this->pos_u_limit_ : (this->vel_pO_ + this->vel_iO_ + this->vel_dO_ + this->cur_fO_ < -this->pos_u_limit_ ? -this->pos_u_limit_ : this->vel_pO_ + this->vel_iO_ + this->vel_dO_ + this->cur_fO_));

    this->vel_pre_error_ = this->vel_error_;
    this->vel_pre_ref_   = this->vel_ref_  ;
}

float PIDControllerV2::getOutput() const {
    return this->u_;
}

void PIDControllerV2::Clear() {
    this->pos_pO_        = 0.0f;
    this->pos_iO_        = 0.0f;
    this->pos_dO_        = 0.0f;
    this->vel_pO_        = 0.0f;
    this->vel_iO_        = 0.0f;
    this->vel_dO_        = 0.0f;
    this->cur_fO_        = 0.0f;
    this->u_             = 0.0f;
    this->pos_error_     = 0.0f;
    this->vel_error_     = 0.0f;
    this->pos_pre_error_ = 0.0f;
    this->vel_pre_error_ = 0.0f;
    this->vel_pre_ref_   = 0.0f;
}
