//
// Created by ASUS on 2026/7/16.
//
#include "../Inc/PID.hpp"

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
        this->dO_lpf_ = 0.9f * this->dO_ + 0.1f * this->dO_pre_;
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
