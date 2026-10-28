//
// Created by ASUS on 2026/7/16.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_MATHUNIT_HPP
#define JIUXIAN_SP_GIMBAL_XPP_MATHUNIT_HPP

#include <main.h>
#include <math.h>

#define MATHUNIT_MIN(a, b) ((a) < (b) ? (a) : (b))
#define MATHUNIT_MAX(a, b) ((a) > (b) ? (a) : (b))

#define MATHUNIT_ABS(x) ((x) < 0 ? -(x) : (x))

#define MATHUNIT_CLAMP(x, min, max) (MATHUNIT_MIN((max), MATHUNIT_MAX((min), (x))))

float uint_to_float(int X_int, float X_min, float X_max, int Bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

class differential {
public:
    differential(bool init_value) : value_now_(init_value),
                                    value_pre_(0.0f),
                                    differential_out_(0.0f),
                                    differential_start_(false)
    {}

    void clear() {
        this->value_now_ = 0.0f;
        this->value_pre_ = 0.0f;
        this->differential_out_ = 0.0f;
        this->differential_start_ = false;
    }

    double getOutput() const {return this->differential_out_;}

    void differential_calc(double x, double dt) {
        if (this->differential_start_ == false) {
            this->differential_start_ = true;
            this->differential_out_   = 0.0f;
            this->value_pre_          = x;
        }else {
            this->value_now_          = x;
            this->differential_out_   = (this->value_now_ - this->value_pre_) / dt;
            this->value_pre_          = this->value_now_;
        }
    }

private:
    double value_now_;
    double value_pre_;
    double differential_out_;

    bool   differential_start_;
};

#endif //JIUXIAN_SP_GIMBAL_XPP_MATHUNIT_HPP