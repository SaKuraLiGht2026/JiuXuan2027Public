//
// Created by ASUS on 2026/8/11.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_ALG_SMC_HPP
#define JIUXIAN_SP_CHASSIS_XPP_ALG_SMC_HPP

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <cmath>

class SMC{
public:
    float C;
    float K;
    float target;
    float error_eps;
    float q;
    float p;
    float u_max;
    float J;
    float angle;
    float ang_vel;
    float epsilon;

    bool  nearby;

    float u;

    SMC(float C,float K,float error_eps,float q,float p,float u_max,float J,float epsilon):
    C(C),K(K),target(0.0f),error_eps(error_eps),q(q),p(p),u_max(u_max),J(J),epsilon(epsilon){}

    void SMC_Update(float angle_now,float angle_vel);

    [[nodiscard]] float gainErr() const {return error;}


private:
    float error;
    float error_last;
    float target_dot;
    float target_ddot;
    float target_last;

    float s;
    float ds;
    float Sat(float y)
    {
        if (fabs(y) <= 1)
            return y;
        else
            return Signal(y);
    }

    int8_t Signal(float y)
    {
        if (y > 0)
            return 1;
        else if (y == 0)
            return 0;
        else
            return -1;
    }
};

#endif //JIUXIAN_SP_CHASSIS_XPP_ALG_SMC_HPP