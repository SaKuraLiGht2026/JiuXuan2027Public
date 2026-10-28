//
// Created by ASUS on 2026/8/11.
//
#include "../Inc/alg_smc.hpp"

void SMC::SMC_Update(float angle_now,float angle_vel)
{
    angle = angle_now;
    ang_vel = angle_vel;
    float e_qp;
    float qp = q/p;

    if (nearby) error = (angle - target > 180) ? (angle - target - 360) : ((angle - target < -180) ? (angle - target + 360) : (angle - target));
    else        error =  angle - target;
    float error_dot = ang_vel - target_dot;
    target_ddot = (target - target_last) - target_dot;
    target_dot = (target - target_last);

    if (fabs(error) < error_eps)
    {
        u = 0;
        return;
    }

    if (error < 0)
        e_qp = -pow(abs(error), qp);
    else
        e_qp = pow(abs(error), qp);

    s = error_dot + C * e_qp;
    ds = -epsilon * tanh(s) - K * s;
    u = J * (target_ddot + ds - C * qp * error_dot * e_qp / abs(error) );

    if (abs(error) < 5)
    {
        error = (angle - target > 180) ? (angle - target - 360) : ((angle - target < -180) ? (angle - target + 360) : (angle - target));

        s = C * error + error_dot;//smc surface
        u = J * (target_ddot - C * error_dot - epsilon * tanh(s) - K * s);
    }

    if (u > u_max)
        u = u_max;
    if (u < -u_max)
        u = -u_max;

    target_last = target;
}
