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

#endif //JIUXIAN_SP_GIMBAL_XPP_MATHUNIT_HPP