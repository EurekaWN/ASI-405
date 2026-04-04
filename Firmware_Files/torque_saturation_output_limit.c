#include <math.h>

static float clipf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float torque_limit_step(float u, float umax, float *u_prev, float du_max)
{
    float u_sat = clipf(u, -umax, umax);
    float du = u_sat - *u_prev;
    du = clipf(du, -du_max, du_max);
    *u_prev += du;
    return *u_prev;
}
