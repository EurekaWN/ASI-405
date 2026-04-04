#include <math.h>

typedef struct {
    float kp;
    float kd;
    float ff_ground_N;
    float ff_air_N;
} leg_length_tune_t;

float leg_length_pd_step(float len_ref, float len_meas, float dlen_meas, const leg_length_tune_t *t, int on_ground)
{
    float err = len_ref - len_meas;
    float ff = on_ground ? t->ff_ground_N : t->ff_air_N;
    float u = t->kp * err - t->kd * dlen_meas + ff;
    return u;
}

void init_leg_length_feedforward(leg_length_tune_t *t)
{
    const float M = 8.7f;
    const float mp = 4.32716f;
    const float mw = 1.58832f;
    const float g = 9.81f;
    const float cos_theta_nom = 1.0f; // theta_nom = 0

    t->ff_ground_N = 0.5f * (M + mp + mw) * g / cos_theta_nom;
    t->ff_air_N = 0.5f * (mp + mw) * g / cos_theta_nom;
}
