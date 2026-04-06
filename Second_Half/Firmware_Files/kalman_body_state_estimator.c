#include <string.h>

typedef struct {
    float x;
    float vx;
} kf_state_t;

typedef struct {
    float q;
    float r;
    float p11, p12, p21, p22;
} kf_cov_t;

void kf_step(kf_state_t *s, kf_cov_t *c, float dt, float z_x, float z_v)
{
    // simple constant-velocity prediction
    s->x += dt * s->vx;
    c->p11 += dt * (c->p12 + c->p21) + dt * dt * c->p22 + c->q;

    // Measurement correction for the reduced two-state estimator.
    s->x  += 0.2f * (z_x - s->x);
    s->vx += 0.2f * (z_v - s->vx);
    c->p11 *= 0.9f;
    c->p22 *= 0.9f;
}
