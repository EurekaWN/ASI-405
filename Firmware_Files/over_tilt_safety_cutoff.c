#include <stdbool.h>
#include <math.h>

typedef struct {
    float pitch_deg;
    float pitch_limit_deg;
    bool over_tilt_latched;
} safety_state_t;

static void set_all_motor_torque_zero(void)
{
    // Replace with actual motor command interface.
}

bool over_tilt_protection_step(safety_state_t *s)
{
    if (fabsf(s->pitch_deg) >= s->pitch_limit_deg) {
        s->over_tilt_latched = true;
    }

    if (s->over_tilt_latched) {
        set_all_motor_torque_zero();   // immediate torque cut-off
        return false;                  // balancing disabled
    }

    return true;                       // balancing allowed
}
