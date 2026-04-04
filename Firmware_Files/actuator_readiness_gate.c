#include <stdbool.h>

typedef struct {
    float test_cmd;
    float measured_speed;
    bool drv_ok;
} motor_diag_t;

static bool sign_match(float cmd, float speed)
{
    if (cmd > 0.0f && speed < 0.0f) return false;
    if (cmd < 0.0f && speed > 0.0f) return false;
    return true;
}

bool actuator_ready_check(const motor_diag_t *m)
{
    if (!m->drv_ok) return false;
    if (!sign_match(m->test_cmd, m->measured_speed)) return false;
    return true;
}
