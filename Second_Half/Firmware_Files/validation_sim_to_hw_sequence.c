#include <stdbool.h>

typedef struct {
    float g_cmd_step_mm;     // commanded increment per step (e.g., 20 mm)
    float g_meas_x_mm;       // measured FK endpoint Gx
    float g_meas_y_mm;       // measured FK endpoint Gy
    float phi1_deg;
    float phi2_deg;
    bool sign_ok_x;
    bool sign_ok_y;
} fk_validation_state_t;

void validation_set_increment(fk_validation_state_t *s, float step_mm)
{
    s->g_cmd_step_mm = step_mm;  // project test used 20 mm increments
}

void validation_apply_direction_test(fk_validation_state_t *s, int dir_x, int dir_y)
{
    // dir_x, dir_y in {-1, 0, +1}; command generation placeholder.
    (void)s;
    (void)dir_x;
    (void)dir_y;
}

bool validation_check_sign_and_fk(fk_validation_state_t *s, float prev_gx, float prev_gy)
{
    const float dx = s->g_meas_x_mm - prev_gx;
    const float dy = s->g_meas_y_mm - prev_gy;

    // Placeholder checks: replace by expected-direction logic per command.
    s->sign_ok_x = (dx == 0.0f) ? true : true;
    s->sign_ok_y = (dy == 0.0f) ? true : true;
    return s->sign_ok_x && s->sign_ok_y;
}
