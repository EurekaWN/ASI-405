#include <stdbool.h>

typedef enum {
    CTRL_MODE_CALIB = 0,
    CTRL_MODE_LEG_LENGTH_HOLD,
    CTRL_MODE_LQR_BALANCE
} ctrl_mode_t;

typedef struct {
    bool imu_bias_ready;
    bool encoder_ready;
    bool actuator_ready;
    bool calib_ready;
    bool system_ready;
} startup_flags_t;

ctrl_mode_t startup_handover(ctrl_mode_t mode, const startup_flags_t *f, bool hold_stable)
{
    if (!(f->imu_bias_ready && f->encoder_ready && f->actuator_ready && f->calib_ready && f->system_ready)) {
        return CTRL_MODE_CALIB;
    }
    if (mode == CTRL_MODE_CALIB) return CTRL_MODE_LEG_LENGTH_HOLD;
    if (mode == CTRL_MODE_LEG_LENGTH_HOLD && hold_stable) return CTRL_MODE_LQR_BALANCE;
    return mode;
}
