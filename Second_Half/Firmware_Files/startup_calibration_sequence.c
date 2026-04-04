#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool imu_bias_ready;
    bool encoder_ready;
    bool actuator_ready;
    bool calib_ready;
    bool system_ready;
} startup_flags_t;

typedef enum {
    CTRL_MODE_CALIB = 0,
    CTRL_MODE_LEG_LENGTH_HOLD,
    CTRL_MODE_LQR_BALANCE
} ctrl_mode_t;

static startup_flags_t g_flags;
static ctrl_mode_t g_mode = CTRL_MODE_CALIB;

void startup_reset_flags_and_outputs(void)
{
    g_flags.imu_bias_ready = false;
    g_flags.encoder_ready = false;
    g_flags.actuator_ready = false;
    g_flags.calib_ready = false;
    g_flags.system_ready = false;
    g_mode = CTRL_MODE_CALIB;
    // reset motor outputs to zero here
}
