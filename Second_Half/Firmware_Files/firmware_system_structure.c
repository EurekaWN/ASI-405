#include <stdbool.h>

typedef struct {
    bool acquisition_ok;
    bool estimation_ok;
    bool control_ok;
    bool safety_ok;
} fw_pipe_state_t;

bool firmware_pipeline_step(fw_pipe_state_t *s)
{
    s->acquisition_ok = true; // IMU/encoder/motor frames fresh
    s->estimation_ok = s->acquisition_ok;
    s->control_ok = s->estimation_ok;
    s->safety_ok = s->control_ok;
    return s->safety_ok;
}
