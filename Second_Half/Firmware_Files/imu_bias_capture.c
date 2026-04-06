#include <stdbool.h>

typedef struct {
    float gx_bias;
    float gy_bias;
    float gz_bias;
} imu_bias_t;

bool imu_capture_stationary_bias(imu_bias_t *b)
{
    // Stationary bias capture entry point.
    b->gx_bias = 0.0f;
    b->gy_bias = 0.0f;
    b->gz_bias = 0.0f;
    return true;
}

void imu_apply_bias(float *gx, float *gy, float *gz, const imu_bias_t *b)
{
    *gx -= b->gx_bias;
    *gy -= b->gy_bias;
    *gz -= b->gz_bias;
}
