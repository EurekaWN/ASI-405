#include <math.h>

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
} imu_raw_t;

typedef struct {
    float gx_b, gy_b, gz_b;
} imu_bias_t;

typedef struct {
    float pitch_fw_deg;
    float dphi_model_dps;
} imu_proc_out_t;

void imu_process(const imu_raw_t *r, const imu_bias_t *b, imu_proc_out_t *o)
{
    const float gy_corr = r->gy - b->gy_b;
    // firmware pitch convention -> model sign mapping
    o->pitch_fw_deg = atan2f(r->ax, r->az) * 57.2957795f;
    o->dphi_model_dps = -gy_corr;
}
