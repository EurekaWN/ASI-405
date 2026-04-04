#include <stdint.h>

typedef struct {
    int16_t rpm;
} m3508_raw_t;

float m3508_speed_mps(const m3508_raw_t *m, float rpm_to_mps, int sign)
{
    return sign * m->rpm * rpm_to_mps;
}

float wheel_center_speed(float vL, float vR)
{
    return 0.5f * (vL + vR);
}
