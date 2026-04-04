#include <stdbool.h>
#include <stdint.h>

#define JOINT_CH 4

typedef struct {
    bool online;
    uint32_t age_ms;
    float angle_deg;
} encoder_ch_t;

bool encoder_channels_valid(const encoder_ch_t ch[JOINT_CH], uint32_t max_age_ms)
{
    for (int i = 0; i < JOINT_CH; ++i) {
        if (!ch[i].online) return false;
        if (ch[i].age_ms > max_age_ms) return false;
        if (ch[i].angle_deg < -720.0f || ch[i].angle_deg > 720.0f) return false;
    }
    return true;
}
