#include <stdint.h>

void ht8108_pack_and_send(float p, float v, float kp, float kd, float t, int motor_id)
{
    // command clamp + packing placeholder
    (void)p; (void)v; (void)kp; (void)kd; (void)t; (void)motor_id;
    // send CAN frame here
}
