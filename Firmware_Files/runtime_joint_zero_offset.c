#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define JOINT_COUNT 4

typedef enum {
    JOINT_LEFT_HIP = 0,
    JOINT_LEFT_KNEE,
    JOINT_RIGHT_HIP,
    JOINT_RIGHT_KNEE
} joint_id_t;

typedef struct {
    float ref_true_deg;          // theta_ref,true from jig geometry
    float encoder_ref_deg;       // encoder reading at capture pose
    float offset_deg;            // computed: encoder_ref_deg - ref_true_deg
} joint_zero_offset_t;

static joint_zero_offset_t g_joint_zero_table[JOINT_COUNT] = {
    [JOINT_LEFT_HIP]  = { 0.00f, 12.40f, 0.00f },
    [JOINT_LEFT_KNEE] = { 0.00f, -8.15f, 0.00f },
    [JOINT_RIGHT_HIP] = { 0.00f, 11.90f, 0.00f },
    [JOINT_RIGHT_KNEE]= { 0.00f, -7.80f, 0.00f }
};

static float wrap_pm180(float a_deg)
{
    while (a_deg > 180.0f) a_deg -= 360.0f;
    while (a_deg < -180.0f) a_deg += 360.0f;
    return a_deg;
}

// Step 1: compute runtime offset from calibration pose
void joint_zero_calibration_capture(void)
{
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const float ref_true = g_joint_zero_table[i].ref_true_deg;
        const float enc_ref  = g_joint_zero_table[i].encoder_ref_deg; // replace with real encoder read
        g_joint_zero_table[i].offset_deg = enc_ref - ref_true;
    }
}

// Step 2: convert raw encoder angle to controller angle each cycle
float joint_angle_true_deg(joint_id_t joint, float encoder_raw_deg)
{
    const float offset = g_joint_zero_table[joint].offset_deg;
    return wrap_pm180(encoder_raw_deg - offset);
}

// Step 3: build state for controller (FK/IK/LQR/VMC uses this corrected angle)
void update_joint_state_from_encoder(const float enc_raw_deg[JOINT_COUNT], float q_true_deg[JOINT_COUNT])
{
    for (int i = 0; i < JOINT_COUNT; ++i) {
        q_true_deg[i] = joint_angle_true_deg((joint_id_t)i, enc_raw_deg[i]);
    }
}
