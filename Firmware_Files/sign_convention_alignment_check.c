#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    float pitch_fw_deg;     // imu_heading.pit in firmware convention
    float gyro_y_dps;       // IMU y-axis angular rate
    float v_left_mps;
    float v_right_mps;
} live_expr_t;

// Example per-joint sign map for mirrored assembly
static const int8_t joint_sign[4] = { +1, +1, -1, -1 };
static float joint_offset_deg[4] = { 12.40f, -8.15f, 11.90f, -7.80f };

// Model uses +phi for forward tilt; firmware uses negative pitch for forward tilt.
static inline float phi_model_from_fw(float pitch_fw_deg)
{
    return -pitch_fw_deg;
}

// Same mapping for pitch rate axis in this project integration.
static inline float dphi_model_from_fw(float gyro_y_dps)
{
    return -gyro_y_dps;
}

static inline float joint_true_from_encoder(float enc_deg, int idx)
{
    return joint_sign[idx] * (enc_deg - joint_offset_deg[idx]);
}

void sign_alignment_live_check(const live_expr_t *x)
{
    float phi_model = phi_model_from_fw(x->pitch_fw_deg);
    float dphi_model = dphi_model_from_fw(x->gyro_y_dps);

    // Example: corrected average forward speed channel
    float xdot_model = 0.5f * (x->v_left_mps + x->v_right_mps);

    printf("[LiveExpr] pitch_fw=%.3f deg -> phi_model=%.3f deg\n", x->pitch_fw_deg, phi_model);
    printf("[LiveExpr] gyro_y=%.3f dps -> dphi_model=%.3f dps\n", x->gyro_y_dps, dphi_model);
    printf("[LiveExpr] vL=%.3f, vR=%.3f -> xdot_model=%.3f m/s\n",
           x->v_left_mps, x->v_right_mps, xdot_model);
}

void lqr_sign_conversion(float K_model[2][6], float K_firmware[2][6])
{
    // Copy all columns first
    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 6; ++c) {
            K_firmware[r][c] = K_model[r][c];
        }
    }

    // Flip pitch-state columns: phi, dphi
    K_firmware[0][4] = -K_model[0][4];
    K_firmware[0][5] = -K_model[0][5];
    K_firmware[1][4] = -K_model[1][4];
    K_firmware[1][5] = -K_model[1][5];
}
