#include <stdio.h>

typedef struct {
    float pitch_fw_deg;
    float gyro_y_dps;
    float left_rpm;
    float right_rpm;
    float phi1_deg;
    float phi2_deg;
    float g_x_mm;
    float g_y_mm;
} live_expr_t;

void print_tuning_live_expression(const live_expr_t *x)
{
    printf("[LiveExpr] pitch=%.3f deg, gyroY=%.3f dps\n", x->pitch_fw_deg, x->gyro_y_dps);
    printf("[LiveExpr] wheel rpm L/R = %.2f / %.2f\n", x->left_rpm, x->right_rpm);
    printf("[LiveExpr] joint phi1/phi2 = %.3f / %.3f deg\n", x->phi1_deg, x->phi2_deg);
    printf("[LiveExpr] FK point G = (%.2f, %.2f) mm\n", x->g_x_mm, x->g_y_mm);
}
