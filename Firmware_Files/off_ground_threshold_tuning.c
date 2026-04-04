#include <stdbool.h>

typedef struct {
    float enter_off_ground_th;
    float exit_off_ground_th;
    int debounce_on;
    int debounce_off;
    int counter;
    bool off_ground;
} off_ground_cfg_t;

bool off_ground_update(off_ground_cfg_t *c, float support_proxy)
{
    if (!c->off_ground) {
        if (support_proxy < c->enter_off_ground_th) c->counter++;
        else c->counter = 0;
        if (c->counter >= c->debounce_on) { c->off_ground = true; c->counter = 0; }
    } else {
        if (support_proxy > c->exit_off_ground_th) c->counter++;
        else c->counter = 0;
        if (c->counter >= c->debounce_off) { c->off_ground = false; c->counter = 0; }
    }
    return c->off_ground;
}
