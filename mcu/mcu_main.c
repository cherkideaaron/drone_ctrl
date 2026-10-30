#include "../core/controller.h"

#define DT_S  (0.005f)

static Controller g_ctrl;
static Sensors    g_sensors;
static Motors     g_motors;

static void hw_init(void)      { }
static void sensors_read(Sensors *s) { s->height_m = 0.0f; s->climb_rate_mps = 0.0f; }
static void escs_write(const Motors *m) { (void)m; }
static void uart_poll(void)    { }

int main(void)
{
    Params p;
    hw_init();
    ctrl_default_params(&p);
    ctrl_init(&g_ctrl, &p);

    while (1) {
        sensors_read(&g_sensors);
        ctrl_step(&g_ctrl, &g_sensors, DT_S, &g_motors);
        escs_write(&g_motors);
        uart_poll();
    }

    return 0;
}