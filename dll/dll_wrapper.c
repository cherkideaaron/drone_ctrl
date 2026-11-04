#include <stdlib.h>
#include "../core/controller.h"

#if defined(_WIN32)
  #define DLL_API __declspec(dllexport)
#else
  #define DLL_API __attribute__((visibility("default")))
#endif

DLL_API Controller *ctrl_dll_create(void)
{
    Controller *c = (Controller *)malloc(sizeof(Controller));
    Params p;
    if (c) {
        ctrl_default_params(&p);
        ctrl_init(c, &p);
    }
    return c;
}

DLL_API void ctrl_dll_destroy(Controller *c)
{
    free(c);
}

DLL_API void ctrl_dll_configure(Controller *c, const float *cfg)
{
    Params p;
    ctrl_default_params(&p);
    p.hover_height_m    = cfg[0];
    p.climb_speed_mps   = cfg[1];
    p.hover_time_s      = cfg[2];
    p.descend_speed_mps = cfg[3];
    p.diag_horiz_m      = cfg[4];
    p.diag_angle_deg    = cfg[5];
    p.diag_speed_mps    = cfg[6];
    p.far_hold_s        = cfg[7];
    ctrl_init(c, &p);
}

DLL_API void ctrl_dll_start(Controller *c)
{
    ctrl_request_start(c);
}

DLL_API int ctrl_dll_step(Controller *c, const float *sens, float dt,
                          float *motors_out, float *status_out)
{
    Sensors s;
    Motors m;
    int i;

    s.x_m = sens[0];  s.y_m = sens[1];  s.height_m = sens[2];
    s.vx_mps = sens[3];  s.vy_mps = sens[4];  s.climb_rate_mps = sens[5];
    s.roll_rad = sens[6];  s.pitch_rad = sens[7];
    s.gyro_x_rps = sens[8];  s.gyro_y_rps = sens[9];

    ctrl_step(c, &s, dt, &m);

    for (i = 0; i < NUM_MOTORS; i++) motors_out[i] = m.motor[i];
    status_out[0] = c->sp_pos[0];
    status_out[1] = c->sp_pos[1];
    status_out[2] = c->sp_pos[2];
    status_out[3] = c->roll_des;
    status_out[4] = c->pitch_des;
    status_out[5] = c->thrust_cmd;
    status_out[6] = c->mode_time_s;
    status_out[7] = 0.0f;
    return (int)c->mode;
}

DLL_API void ctrl_dll_mixer(float thrust, float roll, float pitch, float yaw, float *motors_out)
{
    float torque[3];
    Motors m;
    int i;
    torque[0] = roll; torque[1] = pitch; torque[2] = yaw;
    mixer_update(thrust, torque, &m);
    for (i = 0; i < NUM_MOTORS; i++) motors_out[i] = m.motor[i];
}
