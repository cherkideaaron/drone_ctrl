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

DLL_API void ctrl_dll_configure(Controller *c,
                                 float hover_height_m,
                                 float climb_speed_mps,
                                 float hover_time_s,
                                 float descend_speed_mps)
{
    Params p;
    ctrl_default_params(&p);
    p.hover_height_m    = hover_height_m;
    p.climb_speed_mps   = climb_speed_mps;
    p.hover_time_s      = hover_time_s;
    p.descend_speed_mps = descend_speed_mps;
    ctrl_init(c, &p);
}

DLL_API void ctrl_dll_start(Controller *c)
{
    ctrl_request_start(c);
}

DLL_API int ctrl_dll_step(Controller *c,
                           float height_m, float climb_rate_mps, float dt,
                           float *motors_out, float *status_out)
{
    Sensors s;
    Motors  m;
    int i;

    s.height_m       = height_m;
    s.climb_rate_mps = climb_rate_mps;
    ctrl_step(c, &s, dt, &m);

    for (i = 0; i < NUM_MOTORS; i++) motors_out[i] = m.motor[i];
    status_out[0] = c->sp_height_m;
    status_out[1] = c->sp_climb_mps;
    status_out[2] = c->thrust_cmd;
    status_out[3] = c->mode_time_s;
    return (int)c->mode;
}

DLL_API void ctrl_dll_mixer(float thrust, float roll, float pitch, float yaw,
                             float *motors_out)
{
    float torque[3];
    Motors m;
    int i;
    torque[0] = roll;
    torque[1] = pitch;
    torque[2] = yaw;
    mixer_update(thrust, torque, &m);
    for (i = 0; i < NUM_MOTORS; i++) motors_out[i] = m.motor[i];
}

DLL_API const char *ctrl_dll_mode_name(int mode)
{
    return ctrl_mode_name((FlightMode)mode);
}