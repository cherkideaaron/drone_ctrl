#include "controller.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void enter_mode(Controller *c, FlightMode m)
{
    c->mode = m;
    c->mode_time_s = 0.0f;
}

void ctrl_default_params(Params *p)
{
    p->hover_height_m     = 2.0f;
    p->climb_speed_mps    = 1.0f;
    p->hover_time_s       = 5.0f;
    p->descend_speed_mps  = 0.7f;
    p->land_speed_mps     = 0.25f;
    p->land_slow_height_m = 0.6f;
    p->max_accel_mps2     = 1.5f;
    p->hover_thrust       = 0.50f;
    p->kp_z               = 3.0f;
    p->kv_z               = 0.40f;
    p->ki_z               = 0.60f;
    p->i_limit            = 0.25f;
    p->arrive_tol_m       = 0.05f;
    p->touchdown_height_m = 0.02f;
}

void ctrl_init(Controller *c, const Params *p)
{
    c->p = *p;
    c->mode            = MODE_IDLE;
    c->mode_time_s     = 0.0f;
    c->start_requested = 0;
    c->est.height_m       = 0.0f;
    c->est.climb_rate_mps = 0.0f;
    c->sp_height_m  = 0.0f;
    c->sp_climb_mps = 0.0f;
    c->integ        = 0.0f;
    c->thrust_cmd   = 0.0f;
}

void ctrl_request_start(Controller *c)
{
    c->start_requested = 1;
}

const char *ctrl_mode_name(FlightMode m)
{
    switch (m) {
    case MODE_IDLE:    return "IDLE";
    case MODE_LIFT:    return "LIFT";
    case MODE_HOVER:   return "HOVER";
    case MODE_DESCEND: return "DESCEND";
    case MODE_LANDED:  return "LANDED";
    default:           return "UNKNOWN";
    }
}

void ctrl_step(Controller *c, const Sensors *in, float dt, Motors *out)
{
    Sensors s;
    float rate_sp[3] = {0.0f, 0.0f, 0.0f};
    float torque[3]  = {0.0f, 0.0f, 0.0f};
    float thrust;

    sensors_process(in, &s);
    estimator_update(c, &s, dt);
    mode_update(c, dt);
    setpoint_update(c, dt);
    thrust = altitude_control(c, dt);
    attitude_control(c, rate_sp);
    rate_control(c, rate_sp, torque);
    mixer_update(thrust, torque, out);
    c->thrust_cmd = thrust;
}

void sensors_process(const Sensors *raw, Sensors *out)
{
    *out = *raw;
}

void estimator_update(Controller *c, const Sensors *s, float dt)
{
    (void)dt;
    c->est.height_m       = s->height_m;
    c->est.climb_rate_mps = s->climb_rate_mps;
}

void mode_update(Controller *c, float dt)
{
    c->mode_time_s += dt;

    switch (c->mode) {
    case MODE_IDLE:
    case MODE_LANDED:
        if (c->start_requested) {
            c->start_requested = 0;
            c->sp_height_m  = c->est.height_m;
            c->sp_climb_mps = 0.0f;
            c->integ        = 0.0f;
            enter_mode(c, MODE_LIFT);
        }
        break;

    case MODE_LIFT:
        if (fabsf(c->sp_height_m - c->p.hover_height_m) < 1e-3f &&
            fabsf(c->est.height_m - c->p.hover_height_m) < c->p.arrive_tol_m &&
            fabsf(c->est.climb_rate_mps) < 0.2f) {
            enter_mode(c, MODE_HOVER);
        }
        break;

    case MODE_HOVER:
        if (c->mode_time_s >= c->p.hover_time_s) {
            enter_mode(c, MODE_DESCEND);
        }
        break;

    case MODE_DESCEND:
        if (c->est.height_m < c->p.touchdown_height_m) {
            enter_mode(c, MODE_LANDED);
        }
        break;
    }
}

void setpoint_update(Controller *c, float dt)
{
    float target, vmax, dist, vdes, dv, maxdv;
    const float a = c->p.max_accel_mps2;

    switch (c->mode) {
    case MODE_LIFT:
    case MODE_HOVER:
        target = c->p.hover_height_m;
        vmax   = c->p.climb_speed_mps;
        break;
    case MODE_DESCEND:
        target = -0.30f;
        vmax   = (c->sp_height_m > c->p.land_slow_height_m)
                     ? c->p.descend_speed_mps : c->p.land_speed_mps;
        break;
    default:
        c->sp_height_m  = c->est.height_m;
        c->sp_climb_mps = 0.0f;
        return;
    }

    dist = target - c->sp_height_m;
    if (fabsf(dist) < 1e-4f && fabsf(c->sp_climb_mps) < 1e-3f) {
        c->sp_height_m  = target;
        c->sp_climb_mps = 0.0f;
        return;
    }

    vdes = sqrtf(2.0f * a * fabsf(dist));
    if (vdes > vmax) vdes = vmax;
    if (dist < 0.0f) vdes = -vdes;

    dv    = vdes - c->sp_climb_mps;
    maxdv = a * dt;
    c->sp_climb_mps += clampf(dv, -maxdv, maxdv);
    c->sp_height_m  += c->sp_climb_mps * dt;
}

float altitude_control(Controller *c, float dt)
{
    float vz_des, verr, thrust;

    if (c->mode == MODE_IDLE) {
        c->integ = 0.0f;
        return 0.0f;
    }

    if (c->mode == MODE_LANDED) {
        c->integ = 0.0f;
        thrust = c->thrust_cmd * (1.0f - dt / 0.25f);
        return clampf(thrust, 0.0f, 1.0f);
    }

    vz_des = c->p.kp_z * (c->sp_height_m - c->est.height_m) + c->sp_climb_mps;
    vz_des = clampf(vz_des, -3.0f, 3.0f);

    verr = vz_des - c->est.climb_rate_mps;

    c->integ += c->p.ki_z * verr * dt;
    c->integ  = clampf(c->integ, -c->p.i_limit, c->p.i_limit);

    thrust = c->p.hover_thrust + c->p.kv_z * verr + c->integ;
    return clampf(thrust, 0.0f, 1.0f);
}

void attitude_control(Controller *c, float rate_sp[3])
{
    (void)c;
    rate_sp[0] = rate_sp[1] = rate_sp[2] = 0.0f;
}

void rate_control(Controller *c, const float rate_sp[3], float torque[3])
{
    (void)c; (void)rate_sp;
    torque[0] = torque[1] = torque[2] = 0.0f;
}

void mixer_update(float thrust, const float torque[3], Motors *out)
{
    const float r = torque[0], p = torque[1], y = torque[2];
    int i;

    out->motor[0] = thrust - r + p + y;
    out->motor[1] = thrust + r - p + y;
    out->motor[2] = thrust + r + p - y;
    out->motor[3] = thrust - r - p - y;

    for (i = 0; i < NUM_MOTORS; i++) {
        out->motor[i] = clampf(out->motor[i], 0.0f, 1.0f);
    }
}