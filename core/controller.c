#include "controller.h"
#include <math.h>

#define G_MPS2  9.81f
#define DEG2RAD 0.017453293f

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void enter_mode(Controller *c, FlightMode m)
{
    c->mode = m;
    c->mode_time_s = 0.0f;
}

static void vcopy(float *dst, const float *src)
{
    dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
}

static void seg_begin(Controller *c, const float p1[3])
{
    float d[3], len;
    vcopy(c->seg_p0, c->sp_pos);
    d[0] = p1[0] - c->seg_p0[0];
    d[1] = p1[1] - c->seg_p0[1];
    d[2] = p1[2] - c->seg_p0[2];
    len = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (len < 1e-4f) {
        c->seg_dir[0] = c->seg_dir[1] = c->seg_dir[2] = 0.0f;
        c->seg_len = 0.0f;
    } else {
        c->seg_dir[0] = d[0] / len;
        c->seg_dir[1] = d[1] / len;
        c->seg_dir[2] = d[2] / len;
        c->seg_len = len;
    }
    c->seg_s = 0.0f;
    c->seg_v = 0.0f;
}

void ctrl_default_params(Params *p)
{
    p->hover_height_m     = 2.0f;
    p->climb_speed_mps    = 1.0f;
    p->hover_time_s       = 5.0f;
    p->descend_speed_mps  = 0.7f;
    p->diag_horiz_m       = 2.0f;
    p->diag_angle_deg     = 45.0f;
    p->diag_heading_deg   = 0.0f;
    p->diag_speed_mps     = 1.0f;
    p->far_hold_s         = 1.5f;

    p->land_speed_mps     = 0.25f;
    p->land_slow_height_m = 0.6f;
    p->max_accel_mps2     = 1.5f;

    p->hover_thrust       = 0.50f;
    p->kp_z               = 3.0f;
    p->kv_z               = 0.40f;
    p->ki_z               = 0.60f;
    p->i_limit            = 0.25f;

    p->kp_xy              = 1.5f;
    p->kv_xy              = 5.0f;
    p->max_tilt_deg       = 25.0f;
    p->max_hspeed_mps     = 3.0f;

    p->kp_att             = 8.0f;
    p->kp_rate            = 0.12f;
    p->ki_rate            = 0.10f;
    p->rate_i_limit       = 0.10f;

    p->arrive_tol_m       = 0.06f;
    p->arrive_speed_mps   = 0.20f;
    p->touchdown_height_m = 0.02f;
}

void ctrl_init(Controller *c, const Params *p)
{
    int i;
    c->p = *p;
    c->mode = MODE_IDLE;
    c->mode_time_s = 0.0f;
    c->start_requested = 0;
    c->est.x_m = c->est.y_m = c->est.height_m = 0.0f;
    c->est.vx_mps = c->est.vy_mps = c->est.climb_rate_mps = 0.0f;
    c->est.roll_rad = c->est.pitch_rad = 0.0f;
    c->est.gyro_x_rps = c->est.gyro_y_rps = 0.0f;
    for (i = 0; i < 3; i++) {
        c->home[i] = c->hover_pt[i] = c->far_pt[i] = 0.0f;
        c->seg_p0[i] = c->seg_dir[i] = 0.0f;
        c->sp_pos[i] = c->sp_vel[i] = c->sp_acc[i] = 0.0f;
    }
    c->seg_len = c->seg_s = c->seg_v = 0.0f;
    c->integ = 0.0f;
    c->rate_int[0] = c->rate_int[1] = 0.0f;
    c->roll_des = c->pitch_des = 0.0f;
    c->thrust_cmd = 0.0f;
}

void ctrl_request_start(Controller *c)
{
    c->start_requested = 1;
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
    position_control_xy(c);
    thrust = altitude_control(c, dt);
    attitude_control(c, rate_sp);
    rate_control(c, rate_sp, dt, torque);
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
    c->est = *s;
}

static int segment_arrived(const Controller *c)
{
    const float ex = c->sp_pos[0] - c->est.x_m;
    const float ey = c->sp_pos[1] - c->est.y_m;
    const float ez = c->sp_pos[2] - c->est.height_m;
    const float spd = sqrtf(c->est.vx_mps * c->est.vx_mps + c->est.vy_mps * c->est.vy_mps +
                            c->est.climb_rate_mps * c->est.climb_rate_mps);
    return (c->seg_s >= c->seg_len - 1e-3f) &&
           (sqrtf(ex * ex + ey * ey + ez * ez) < c->p.arrive_tol_m) &&
           (spd < c->p.arrive_speed_mps);
}

static void begin_descent(Controller *c)
{
    float low[3];
    low[0] = c->hover_pt[0];
    low[1] = c->hover_pt[1];
    low[2] = c->home[2] - 0.30f;
    seg_begin(c, low);
    enter_mode(c, MODE_DESCEND);
}

void mode_update(Controller *c, float dt)
{
    c->mode_time_s += dt;

    switch (c->mode) {
    case MODE_IDLE:
    case MODE_LANDED:
        if (c->start_requested) {
            const float h  = c->p.diag_heading_deg * DEG2RAD;
            const float run = c->p.diag_horiz_m;
            c->start_requested = 0;

            c->home[0] = c->est.x_m;  c->home[1] = c->est.y_m;  c->home[2] = c->est.height_m;
            c->hover_pt[0] = c->home[0];
            c->hover_pt[1] = c->home[1];
            c->hover_pt[2] = c->home[2] + c->p.hover_height_m;

            c->far_pt[0] = c->hover_pt[0] + run * cosf(h);
            c->far_pt[1] = c->hover_pt[1] + run * sinf(h);
            c->far_pt[2] = c->hover_pt[2] + run * tanf(c->p.diag_angle_deg * DEG2RAD);

            vcopy(c->sp_pos, c->home);
            c->sp_vel[0] = c->sp_vel[1] = c->sp_vel[2] = 0.0f;
            c->sp_acc[0] = c->sp_acc[1] = c->sp_acc[2] = 0.0f;
            c->integ = 0.0f;
            c->rate_int[0] = c->rate_int[1] = 0.0f;
            seg_begin(c, c->hover_pt);
            enter_mode(c, MODE_LIFT);
        }
        break;

    case MODE_LIFT:
        if (segment_arrived(c)) enter_mode(c, MODE_HOVER);
        break;

    case MODE_HOVER:
        if (c->mode_time_s >= c->p.hover_time_s) {
            if (c->p.diag_horiz_m > 1e-3f) {
                seg_begin(c, c->far_pt);
                enter_mode(c, MODE_OUTBOUND);
            } else {
                begin_descent(c);
            }
        }
        break;

    case MODE_OUTBOUND:
        if (segment_arrived(c)) enter_mode(c, MODE_FAR_HOLD);
        break;

    case MODE_FAR_HOLD:
        if (c->mode_time_s >= c->p.far_hold_s) {
            seg_begin(c, c->hover_pt);
            enter_mode(c, MODE_RETURN);
        }
        break;

    case MODE_RETURN:
        if (segment_arrived(c)) begin_descent(c);
        break;

    case MODE_DESCEND:
        if (c->est.height_m - c->home[2] < c->p.touchdown_height_m) {
            enter_mode(c, MODE_LANDED);
        }
        break;
    }
}

void setpoint_update(Controller *c, float dt)
{
    float vmax, remaining, vdes, dv, maxdv, v_old;
    const float a = c->p.max_accel_mps2;

    switch (c->mode) {
    case MODE_LIFT:
        vmax = c->p.climb_speed_mps;
        break;
    case MODE_OUTBOUND:
    case MODE_RETURN:
        vmax = c->p.diag_speed_mps;
        break;
    case MODE_DESCEND:
        vmax = (c->sp_pos[2] - c->home[2] > c->p.land_slow_height_m)
                   ? c->p.descend_speed_mps : c->p.land_speed_mps;
        break;
    case MODE_HOVER:
    case MODE_FAR_HOLD:
        c->sp_vel[0] = c->sp_vel[1] = c->sp_vel[2] = 0.0f;
        c->sp_acc[0] = c->sp_acc[1] = c->sp_acc[2] = 0.0f;
        return;
    default:
        c->sp_pos[0] = c->est.x_m;
        c->sp_pos[1] = c->est.y_m;
        c->sp_pos[2] = c->est.height_m;
        c->sp_vel[0] = c->sp_vel[1] = c->sp_vel[2] = 0.0f;
        c->sp_acc[0] = c->sp_acc[1] = c->sp_acc[2] = 0.0f;
        return;
    }

    v_old = c->seg_v;
    remaining = c->seg_len - c->seg_s;
    if (remaining < 1e-4f && fabsf(c->seg_v) < 1e-3f) {
        c->seg_s = c->seg_len;
        c->seg_v = 0.0f;
    } else {

        vdes = sqrtf(2.0f * a * (remaining > 0.0f ? remaining : 0.0f));
        if (vdes > vmax) vdes = vmax;
        dv    = vdes - c->seg_v;
        maxdv = a * dt;
        c->seg_v += clampf(dv, -maxdv, maxdv);
        c->seg_s += c->seg_v * dt;
        if (c->seg_s > c->seg_len) c->seg_s = c->seg_len;
    }

    c->sp_pos[0] = c->seg_p0[0] + c->seg_dir[0] * c->seg_s;
    c->sp_pos[1] = c->seg_p0[1] + c->seg_dir[1] * c->seg_s;
    c->sp_pos[2] = c->seg_p0[2] + c->seg_dir[2] * c->seg_s;
    c->sp_vel[0] = c->seg_dir[0] * c->seg_v;
    c->sp_vel[1] = c->seg_dir[1] * c->seg_v;
    c->sp_vel[2] = c->seg_dir[2] * c->seg_v;
    {
        const float acc = (dt > 0.0f) ? (c->seg_v - v_old) / dt : 0.0f;
        c->sp_acc[0] = c->seg_dir[0] * acc;
        c->sp_acc[1] = c->seg_dir[1] * acc;
        c->sp_acc[2] = c->seg_dir[2] * acc;
    }
}

void position_control_xy(Controller *c)
{
    float vx_des, vy_des, ax, ay, amax, tilt_max;

    if (c->mode == MODE_IDLE || c->mode == MODE_LANDED) {
        c->roll_des = c->pitch_des = 0.0f;
        return;
    }

    vx_des = c->p.kp_xy * (c->sp_pos[0] - c->est.x_m) + c->sp_vel[0];
    vy_des = c->p.kp_xy * (c->sp_pos[1] - c->est.y_m) + c->sp_vel[1];
    vx_des = clampf(vx_des, -c->p.max_hspeed_mps, c->p.max_hspeed_mps);
    vy_des = clampf(vy_des, -c->p.max_hspeed_mps, c->p.max_hspeed_mps);

    ax = c->p.kv_xy * (vx_des - c->est.vx_mps) + c->sp_acc[0];
    ay = c->p.kv_xy * (vy_des - c->est.vy_mps) + c->sp_acc[1];

    tilt_max = c->p.max_tilt_deg * DEG2RAD;
    amax = G_MPS2 * tanf(tilt_max);
    ax = clampf(ax, -amax, amax);
    ay = clampf(ay, -amax, amax);

    c->pitch_des = clampf(-atan2f(ax, G_MPS2), -tilt_max, tilt_max);
    c->roll_des  = clampf(-atan2f(ay, G_MPS2), -tilt_max, tilt_max);
}

float altitude_control(Controller *c, float dt)
{
    float vz_des, verr, thrust, cos_tilt;

    if (c->mode == MODE_IDLE) {
        c->integ = 0.0f;
        return 0.0f;
    }
    if (c->mode == MODE_LANDED) {

        c->integ = 0.0f;
        thrust = c->thrust_cmd * (1.0f - dt / 0.25f);
        return clampf(thrust, 0.0f, 1.0f);
    }

    vz_des = c->p.kp_z * (c->sp_pos[2] - c->est.height_m) + c->sp_vel[2];
    vz_des = clampf(vz_des, -3.0f, 3.0f);
    verr = vz_des - c->est.climb_rate_mps;

    c->integ += c->p.ki_z * verr * dt;
    c->integ  = clampf(c->integ, -c->p.i_limit, c->p.i_limit);

    thrust = c->p.hover_thrust * (1.0f + c->sp_acc[2] / G_MPS2)
             + c->p.kv_z * verr + c->integ;

    cos_tilt = cosf(c->est.roll_rad) * cosf(c->est.pitch_rad);
    thrust  /= clampf(cos_tilt, 0.7f, 1.0f);
    return clampf(thrust, 0.0f, 1.0f);
}

void attitude_control(Controller *c, float rate_sp[3])
{
    rate_sp[0] = clampf(c->p.kp_att * (c->roll_des  - c->est.roll_rad),  -4.0f, 4.0f);
    rate_sp[1] = clampf(c->p.kp_att * (c->pitch_des - c->est.pitch_rad), -4.0f, 4.0f);
    rate_sp[2] = 0.0f;
}

void rate_control(Controller *c, const float rate_sp[3], float dt, float torque[3])
{
    float err_r, err_p;

    if (c->mode == MODE_IDLE || c->mode == MODE_LANDED) {
        c->rate_int[0] = c->rate_int[1] = 0.0f;
        torque[0] = torque[1] = torque[2] = 0.0f;
        return;
    }

    err_r = rate_sp[0] - c->est.gyro_x_rps;
    err_p = rate_sp[1] - c->est.gyro_y_rps;

    c->rate_int[0] = clampf(c->rate_int[0] + c->p.ki_rate * err_r * dt,
                            -c->p.rate_i_limit, c->p.rate_i_limit);
    c->rate_int[1] = clampf(c->rate_int[1] + c->p.ki_rate * err_p * dt,
                            -c->p.rate_i_limit, c->p.rate_i_limit);

    torque[0] = c->p.kp_rate * err_r + c->rate_int[0];
    torque[1] = c->p.kp_rate * err_p + c->rate_int[1];
    torque[2] = 0.0f;
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
