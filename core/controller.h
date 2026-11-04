#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 4

typedef enum {
    MODE_IDLE = 0,
    MODE_LIFT,
    MODE_HOVER,
    MODE_OUTBOUND,
    MODE_FAR_HOLD,
    MODE_RETURN,
    MODE_DESCEND,
    MODE_LANDED
} FlightMode;

typedef struct {

    float hover_height_m;
    float climb_speed_mps;
    float hover_time_s;
    float descend_speed_mps;
    float diag_horiz_m;
    float diag_angle_deg;
    float diag_heading_deg;
    float diag_speed_mps;
    float far_hold_s;

    float land_speed_mps;
    float land_slow_height_m;
    float max_accel_mps2;

    float hover_thrust;
    float kp_z;
    float kv_z;
    float ki_z;
    float i_limit;

    float kp_xy;
    float kv_xy;
    float max_tilt_deg;
    float max_hspeed_mps;

    float kp_att;
    float kp_rate;
    float ki_rate;
    float rate_i_limit;

    float arrive_tol_m;
    float arrive_speed_mps;
    float touchdown_height_m;
} Params;

typedef struct {
    float x_m, y_m, height_m;
    float vx_mps, vy_mps, climb_rate_mps;
    float roll_rad, pitch_rad;
    float gyro_x_rps, gyro_y_rps;
} Sensors;

typedef Sensors Estimate;

typedef struct {
    float motor[NUM_MOTORS];
} Motors;

typedef struct {
    Params     p;
    FlightMode mode;
    float      mode_time_s;
    int        start_requested;

    Estimate   est;

    float home[3], hover_pt[3], far_pt[3];

    float seg_p0[3], seg_dir[3], seg_len, seg_s, seg_v;
    float sp_pos[3];
    float sp_vel[3];
    float sp_acc[3];

    float integ;
    float rate_int[2];
    float roll_des, pitch_des;
    float thrust_cmd;
} Controller;

void ctrl_default_params(Params *p);
void ctrl_init(Controller *c, const Params *p);
void ctrl_request_start(Controller *c);
void ctrl_step(Controller *c, const Sensors *in, float dt, Motors *out);

void  sensors_process(const Sensors *raw, Sensors *out);
void  estimator_update(Controller *c, const Sensors *s, float dt);
void  mode_update(Controller *c, float dt);
void  setpoint_update(Controller *c, float dt);
void  position_control_xy(Controller *c);
float altitude_control(Controller *c, float dt);
void  attitude_control(Controller *c, float rate_sp[3]);
void  rate_control(Controller *c, const float rate_sp[3], float dt, float torque[3]);
void  mixer_update(float thrust, const float torque[3], Motors *out);

#ifdef __cplusplus
}
#endif
#endif
