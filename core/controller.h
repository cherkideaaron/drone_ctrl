#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 4

typedef enum {
    MODE_IDLE    = 0,
    MODE_LIFT,
    MODE_HOVER,
    MODE_DESCEND,
    MODE_LANDED
} FlightMode;

typedef struct {
    float hover_height_m;
    float climb_speed_mps;
    float hover_time_s;
    float descend_speed_mps;
    float land_speed_mps;
    float land_slow_height_m;
    float max_accel_mps2;
    float hover_thrust;
    float kp_z;
    float kv_z;
    float ki_z;
    float i_limit;
    float arrive_tol_m;
    float touchdown_height_m;
} Params;

typedef struct {
    float height_m;
    float climb_rate_mps;
} Sensors;

typedef struct {
    float height_m;
    float climb_rate_mps;
} Estimate;

typedef struct {
    float motor[NUM_MOTORS];
} Motors;

typedef struct {
    Params     p;
    FlightMode mode;
    float      mode_time_s;
    int        start_requested;
    Estimate   est;
    float      sp_height_m;
    float      sp_climb_mps;
    float      integ;
    float      thrust_cmd;
} Controller;

void ctrl_default_params(Params *p);
void ctrl_init(Controller *c, const Params *p);
void ctrl_request_start(Controller *c);
void ctrl_step(Controller *c, const Sensors *in, float dt, Motors *out);
const char *ctrl_mode_name(FlightMode m);

void  sensors_process(const Sensors *raw, Sensors *out);
void  estimator_update(Controller *c, const Sensors *s, float dt);
void  mode_update(Controller *c, float dt);
void  setpoint_update(Controller *c, float dt);
float altitude_control(Controller *c, float dt);
void  attitude_control(Controller *c, float rate_sp[3]);
void  rate_control(Controller *c, const float rate_sp[3], float torque[3]);
void  mixer_update(float thrust, const float torque[3], Motors *out);

#ifdef __cplusplus
}
#endif
#endif