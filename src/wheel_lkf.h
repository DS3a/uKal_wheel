#ifndef WHEEL_LKF_H
#define WHEEL_LKF_H

#include "ukal.h"
#include "ukal_type.h"
#include "ulapack.h"

// initialize
FilterError_t create_wheel_lkf(Filter_t *const filter);

// set the transition matrix based on dt
// use ukal_set_phi
void set_dt(Filter_t *const filter, double dt);

// prediction step
// call ukal_model_predict
FilterError_t wheel_lkf_predict(Filter_t *const filter);

// position measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_position(Filter_t *const filter, double position, double noise);

// velocity measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_velocity(Filter_t *const filter, double velocity, double noise);

// acceleration measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_acceleration(Filter_t *const filter, double acceleration, double noise);

// get the state
Matrix_t* get_state(Filter_t *const filter);

#endif
