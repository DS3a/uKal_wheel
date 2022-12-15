#include "wheel_lkf.h"

#define DT 0.02

void copy_matrix(Matrix_t *m, double m_to_copy[3][3]) {
  ulapack_init(m, 3, 3);
  for (Index_t row = 0; row < 3; row++) {
    for (Index_t col = 0; col < 3; col++) {
      ulapack_edit_entry(m, row, col, m_to_copy[row][col]);
    }
  }
}

// initialize
FilterError_t create_wheel_lkf(Filter_t *const filter) {
  uint8_t linear_filter_type=1, n_states=3, n_measurements=1;
  Matrix_t *Phi, *gamma, *x0, *Q, *P0, *H, *R;
  double phi[3][3] = {
    {1, DT, DT*Dt},
    {0, 1, DT},
    {0, 0, 1}
  };
  ulapack_init(Phi, 3, 3);
  copy_matrix(Phi, phi);

  ulapack_init(gamma, 3, 1);
  ulapack_init(x0, 3, 1); // initializes all to 0

  double q[3][3] = {
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1}
  };
  ulapack_init(Q, 3, 3);
  copy_matrix(Q, q);

  ulapack_init(P0, 3, 3);
  copy_matrix(P0, q);

  ulapack_init(H, 1, 3);
  // TODO declare Phi, gamma, x0, Q, P0, H and R
  ukal_filter_create(filter, linear_filter_type,
                     n_states, n_measurements,
                     Phi, gamma, x0, Q, P0, H, R);
}

// set the transition matrix based on dt
// use ukal_set_phi
void set_dt(Filter_t *const filter, double dt) {
  // x = x0 + v*dt + a*(dt*dt)/2;
  // v = v0 + a*dt
  // need to put this into matrix form
}

// prediction step
// call ukal_model_predict
FilterError_t wheel_lkf_predict(Filter_t *const filter) {
  return ukal_model_predict(filter);
}

// position measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_position(Filter_t *const filter, double position, double noise) {
  Matrix_t *H, *R, *measurement;
  ulapack_init(measurement, 1, 1);
  ulapack_edit_entry(measurement, 0, 0, position);

  ulapack_init(R, 1, 1);
  ulapack_edit_entry(R, 0, 0, noise);

  ulapack_init(H, 1, 3);
  ulapack_edit_entry(H, 0, 0, 1);
  ulapack_edit_entry(H, 0, 1, 0);
  ulapack_edit_entry(H, 0, 2, 0);

  ukal_set_obs(filter, H);
  ukal_set_obs_noise(filter, R);

  return ukal_update(filter, measurement);
}

// velocity measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_velocity(Filter_t *const filter, double velocity, double noise) {
  Matrix_t *H, *R, *measurement;
  ulapack_init(measurement, 1, 1);
  ulapack_edit_entry(measurement, 0, 0, velocity);

  ulapack_init(R, 1, 1);
  ulapack_edit_entry(R, 0, 0, noise);

  ulapack_init(H, 1, 3);
  ulapack_edit_entry(H, 0, 0, 0);
  ulapack_edit_entry(H, 0, 1, 1);
  ulapack_edit_entry(H, 0, 2, 0);

  ukal_set_obs(filter, H);
  ukal_set_obs_noise(filter, R);

  return ukal_update(filter, measurement);
}

// acceleration measurement step
// use ukal_set_obs to change H, and set ukal_set_obs_nosise to set R
// then use ukal_update
FilterError_t measure_acceleration(Filter_t *const filter, double acceleration, double noise) {
  Matrix_t *H, *R, *measurement;
  ulapack_init(measurement, 1, 1);
  ulapack_edit_entry(measurement, 0, 0, acceleration);

  ulapack_init(R, 1, 1);
  ulapack_edit_entry(R, 0, 0, noise);

  ulapack_init(H, 1, 3);
  ulapack_edit_entry(H, 0, 0, 0);
  ulapack_edit_entry(H, 0, 1, 0);
  ulapack_edit_entry(H, 0, 2, 1);

  ukal_set_obs(filter, H);
  ukal_set_obs_noise(filter, R);

  return ukal_update(filter, measurement);
}

// get the state
Matrix_t* get_state(Filter_t *const filter);
