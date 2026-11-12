/*
 * Code generation for system system '<Root>/Simulink Function'
 *
 * Model                      : root_model
 * Model version              : 1.97
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Wed Nov 11 21:42:07 2026
 *
 * Note that the functions contained in this file are part of a Simulink
 * model, and are not self-contained algorithms.
 */

#include "rtwtypes.h"
#include "ekf_t_func.h"
#include <cstring>
#include "ekf_t_func_private.h"
#include <emmintrin.h>
#include "root_model.h"
#include "sind_CjVy6Jw4.h"
#include "cosd_ZK11uGDT.h"
#include <cmath>

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
void root_model_vehicle_state_func(const real_T x[17], const real_T u[3], real_T
  kP, real_T body_mass, real_T gravity_accel, const real_T inertia_tensor[9],
  real_T lever_arm, real_T x_dot[17])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  __m128d tmp_8;
  __m128d tmp_9;
  real_T c[9];
  real_T c_0[9];
  real_T n_0[9];
  real_T B[3];
  real_T omega[3];
  real_T tmp[2];
  real_T a21;
  real_T accel_idx_2;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T b_idx_2;
  real_T b_tmp;
  real_T b_x_b;
  real_T b_x_c;
  real_T b_x_d;
  real_T body_thrust_idx_0;
  real_T body_thrust_idx_1;
  real_T c_1;
  real_T maxval;
  real_T n;
  real_T y;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  static const int8_T c_2[3]{ 1, 0, 0 };

  static const int8_T d[3]{ 0, 1, 0 };

  static const int8_T c_b[3]{ 0, 0, 1 };

  omega[0] = x[10];
  omega[1] = x[11];
  omega[2] = x[12];
  maxval = u[0] * u[0] * kP;
  a21 = 57.295779513082323 * u[1];
  accel_idx_2 = a21;
  sind_CjVy6Jw4(&accel_idx_2);
  cosd_ZK11uGDT(&a21);
  n = 57.295779513082323 * u[2];
  b_tmp = n;
  sind_CjVy6Jw4(&b_tmp);
  cosd_ZK11uGDT(&n);
  c[1] = 0.0;
  c[4] = a21;
  c[7] = -accel_idx_2;
  c[2] = 0.0;
  c[5] = accel_idx_2;
  c[8] = a21;
  n_0[0] = n;
  n_0[3] = 0.0;
  n_0[6] = b_tmp;
  n_0[2] = -b_tmp;
  n_0[5] = 0.0;
  n_0[8] = n;
  for (r1 = 0; r1 < 3; r1++) {
    c[3 * r1] = c_2[r1];
    r2 = 3 * r1 + 1;
    n_0[r2] = d[r1];
    c_0[3 * r1] = 0.0;
    c_0[r2] = 0.0;
    c_0[3 * r1 + 2] = 0.0;
  }

  for (r1 = 0; r1 < 3; r1++) {
    c_1 = c_0[3 * r1];
    r2 = 3 * r1 + 1;
    body_thrust_idx_1 = c_0[r2];
    r3 = 3 * r1 + 2;
    b_tmp = c_0[r3];
    for (rtemp = 0; rtemp < 3; rtemp++) {
      body_thrust_idx_0 = n_0[3 * r1 + rtemp];
      tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&c[3 * rtemp]), _mm_set1_pd
        (body_thrust_idx_0)), _mm_set_pd(body_thrust_idx_1, c_1));
      _mm_storeu_pd(&tmp[0], tmp_0);
      c_1 = tmp[0];
      body_thrust_idx_1 = tmp[1];
      b_tmp += c[3 * rtemp + 2] * body_thrust_idx_0;
    }

    c_0[r3] = b_tmp;
    c_0[r2] = body_thrust_idx_1;
    c_0[3 * r1] = c_1;
  }

  n = std::sqrt(((x[6] * x[6] + x[7] * x[7]) + x[8] * x[8]) + x[9] * x[9]);
  _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_loadu_pd(&x[7]), _mm_set1_pd(n)));
  a21 = tmp[0];
  accel_idx_2 = tmp[1];
  y = 1.0 / body_mass;
  b_idx_0 = 0.0;
  b_idx_1 = 0.0;
  b_idx_2 = 0.0;
  c_1 = 0.0;
  body_thrust_idx_1 = 0.0;
  b_tmp = 0.0;
  for (r1 = 0; r1 < 3; r1++) {
    body_thrust_idx_0 = c_b[r1];
    tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&c_0[3 * r1]), _mm_set1_pd
      (body_thrust_idx_0)), _mm_set_pd(body_thrust_idx_1, c_1));
    _mm_storeu_pd(&tmp[0], tmp_0);
    c_1 = tmp[0];
    body_thrust_idx_1 = tmp[1];
    _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(inertia_tensor[3 *
      r1], c_0[3 * r1 + 2]), _mm_set_pd(omega[r1], body_thrust_idx_0)),
      _mm_set_pd(b_idx_0, b_tmp)));
    b_tmp = tmp[0];
    b_idx_0 = tmp[1];
    _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&inertia_tensor[3 *
      r1 + 1]), _mm_set1_pd(omega[r1])), _mm_set_pd(b_idx_2, b_idx_1)));
    b_idx_1 = tmp[0];
    b_idx_2 = tmp[1];
  }

  body_thrust_idx_0 = maxval * c_1;
  body_thrust_idx_1 *= maxval;
  c_1 = maxval * b_tmp;
  maxval = x[6] / n;
  n = x[9] / n;
  b_tmp = ((maxval * 0.0 - a21 * body_thrust_idx_0) - accel_idx_2 *
           body_thrust_idx_1) - n * c_1;
  b_x_b = ((maxval * body_thrust_idx_0 + a21 * 0.0) + accel_idx_2 * c_1) - n *
    body_thrust_idx_1;
  b_x_c = ((maxval * body_thrust_idx_1 - a21 * c_1) + accel_idx_2 * 0.0) + n *
    body_thrust_idx_0;
  b_x_d = ((maxval * c_1 + a21 * body_thrust_idx_1) - accel_idx_2 *
           body_thrust_idx_0) + n * 0.0;
  tmp_0 = _mm_set_pd(1.0, -1.0);
  tmp_6 = _mm_set_pd(-1.0, 1.0);
  _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
    (_mm_mul_pd(_mm_set1_pd(b_tmp), _mm_set_pd(-accel_idx_2, -a21)), _mm_mul_pd
     (_mm_mul_pd(_mm_set1_pd(b_x_b), _mm_set_pd(-n, maxval)), tmp_6)),
    _mm_mul_pd(_mm_set1_pd(b_x_c), _mm_set_pd(maxval, -n))), _mm_mul_pd
    (_mm_mul_pd(_mm_set1_pd(b_x_d), _mm_set_pd(-a21, -accel_idx_2)), tmp_0)),
    _mm_set1_pd(y)), _mm_div_pd(_mm_loadu_pd(&x[13]), _mm_set1_pd(body_mass))));
  accel_idx_2 = ((((b_tmp * -n + b_x_b * -accel_idx_2) - b_x_c * -a21) + b_x_d *
                  maxval) * y - gravity_accel) + 0.0 / body_mass;
  std::memcpy(&c[0], &inertia_tensor[0], 9U * sizeof(real_T));
  _mm_storeu_pd(&B[0], _mm_sub_pd(_mm_add_pd(_mm_sub_pd(_mm_mul_pd(_mm_set_pd
    (body_thrust_idx_0, 0.0), _mm_set_pd(lever_arm, c_1)), _mm_mul_pd(_mm_set_pd
    (0.0, body_thrust_idx_1), _mm_set_pd(c_1, lever_arm))), _mm_loadu_pd(&x[15])),
    _mm_sub_pd(_mm_mul_pd(_mm_set_pd(b_idx_0, x[11]), _mm_set_pd(x[12], b_idx_2)),
               _mm_mul_pd(_mm_set_pd(x[10], b_idx_1), _mm_set_pd(b_idx_2, x[12])))));
  B[2] = (0.0 * body_thrust_idx_1 - body_thrust_idx_0 * 0.0) - (x[10] * b_idx_1
    - b_idx_0 * x[11]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(inertia_tensor[0]);
  a21 = std::abs(inertia_tensor[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(inertia_tensor[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  c[r2] = inertia_tensor[r2] / inertia_tensor[r1];
  c[r3] /= c[r1];
  c[r2 + 3] -= c[r1 + 3] * c[r2];
  c[r3 + 3] -= c[r1 + 3] * c[r3];
  c[r2 + 6] -= c[r1 + 6] * c[r2];
  c[r3 + 6] -= c[r1 + 6] * c[r3];
  if (std::abs(c[r3 + 3]) > std::abs(c[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  c[r3 + 3] /= c[r2 + 3];
  c[r3 + 6] -= c[r3 + 3] * c[r2 + 6];
  body_thrust_idx_1 = B[r2] - B[r1] * c[r2];
  maxval = ((B[r3] - B[r1] * c[r3]) - c[r3 + 3] * body_thrust_idx_1) / c[r3 + 6];
  body_thrust_idx_1 = (body_thrust_idx_1 - c[r2 + 6] * maxval) / c[r2 + 3];
  std::memset(&x_dot[0], 0, 17U * sizeof(real_T));
  x_dot[0] = x[3];
  x_dot[3] = tmp[0];
  x_dot[10] = ((B[r1] - c[r1 + 6] * maxval) - c[r1 + 3] * body_thrust_idx_1) /
    c[r1];
  x_dot[1] = x[4];
  x_dot[4] = tmp[1];
  x_dot[11] = body_thrust_idx_1;
  x_dot[2] = x[5];
  x_dot[5] = accel_idx_2;
  x_dot[12] = maxval;
  tmp_1 = _mm_set1_pd(x[6]);
  tmp_2 = _mm_set1_pd(x[7]);
  tmp_3 = _mm_set_pd(x[11], x[12]);
  tmp_4 = _mm_set1_pd(x[8]);
  tmp_5 = _mm_set_pd(x[10], 0.0);
  tmp_7 = _mm_set1_pd(x[9]);
  tmp_8 = _mm_set_pd(0.0, x[10]);
  tmp_9 = _mm_set1_pd(0.5);
  _mm_storeu_pd(&x_dot[6], _mm_mul_pd(_mm_sub_pd(_mm_add_pd(_mm_add_pd
    (_mm_mul_pd(tmp_1, tmp_5), _mm_mul_pd(_mm_mul_pd(tmp_2, tmp_8), tmp_0)),
    _mm_mul_pd(_mm_mul_pd(tmp_4, _mm_loadu_pd(&x[11])), tmp_0)), _mm_mul_pd
    (tmp_7, tmp_3)), tmp_9));
  _mm_storeu_pd(&x_dot[8], _mm_mul_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
    (_mm_mul_pd(tmp_1, _mm_loadu_pd(&x[11])), _mm_mul_pd(_mm_mul_pd(tmp_2, tmp_3),
    tmp_0)), _mm_mul_pd(_mm_mul_pd(tmp_4, tmp_5), tmp_6)), _mm_mul_pd(tmp_7,
    tmp_8)), tmp_9));
}

/* Output and update for Simulink Function: '<Root>/Simulink Function' */
void ekf_t_func(const real_T rtu_x[17], real_T rty_x_hat[17])
{
  /* local block i/o variables */
  real_T rtb_TmpLatchAtIn1Outport1[16];
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  real_T k_1[17];
  real_T k_2[17];
  real_T k_3[17];
  real_T k_4[17];
  real_T rtu_x_0[17];
  real_T rtu_x_1;
  int32_T i;

  /* SignalConversion generated from: '<S7>/In1' */
  std::memcpy(&rtb_TmpLatchAtIn1Outport1[0], &root_model_B.VectorConcatenate1[0],
              sizeof(real_T) << 4U);

  /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
   *  SignalConversion generated from: '<S7>/x'
   */
  root_model_vehicle_state_func(rtu_x, &rtb_TmpLatchAtIn1Outport1[0],
    rtb_TmpLatchAtIn1Outport1[3], rtb_TmpLatchAtIn1Outport1[4],
    rtb_TmpLatchAtIn1Outport1[5], &rtb_TmpLatchAtIn1Outport1[6],
    rtb_TmpLatchAtIn1Outport1[15], k_1);
  for (i = 0; i <= 14; i += 2) {
    tmp_2 = _mm_loadu_pd(&k_1[i]);
    tmp_3 = _mm_loadu_pd(&rtu_x[i]);
    _mm_storeu_pd(&rtu_x_0[i], _mm_add_pd(_mm_div_pd(tmp_2, _mm_set1_pd(2.0)),
      tmp_3));
  }

  for (i = 16; i < 17; i++) {
    rtu_x_0[i] = k_1[i] / 2.0 + rtu_x[i];
  }

  root_model_vehicle_state_func(rtu_x_0, &rtb_TmpLatchAtIn1Outport1[0],
    rtb_TmpLatchAtIn1Outport1[3], rtb_TmpLatchAtIn1Outport1[4],
    rtb_TmpLatchAtIn1Outport1[5], &rtb_TmpLatchAtIn1Outport1[6],
    rtb_TmpLatchAtIn1Outport1[15], k_2);
  for (i = 0; i <= 14; i += 2) {
    tmp_2 = _mm_loadu_pd(&k_2[i]);
    tmp_3 = _mm_loadu_pd(&rtu_x[i]);
    _mm_storeu_pd(&rtu_x_0[i], _mm_add_pd(_mm_div_pd(tmp_2, _mm_set1_pd(2.0)),
      tmp_3));
  }

  for (i = 16; i < 17; i++) {
    rtu_x_0[i] = k_2[i] / 2.0 + rtu_x[i];
  }

  root_model_vehicle_state_func(rtu_x_0, &rtb_TmpLatchAtIn1Outport1[0],
    rtb_TmpLatchAtIn1Outport1[3], rtb_TmpLatchAtIn1Outport1[4],
    rtb_TmpLatchAtIn1Outport1[5], &rtb_TmpLatchAtIn1Outport1[6],
    rtb_TmpLatchAtIn1Outport1[15], k_3);
  for (i = 0; i <= 14; i += 2) {
    tmp_2 = _mm_loadu_pd(&rtu_x[i]);
    tmp_3 = _mm_loadu_pd(&k_3[i]);
    _mm_storeu_pd(&rtu_x_0[i], _mm_add_pd(tmp_2, tmp_3));
  }

  for (i = 16; i < 17; i++) {
    rtu_x_0[i] = rtu_x[i] + k_3[i];
  }

  root_model_vehicle_state_func(rtu_x_0, &rtb_TmpLatchAtIn1Outport1[0],
    rtb_TmpLatchAtIn1Outport1[3], rtb_TmpLatchAtIn1Outport1[4],
    rtb_TmpLatchAtIn1Outport1[5], &rtb_TmpLatchAtIn1Outport1[6],
    rtb_TmpLatchAtIn1Outport1[15], k_4);
  for (i = 0; i <= 14; i += 2) {
    /* SignalConversion generated from: '<S7>/x_hat' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function'
     *  SignalConversion generated from: '<S7>/x'
     */
    tmp_2 = _mm_loadu_pd(&rtu_x[i]);
    _mm_storeu_pd(&rty_x_hat[i], tmp_2);

    /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
     *  SignalConversion generated from: '<S7>/x'
     *  SignalConversion generated from: '<S7>/x_hat'
     */
    tmp_3 = _mm_loadu_pd(&k_2[i]);
    tmp_4 = _mm_set1_pd(2.0);
    tmp = _mm_loadu_pd(&k_1[i]);
    tmp_0 = _mm_loadu_pd(&k_3[i]);
    tmp_1 = _mm_loadu_pd(&k_4[i]);
    _mm_storeu_pd(&k_1[i], _mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(tmp_4, tmp_3), tmp), _mm_mul_pd(tmp_4, tmp_0)),
      tmp_1), _mm_set1_pd(0.16666666666666666)), tmp_2));
  }

  for (i = 16; i < 17; i++) {
    /* SignalConversion generated from: '<S7>/x' */
    rtu_x_1 = rtu_x[i];

    /* SignalConversion generated from: '<S7>/x_hat' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function'
     *  SignalConversion generated from: '<S7>/x'
     */
    rty_x_hat[i] = rtu_x_1;

    /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
     *  SignalConversion generated from: '<S7>/x'
     */
    k_1[i] = (((2.0 * k_2[i] + k_1[i]) + 2.0 * k_3[i]) + k_4[i]) *
      0.16666666666666666 + rtu_x_1;
  }

  /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
   *  SignalConversion generated from: '<S7>/x_hat'
   */
  std::memcpy(&rty_x_hat[0], &k_1[0], 13U * sizeof(real_T));
}
