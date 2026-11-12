/*
 * Code generation for system system '<Root>/Simulink Function'
 * For more details, see corresponding source file ekf_t_func.c
 *
 */

#ifndef ekf_t_func_private_h_
#define ekf_t_func_private_h_
#include <cmath>
#include <stdio.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#ifdef __cplusplus

extern "C"
{

#endif

  /* Exported functions */
  extern void root_model_vehicle_state_func(const real_T x[17], const real_T u[3],
    real_T kP, real_T body_mass, real_T gravity_accel, const real_T
    inertia_tensor[9], real_T lever_arm, real_T x_dot[17]);

#ifdef __cplusplus

}

#endif
#endif                                 /* ekf_t_func_private_h_ */
