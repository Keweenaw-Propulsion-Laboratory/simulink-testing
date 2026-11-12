/*
 * root_model_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "root_model".
 *
 * Model version              : 1.97
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Wed Nov 11 21:42:07 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef root_model_private_h_
#define root_model_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "root_model_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern const real_T rtCP_pooled_dncvkHsf0V0w[17];
extern const real_T rtCP_pooled_SILOYW4lbPMp[13];
extern const real_T rtCP_pooled_4Rb06x98nFwl[9];
extern const real_T rtCP_pooled_WtWM9lZWBlv3[289];
extern const real_T rtCP_pooled_14l6eM7rQ1QF[169];
extern const real_T rtCP_pooled_lNnM4YZ0eDCc[289];

#define rtCP_Constant14_Value          rtCP_pooled_dncvkHsf0V0w  /* Expression: [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                                                                  * Referenced by: '<Root>/Constant14'
                                                                  */
#define rtCP_Constant2_Value           rtCP_pooled_SILOYW4lbPMp  /* Expression: w_ov
                                                                  * Referenced by: '<Root>/Constant2'
                                                                  */
#define rtCP_Constant7_Value           rtCP_pooled_4Rb06x98nFwl  /* Expression: inertia_tensor
                                                                  * Referenced by: '<Root>/Constant7'
                                                                  */
#define rtCP_Q_Value                   rtCP_pooled_WtWM9lZWBlv3  /* Expression: p.Q
                                                                  * Referenced by: '<S3>/Q'
                                                                  */
#define rtCP_R1_Value                  rtCP_pooled_14l6eM7rQ1QF  /* Expression: p.R{1}
                                                                  * Referenced by: '<S3>/R1'
                                                                  */
#define rtCP_DataStoreMemoryP_InitialVa rtCP_pooled_lNnM4YZ0eDCc /* Expression: p.InitialCovariance
                                                                  * Referenced by: '<S3>/DataStoreMemory - P'
                                                                  */

/* private model entry point functions */
extern void root_model_derivatives(void);

#endif                                 /* root_model_private_h_ */
