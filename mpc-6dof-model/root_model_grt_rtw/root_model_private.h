/*
 * root_model_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "root_model".
 *
 * Model version              : 1.99
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Thu Nov 12 13:53:48 2026
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

extern const real_T rtCP_pooled_OAywf2uaAdex[17];
extern const real_T rtCP_pooled_diAhBWWNVPNH[13];
extern const real_T rtCP_pooled_cEv1lNBE48f3[9];
extern const real_T rtCP_pooled_SJBgJpKqUfJl[289];
extern const real_T rtCP_pooled_0yWDPq4lclTh[169];
extern const real_T rtCP_pooled_vybzbDfrJJXS[289];

#define rtCP_Constant14_Value          rtCP_pooled_OAywf2uaAdex  /* Expression: [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                                                                  * Referenced by: '<Root>/Constant14'
                                                                  */
#define rtCP_Constant2_Value           rtCP_pooled_diAhBWWNVPNH  /* Expression: w_ov
                                                                  * Referenced by: '<Root>/Constant2'
                                                                  */
#define rtCP_Constant7_Value           rtCP_pooled_cEv1lNBE48f3  /* Expression: inertia_tensor
                                                                  * Referenced by: '<Root>/Constant7'
                                                                  */
#define rtCP_Q_Value                   rtCP_pooled_SJBgJpKqUfJl  /* Expression: p.Q
                                                                  * Referenced by: '<S3>/Q'
                                                                  */
#define rtCP_R1_Value                  rtCP_pooled_0yWDPq4lclTh  /* Expression: p.R{1}
                                                                  * Referenced by: '<S3>/R1'
                                                                  */
#define rtCP_DataStoreMemoryP_InitialVa rtCP_pooled_vybzbDfrJJXS /* Expression: p.InitialCovariance
                                                                  * Referenced by: '<S3>/DataStoreMemory - P'
                                                                  */

/* private model entry point functions */
extern void root_model_derivatives();

#endif                                 /* root_model_private_h_ */
