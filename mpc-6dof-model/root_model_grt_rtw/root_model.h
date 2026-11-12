/*
 * root_model.h
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

#ifndef root_model_h_
#define root_model_h_
#include <cmath>
#include <stdio.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "root_model_types.h"
#include "ekf_t_func.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <cstring>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
struct B_root_model_T {
  s_7iu1rzNoAoHigERqeOFm2G_root_T TrialState;
  s_Jrk1gF3jitJmkPBgphEknE_root_T WorkingSet;
  s_Jrk1gF3jitJmkPBgphEknE_root_T b_obj;
  s_Jrk1gF3jitJmkPBgphEknE_root_T b_WorkingSet;
  s_Pct27jAhkqchx6iRs2zKpD_root_T QRManager;
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T memspace;
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T QPObjective;
  s_mdyENYu4oaiCttMAPwg9iE_root_T CholManager;
  real_T y_data[2640625];
  real_T B_data[1459250];
  real_T JacCineqTrans_data[46020];
  real_T b_varargin_1_data[46020];
  real_T a__4_data[46020];
  real_T Jx[44200];
  real_T Jx_data[44200];
  real_T varargin_1_data[44200];
  real_T unusedExpr[31329];
  real_T JacCeqTrans[30090];
  real_T JacEqTrans_tmp[30090];
  real_T Jx_m[28900];
  real_T A_data[24120];
  real_T tmp_data[7800];
  real_T Jmv[5100];
  coder_internal_stickyStruct_2_T FcnEvaluator;
  real_T y_data_c[1625];
  real_T y_data_k[1625];
  real_T work_data[1625];
  real_T work_data_c[1625];
  real_T vn1_data[1625];
  real_T vn2_data[1625];
  real_T work_data_b[1625];
  real_T b_data[1625];
  real_T y_data_p[1625];
  real_T y_data_cv[1625];
  real_T Auf_data[1560];
  real_T Jmv_data[1560];
  real_T tmp_data_f[1560];
  s_WBLIzjzqQS4bg0a8DMrwYD_root_T expl_temp;
  s_WBLIzjzqQS4bg0a8DMrwYD_root_T expl_temp_g;
  real_T Jmv_g[1020];
  real_T dv[1020];
  real_T M[578];
  real_T VectorConcatenate2[13];       /* '<Root>/Vector Concatenate2' */
  real_T VectorConcatenate1[16];       /* '<Root>/Vector Concatenate1' */
  real_T Add[3];                       /* '<Root>/Add' */
  real_T mv[3];                        /* '<S12>/NLMPC' */
  real_T prop_force[3];                /* '<Root>/MATLAB Function' */
  real_T prop_torque[3];               /* '<Root>/MATLAB Function' */
  boolean_T PWLCheck;                  /* '<S1>/PWLCheck' */
  boolean_T PWLCheck_m;                /* '<S2>/PWLCheck' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_root_model_T {
  real_T UnitDelay_DSTATE[3];          /* '<Root>/Unit Delay' */
  real_T P[289];                       /* '<S3>/DataStoreMemory - P' */
  real_T x[17];                        /* '<S3>/DataStoreMemory - x' */
  void *PWLCheck_PWORK;                /* '<S1>/PWLCheck' */
  void *PWLCheck_PWORK_p;              /* '<S2>/PWLCheck' */
  matlabshared_tracking_interna_T obj; /* '<S8>/MATLAB System' */
  matlabshared_tracking_inter_l_T obj_l;/* '<S10>/MATLAB System' */
  boolean_T objisempty;                /* '<S10>/MATLAB System' */
  boolean_T objisempty_d;              /* '<S8>/MATLAB System' */
};

/* Continuous states (default storage) */
struct X_root_model_T {
  real_T uDOFQuaternion_CSTATE[13];    /* '<Root>/6DOF (Quaternion)' */
};

/* State derivatives (default storage) */
struct XDot_root_model_T {
  real_T uDOFQuaternion_CSTATE[13];    /* '<Root>/6DOF (Quaternion)' */
};

/* State disabled  */
struct XDis_root_model_T {
  boolean_T uDOFQuaternion_CSTATE[13]; /* '<Root>/6DOF (Quaternion)' */
};

/* Invariant block signals (default storage) */
struct ConstB_root_model_T {
  real_T refrowvecinput[13];           /* '<Root>/Transpose' */
  real_T Reshape[9];                   /* '<Root>/Reshape' */
};

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
struct ODE3_IntgData {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
};

#endif

/* Real-time Model Data Structure */
struct tag_RTM_root_model_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_root_model_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_root_model_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[13];
  real_T odeF[3][13];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_root_model_T root_model_B;

#ifdef __cplusplus

}

#endif

/* Continuous states (default storage) */
extern X_root_model_T root_model_X;

/* Disabled states (default storage) */
extern XDis_root_model_T root_model_XDis;

/* Block states (default storage) */
extern struct DW_root_model_T root_model_DW;
extern const ConstB_root_model_T root_model_ConstB;/* constant block i/o */

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void root_model_initialize(void);
  extern void root_model_step(void);
  extern void root_model_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_root_model_T *const root_model_M;

#ifdef __cplusplus

}

#endif

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/checkMeasurementFcn1Signals' : Unused code path elimination
 * Block '<S3>/checkStateTransitionFcnSignals' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S18>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S25>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S26>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S27>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S28>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S29>/Vector Dimension Check' : Unused code path elimination
 * Block '<S30>/Vector Dimension Check' : Unused code path elimination
 * Block '<S31>/Vector Dimension Check' : Unused code path elimination
 * Block '<S32>/Vector Dimension Check' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope4' : Unused code path elimination
 * Block '<Root>/Scope5' : Unused code path elimination
 * Block '<S3>/DataTypeConversion_Enable1' : Eliminate redundant data type conversion
 * Block '<S3>/DataTypeConversion_Q' : Eliminate redundant data type conversion
 * Block '<S3>/DataTypeConversion_R1' : Eliminate redundant data type conversion
 * Block '<S3>/DataTypeConversion_uMeas1' : Eliminate redundant data type conversion
 * Block '<S3>/DataTypeConversion_uState' : Eliminate redundant data type conversion
 * Block '<S3>/DataTypeConversion_y1' : Eliminate redundant data type conversion
 * Block '<S12>/Reshape' : Reshape block reduction
 * Block '<S12>/Reshape1' : Reshape block reduction
 * Block '<S12>/mo or x Conversion' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion1' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion10' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion11' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion12' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion13' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion14' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion15' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion16' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion17' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion18' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion19' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion2' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion3' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion4' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion5' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion6' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion7' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion8' : Eliminate redundant data type conversion
 * Block '<S12>/mo or x Conversion9' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'root_model'
 * '<S1>'   : 'root_model/Check Custom Bounds'
 * '<S2>'   : 'root_model/Check Step Response  Characteristics2'
 * '<S3>'   : 'root_model/Extended Kalman Filter'
 * '<S4>'   : 'root_model/MATLAB Function'
 * '<S5>'   : 'root_model/Nonlinear MPC Controller'
 * '<S6>'   : 'root_model/Quaternion Rotation'
 * '<S7>'   : 'root_model/Simulink Function'
 * '<S8>'   : 'root_model/Extended Kalman Filter/Correct1'
 * '<S9>'   : 'root_model/Extended Kalman Filter/Output'
 * '<S10>'  : 'root_model/Extended Kalman Filter/Predict'
 * '<S11>'  : 'root_model/Extended Kalman Filter/Output/MATLAB Function'
 * '<S12>'  : 'root_model/Nonlinear MPC Controller/MPC'
 * '<S13>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check'
 * '<S14>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S15>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check10'
 * '<S16>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check11'
 * '<S17>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check12'
 * '<S18>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check13'
 * '<S19>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check14'
 * '<S20>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check15'
 * '<S21>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check16'
 * '<S22>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S23>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S24>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S25>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S26>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S27>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S28>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Preview Signal Check9'
 * '<S29>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S30>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S31>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S32>'  : 'root_model/Nonlinear MPC Controller/MPC/MPC Vector Signal Check11'
 * '<S33>'  : 'root_model/Nonlinear MPC Controller/MPC/NLMPC'
 * '<S34>'  : 'root_model/Quaternion Rotation/Quaternion Normalize'
 * '<S35>'  : 'root_model/Quaternion Rotation/V1'
 * '<S36>'  : 'root_model/Quaternion Rotation/V2'
 * '<S37>'  : 'root_model/Quaternion Rotation/V3'
 * '<S38>'  : 'root_model/Quaternion Rotation/Quaternion Normalize/Quaternion Modulus'
 * '<S39>'  : 'root_model/Quaternion Rotation/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S40>'  : 'root_model/Simulink Function/MATLAB Function'
 */
#endif                                 /* root_model_h_ */
