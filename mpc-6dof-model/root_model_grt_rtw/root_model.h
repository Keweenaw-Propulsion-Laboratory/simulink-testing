/*
 * root_model.h
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
#include "rtGetInf.h"
#include "rtGetNaN.h"
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

extern const ConstB_root_model_T root_model_ConstB;/* constant block i/o */

/* Class declaration for model root_model */
class root_model final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  root_model(root_model const&) = delete;

  /* Assignment Operator */
  root_model& operator= (root_model const&) & = delete;

  /* Move Constructor */
  root_model(root_model &&) = delete;

  /* Move Assignment Operator */
  root_model& operator= (root_model &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_root_model_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  root_model();

  /* Destructor */
  ~root_model();

  /* private data and function members */
 private:
  /* Block signals */
  B_root_model_T root_model_B;

  /* Block states */
  DW_root_model_T root_model_DW;

  /* Block continuous states */
  X_root_model_T root_model_X;

  /* Block Continuous state disabled vector */
  XDis_root_model_T root_model_XDis;

  /* private member function(s) for subsystem '<Root>/Simulink Function'*/
  void root_model_vehicle_state_func(const real_T x[17], const real_T u[3],
    real_T kP, real_T body_mass, real_T gravity_accel, const real_T
    inertia_tensor[9], real_T lever_arm, real_T x_dot[17]);
  void ekf_t_func(const real_T rtu_x[17], real_T rty_x_hat[17]);

  /* private member function(s) for subsystem '<Root>'*/
  real_T root_model_xnrm2(int32_T n, const real_T x[390], int32_T ix0);
  void root_model_qrFactor(const real_T A[221], const real_T S[289], const
    real_T Ns[169], real_T b_S[169]);
  void root_model_trisolve(const real_T A[169], real_T B[221]);
  void root_model_trisolve_l(const real_T A[169], real_T B[221]);
  real_T root_model_xnrm2_l(int32_T n, const real_T x[510], int32_T ix0);
  void root_model_qrFactor_l(const real_T A[289], const real_T S[289], const
    real_T Ns[221], real_T b_S[289]);
  void root_model_getUBounds(const real_T runtimedata_lastMV[3], const real_T
    runtimedata_MVMin[30], const real_T runtimedata_MVMax[30], const real_T
    runtimedata_MVRateMin[30], const real_T runtimedata_MVRateMax[30], real_T
    A_data[], int32_T A_size[2], real_T Bu_data[], int32_T Bu_size[1]);
  void root_model_computeJacobianState(real_T parameters_f1, real_T
    parameters_f2, real_T parameters_f3, const real_T parameters_f4[9], real_T
    parameters_f5, const real_T f0[17], real_T x0[17], real_T u0[3], real_T Jx
    [289], real_T Jmv[51]);
  void root_model_stateEvolution(const real_T X[187], const real_T U[33], real_T
    parameters_f1, real_T parameters_f2, real_T parameters_f3, const real_T
    parameters_f4[9], real_T parameters_f5, real_T c[170], real_T J[30090]);
  boolean_T root_model_any(const boolean_T x[26]);
  void root_model_reformJacobian(const real_T Jx_data[], const int32_T Jx_size[3],
    const real_T Jmv_data[], const real_T Je_data[], const int32_T Je_size[1],
    real_T Jc_data[], int32_T Jc_size[2]);
  void root_model_outputBounds(const real_T runtimedata_OutputMin[130], const
    real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
    c_data[], int32_T c_size[2], real_T Jc_data[], int32_T Jc_size[2]);
  void root_model_c4_mpclib_anonFcn2(const real_T runtimedata_x[17], const
    real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax[130],
    real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2, real_T
    runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9], real_T
    runtimedata_Parameters_f5, const real_T z[177], real_T varargout_1_data[],
    int32_T varargout_1_size[2], real_T varargout_2[170], real_T
    varargout_3_data[], int32_T varargout_3_size[2], real_T varargout_4[30090]);
  void root_model_factoryConstruct(int32_T nVarMax, int32_T mConstrMax, int32_T
    mIneq, int32_T mNonlinIneq, s_7iu1rzNoAoHigERqeOFm2G_root_T *obj);
  void root_mode_factoryConstruct_n0cp(int32_T MaxVars, int32_T obj_grad_size[1],
    int32_T obj_Hx_size[1], boolean_T *obj_hasLinear, int32_T *obj_nvar, int32_T
    *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T *obj_objtype,
    int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
    *obj_prev_hasLinear, real_T *obj_gammaScalar);
  void root_mod_factoryConstruct_n0cp3(int32_T mIneqMax, int32_T nVarMax,
    int32_T mConstrMax, s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
  void computeObjectiveAndUserGradient(const s_pVH6DktXFODPXN9VaZK2hE_root_T
    *obj_next_next_next_next_next_ne, const real_T x[177], real_T
    grad_workspace_data[], real_T *fval, int32_T *status);
  int32_T root_model_checkVectorNonFinite(int32_T N, const real_T vec_data[],
    int32_T iv0);
  int32_T computeConstraintsAndUserJacobi(int32_T
    obj_next_next_next_next_next_b_, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
    *obj_next_next_next_next_next_ne, const real_T x[177], real_T
    Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170], real_T
    JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
    JacEqTrans_workspace_data[], int32_T ldJE);
  void evalObjAndConstrAndDerivatives(int32_T obj_next_next_next_next_next_b_,
    const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *obj_next_next_next_next_next_ne,
    const s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next__0,
    const real_T x[177], real_T grad_workspace_data[], real_T
    Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170], real_T
    JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
    JacEqTrans_workspace_data[], int32_T ldJE, real_T *fval, int32_T *status);
  void root_mo_modifyOverheadPhaseOne_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
  void root_model_setProblemType(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, int32_T
    PROBLEM_TYPE);
  void root_model_initActiveSet(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
  void root_model_factoryConstruct_n0(int32_T maxRows, int32_T maxCols, int32_T *
    obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T obj_Q_size[2],
    int32_T obj_jpvt_data[], int32_T obj_jpvt_size[1], int32_T *obj_mrows,
    int32_T *obj_ncols, int32_T obj_tau_size[1], int32_T *obj_minRowCol,
    boolean_T *obj_usedPivoting);
  void root_model_factoryConstruct_n0c(int32_T MaxDims, int32_T obj_FMat_size[2],
    int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info, real_T
    *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_, real_T
    *obj_workspace_, real_T *obj_workspace2_);
  void root_model_computeGradLag(real_T workspace_data[], int32_T ldA, int32_T
    nVar, const real_T grad_data[], int32_T mIneq, const real_T AineqTrans_data[],
    const real_T AeqTrans_data[], const int32_T finiteFixed_data[], int32_T
    mFixed, const int32_T finiteLB_data[], int32_T mLB, const int32_T
    finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
  real_T root_mod_computePrimalFeasError(const real_T x[177], int32_T mLinIneq,
    int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170], const
    int32_T finiteLB_data[], int32_T mLB, const real_T lb[177], const int32_T
    finiteUB_data[], int32_T mUB);
  void root_model_computeDualFeasError(int32_T nVar, const real_T gradLag_data[],
    boolean_T *gradOK, real_T *val);
  void root_model_test_exit(sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
    const s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
    s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, const real_T lb[177], boolean_T
    *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done, boolean_T
    *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
    *Flags_stepType);
  void root_model_saveJacobian(s_7iu1rzNoAoHigERqeOFm2G_root_T *obj, int32_T
    nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T ineqCol0,
    const real_T JacCeqTrans_data[], int32_T ldJ);
  real_T root_model_computeComplError(const int32_T
    fscales_lineq_constraint_size[1], const int32_T
    fscales_cineq_constraint_size[1], const real_T xCurrent[177], int32_T mIneq,
    const real_T cIneq_data[], const int32_T finiteLB_data[], int32_T mLB, const
    real_T lb[177], const int32_T finiteUB_data[], int32_T mUB, const real_T
    lambda_data[], int32_T iL0);
  void root_model_computeGradLag_h(real_T workspace_data[], int32_T ldA, int32_T
    nVar, const real_T grad_data[], int32_T mIneq, const real_T AineqTrans_data[],
    const real_T AeqTrans_data[], const int32_T finiteFixed_data[], int32_T
    mFixed, const int32_T finiteLB_data[], int32_T mLB, const int32_T
    finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
  void root_mod_computeDualFeasError_k(int32_T nVar, const real_T gradLag_data[],
    boolean_T *gradOK, real_T *val);
  void root_m_updateWorkingSetForNewQP(const real_T xk[177],
    s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet, int32_T mIneq, int32_T
    mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170], int32_T mLB,
    const real_T lb[177], int32_T mUB, int32_T mFixed);
  void root_model_xswap(int32_T n, real_T x_data[], int32_T ix0, int32_T iy0);
  real_T root_model_xnrm2_o(int32_T n, const real_T x_data[], int32_T ix0);
  real_T root_model_xzlarfg(int32_T n, real_T *alpha1, real_T x_data[], int32_T
    ix0);
  void root_model_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T ia0,
                        int32_T lda, const real_T x_data[], int32_T ix0, real_T
                        y_data[]);
  void root_model_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                        real_T y_data[], real_T A_data[], int32_T ia0, int32_T
                        lda);
  void root_model_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T
    C_data[], int32_T ic0, int32_T ldc, real_T work_data[]);
  void root_model_qrf(real_T A_data[], const int32_T A_size[2], int32_T m,
                      int32_T n, int32_T nfxd, real_T tau_data[]);
  void root_model_qrpf(real_T A_data[], const int32_T A_size[2], int32_T m,
                       int32_T n, int32_T nfxd, real_T tau_data[], int32_T
                       jpvt_data[]);
  void root_model_xgeqp3(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T tau_size[1]);
  void root_model_factorQRE(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const real_T
    A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
  void root_model_xorgqr(int32_T m, int32_T n, int32_T k, real_T A_data[], const
    int32_T A_size[2], int32_T lda, const real_T tau_data[]);
  void root_model_sortLambdaQP(real_T lambda_data[], int32_T
    WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
    WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const
    int32_T WorkingSet_Wlocalidx_data[], real_T workspace_data[]);
  void root_model_test_exit_n(s7RdrPWkr8UPAUyTdDJkLaG_root__T *Flags,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, sG8JZ69axY52WWR6RKyApQC_root__T
    *MeritFunction, const int32_T fscales_lineq_constraint_size[1], const
    int32_T fscales_cineq_constraint_size[1], s_Jrk1gF3jitJmkPBgphEknE_root_T
    *WorkingSet, s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager, const real_T lb[177]);
  boolean_T root_model_BFGSUpdate(int32_T nvar, real_T Bk[31329], const real_T
    sk_data[], real_T yk_data[], real_T workspace_data[]);
  void root_model_factorQRE_h(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, int32_T
    mrows, int32_T ncols);
  void root_model_countsort(int32_T x_data[], int32_T xLen, int32_T
    workspace_data[], int32_T xMin, int32_T xMax);
  void root_model_removeConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, int32_T
    idx_global);
  int32_T root_model_RemoveDependentEq_(s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager);
  void root_model_RemoveDependentIneq_(s_Jrk1gF3jitJmkPBgphEknE_root_T
    *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace);
  int32_T root_model_rank(const real_T qrmanager_QR_data[], const int32_T
    qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols);
  void root_model_xgemv_i(int32_T m, int32_T n, const real_T A_data[], int32_T
    lda, const real_T x_data[], real_T y_data[]);
  real_T root_m_maxConstraintViolation_i(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    const real_T x_data[]);
  void root_model_xgemv_it(int32_T m, int32_T n, const real_T A_data[], int32_T
    lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
  real_T root__maxConstraintViolation_i0(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    const real_T x_data[], int32_T ix0);
  boolean_T root_mo_feasibleX0ForWorkingSet(real_T workspace_data[], const
    int32_T workspace_size[2], real_T xCurrent_data[],
    s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
    *qrmanager);
  void root_mod_RemoveDependentIneq__h(s_Jrk1gF3jitJmkPBgphEknE_root_T
    *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace);
  void root_model_xgemv_it3(int32_T m, int32_T n, const real_T A_data[], int32_T
    lda, const real_T x_data[], real_T y_data[]);
  real_T maxConstraintViolation_AMats_no(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    const real_T x_data[]);
  real_T maxConstraintViolation_AMats_re(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    const real_T x_data[]);
  real_T root_maxConstraintViolation_i0u(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    const real_T x_data[]);
  void root_model_PresolveWorkingSet(s_7iu1rzNoAoHigERqeOFm2G_root_T *solution,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
    *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager);
  void root_model_xgemv_it3a(int32_T m, int32_T n, const real_T A[31329],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  void root_model_computeGrad_StoreHx(s_1ECtvwuWb7KTJMqOG7pbsD_root_T *obj,
    const real_T H[31329], const real_T f_data[], const real_T x_data[]);
  real_T root_model_computeFval_ReuseHx(const s_1ECtvwuWb7KTJMqOG7pbsD_root_T
    *obj, real_T workspace_data[], const real_T f_data[], const real_T x_data[]);
  void root_model_xgeqrf(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, real_T tau_data[], int32_T tau_size[1]);
  void root_model_factorQR(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const real_T
    A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
  void root_model_squareQ_appendCol(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const
    real_T vec_data[], int32_T iv0);
  void root_model_deleteColMoveEnd(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, int32_T
    idx);
  boolean_T root_model_strcmp(const char_T a[7]);
  void root_model_xgemm(int32_T m, int32_T n, int32_T k, const real_T A[31329],
                        int32_T lda, const real_T B_data[], int32_T ib0, int32_T
                        ldb, real_T C_data[], int32_T ldc);
  void root_model_xgemm_h(int32_T m, int32_T n, int32_T k, const real_T A_data[],
    int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb, real_T C_data[],
    int32_T ldc);
  void root_model_fullColLDL2_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, int32_T
    LD_offset, int32_T NColsRemain);
  void root_model_partialColLDL3_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, int32_T
    LD_offset, int32_T NColsRemain);
  int32_T root_model_xpotrf(int32_T n, real_T A_data[], int32_T lda);
  void root_model_xgemv_it3ax(int32_T m, int32_T n, const real_T A_data[],
    int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[]);
  void root_model_factor_j(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const real_T A
    [31329], int32_T ndims, int32_T ldA);
  void root_model_factor(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const real_T A
    [31329], int32_T ndims, int32_T ldA);
  void root_model_solve_o(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, real_T
    rhs_data[]);
  void root_model_solve(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, real_T
                        rhs_data[]);
  void root_model_compute_deltax(const real_T H[31329],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, const s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
    s_mdyENYu4oaiCttMAPwg9iE_root_T *cholmanager, const
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, boolean_T alwaysPositiveDef);
  real_T root_model_xnrm2_ob(int32_T n, const real_T x_data[]);
  void root_model_xgemv_it3axg(int32_T m, int32_T n, const real_T A_data[],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  void root_model_feasibleratiotest(const real_T solution_xstar_data[], const
    real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
    workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
    real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
    real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
    int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
    boolean_T workingset_isActiveConstr_data[], const int32_T
    workingset_nWConstr[5], boolean_T isPhaseOne, real_T *alpha, boolean_T
    *newBlocking, int32_T *constrType, int32_T *constrIdx);
  void root_m_checkUnboundedOrIllPosed(s_7iu1rzNoAoHigERqeOFm2G_root_T *solution,
    const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective);
  void root_addBoundToActiveSetMatrix_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
    int32_T TYPE, int32_T idx_local);
  void root_model_addAineqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, int32_T
    idx_local);
  void root_model_compute_lambda(real_T workspace_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, const
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager);
  void root_checkStoppingAndUpdateFval(int32_T *activeSetChangeID, const real_T
    f_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *solution,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, const
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, s_Jrk1gF3jitJmkPBgphEknE_root_T *
    workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, int32_T
    runTimeOptions_MaxIterations, const boolean_T *updateFval);
  void root_model_iterate(const real_T H[31329], const real_T f_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T *
    cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
    options_SolverName[7], int32_T runTimeOptions_MaxIterations);
  void root_model_phaseone(const real_T H[31329], const real_T f_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T *
    cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
    options_SolverName[7], const somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions);
  void ro_checkStoppingAndUpdateFval_j(int32_T *activeSetChangeID,
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective,
    s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
    *qrmanager, int32_T runTimeOptions_MaxIterations, boolean_T *updateFval);
  void root_model_iterate_j(const real_T H[31329], const real_T f_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T *
    cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
    options_SolverName[7], int32_T runTimeOptions_MaxIterations);
  void root_model_linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar, real_T
    workspace_data[], const real_T H[31329], const real_T f_data[], const real_T
    x_data[]);
  void root_model_driver_i(const real_T H[31329], const real_T f_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T *
    cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
    somzaGboVhDG7PNQS6E98jD_root__T *options, const
    somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions);
  void root_model_addAeqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, int32_T
    idx_local);
  boolean_T root_model_soc(const real_T Hessian[31329], const real_T grad_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
    *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
    s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager, s_mdyENYu4oaiCttMAPwg9iE_root_T *
    CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *QPObjective, const
    somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
  real_T root_mod_maxConstraintViolation(const s_Jrk1gF3jitJmkPBgphEknE_root_T
    *obj, const real_T x_data[]);
  void root_model_normal(const real_T Hessian[31329], const real_T grad_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T
    *MeritFunction, s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace,
    s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T
    *QRManager, s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager,
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *QPObjective, const
    somzaGboVhDG7PNQS6E98jD_root__T *qpoptions, s7RdrPWkr8UPAUyTdDJkLaG_root__T *
    stepFlags);
  void root_model_relaxed(const real_T Hessian[31329], const real_T grad_data[],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T
    *MeritFunction, s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace,
    s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T
    *QRManager, s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager,
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *QPObjective,
    somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
  void root_model_step_k(s7RdrPWkr8UPAUyTdDJkLaG_root__T *stepFlags, real_T
    Hessian[31329], const real_T lb[177], s_7iu1rzNoAoHigERqeOFm2G_root_T
    *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
    *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
    s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager,
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *QPObjective,
    somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
  void root_model_stateEvolution_c(const real_T X[187], const real_T U[33],
    real_T parameters_f1, real_T parameters_f2, real_T parameters_f3, const
    real_T parameters_f4[9], real_T parameters_f5, real_T c[170]);
  void root_model_outputBounds_n(const real_T runtimedata_OutputMin[130], const
    real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
    c_data[], int32_T c_size[2]);
  void root_model_c4_mpclib_anonFcn2_n(const real_T runtimedata_x[17], const
    real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax[130],
    real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2, real_T
    runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9], real_T
    runtimedata_Parameters_f5, const real_T z[177], real_T varargout_1_data[],
    int32_T varargout_1_size[2], real_T varargout_2[170]);
  void root_model_evalObjAndConstr(int32_T obj_next_next_next_next_next_b_,
    const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *obj_next_next_next_next_next_ne,
    const s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next__0,
    const real_T x[177], real_T Cineq_workspace_data[], int32_T ineq0, real_T
    Ceq_workspace[170], real_T *fval, int32_T *status);
  void root_mod_computeLinearResiduals(const real_T x[177], int32_T nVar, real_T
    workspaceIneq_data[], const int32_T workspaceIneq_size[1], int32_T mLinIneq,
    const real_T AineqT_data[], const real_T bineq_data[], int32_T ldAi);
  real_T root_model_computeMeritFcn(real_T obj_penaltyParam, real_T fval, const
    real_T Cineq_workspace_data[], int32_T mIneq, const real_T Ceq_workspace[170],
    boolean_T evalWellDefined);
  void root_model_linesearch(boolean_T *evalWellDefined, const real_T
    bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const real_T
    WorkingSet_Aineq_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, real_T
    MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
    MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
    FcnEvaluator_next_next_next_nex, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
    *FcnEvaluator_next_next_next_n_0, const s_pVH6DktXFODPXN9VaZK2hE_root_T
    *FcnEvaluator_next_next_next_n_1, boolean_T socTaken, real_T *alpha, int32_T
    *exitflag);
  void root_model_driver(const real_T bineq_data[], const real_T lb[177],
    s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T
    *MeritFunction, const coder_internal_stickyStruct_2_T *FcnEvaluator,
    s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
    *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
    s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager,
    s_1ECtvwuWb7KTJMqOG7pbsD_root_T *QPObjective, const int32_T
    fscales_lineq_constraint_size[1], const int32_T
    fscales_cineq_constraint_size[1], real_T Hessian[31329]);
  void root_model_fmincon(const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
    *fun_workspace_runtimedata, const scI1GGq4Cpah2bZUcB8s80D_root__T
    *fun_workspace_userdata, const real_T x0[177], const real_T Aineq_data[],
    const real_T bineq_data[], const int32_T bineq_size[1], const real_T lb[177],
    const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *nonlcon_workspace_runtimedata, real_T
    x[177], real_T *fval, real_T *exitflag, sttYSJM5GCi2c1Eu0R50efC_root__T
    *output);
  real_T root_model_xnrm2_lx(int32_T n, const real_T x[578], int32_T ix0);
  void root_model_qrFactor_lx(const real_T A[289], const real_T S[289], const
    real_T Ns[289], real_T b_S[289]);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void root_model_derivatives();

  /* Projection member function */
  void root_model_projection();

  /* Real-Time Model */
  RT_MODEL_root_model_T root_model_M;
};

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
