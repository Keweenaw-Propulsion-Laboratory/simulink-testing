/*
 * root_model.cpp
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

#include "root_model.h"
#include "rtwtypes.h"
#include "root_model_types.h"
#include <cstring>
#include <emmintrin.h>
#include <cmath>
#include "rt_assert.h"
#include "getXUe_FXX9Wcuz.h"
#include "sind_CjVy6Jw4.h"
#include "cosd_ZK11uGDT.h"
#include "root_model_private.h"
#include "rt_hypotd_snf.h"
#include "isfinite_l3no8sDX.h"
#include "all_Dt0A7QFv.h"
#include "checkVectorNonFinite_yVQywJ02.h"
#include "xrotg_FR4eOElQ.h"
#include "div_nde_s32_floor.h"
#include "cmath"
#include "ekf_t_func.h"
#include "ekf_t_func_private.h"

/* Block signals (default storage) */
B_root_model_T root_model_B;

/* Continuous states */
X_root_model_T root_model_X;

/* Disabled State Vector */
XDis_root_model_T root_model_XDis;

/* Block states (default storage) */
DW_root_model_T root_model_DW;

/* Real-time model */
RT_MODEL_root_model_T root_model_M_{ };

RT_MODEL_root_model_T *const root_model_M{ &root_model_M_ };

/* Forward declaration for local functions */
static real_T root_model_xnrm2(int32_T n, const real_T x[390], int32_T ix0);
static void root_model_qrFactor(const real_T A[221], const real_T S[289], const
  real_T Ns[169], real_T b_S[169]);
static void root_model_trisolve(const real_T A[169], real_T B[221]);
static void root_model_trisolve_l(const real_T A[169], real_T B[221]);
static real_T root_model_xnrm2_l(int32_T n, const real_T x[510], int32_T ix0);
static void root_model_qrFactor_l(const real_T A[289], const real_T S[289],
  const real_T Ns[221], real_T b_S[289]);
static void root_model_getUBounds(const real_T runtimedata_lastMV[3], const
  real_T runtimedata_MVMin[30], const real_T runtimedata_MVMax[30], const real_T
  runtimedata_MVRateMin[30], const real_T runtimedata_MVRateMax[30], real_T
  A_data[], int32_T A_size[2], real_T Bu_data[], int32_T Bu_size[1]);
static void root_model_computeJacobianState(real_T parameters_f1, real_T
  parameters_f2, real_T parameters_f3, const real_T parameters_f4[9], real_T
  parameters_f5, const real_T f0[17], real_T x0[17], real_T u0[3], real_T Jx[289],
  real_T Jmv[51]);
static void root_model_stateEvolution(const real_T X[187], const real_T U[33],
  real_T parameters_f1, real_T parameters_f2, real_T parameters_f3, const real_T
  parameters_f4[9], real_T parameters_f5, real_T c[170], real_T J[30090]);
static boolean_T root_model_any(const boolean_T x[26]);
static void root_model_reformJacobian(const real_T Jx_data[], const int32_T
  Jx_size[3], const real_T Jmv_data[], const real_T Je_data[], const int32_T
  Je_size[1], real_T Jc_data[], int32_T Jc_size[2]);
static void root_model_outputBounds(const real_T runtimedata_OutputMin[130],
  const real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
  c_data[], int32_T c_size[2], real_T Jc_data[], int32_T Jc_size[2]);
static void root_model_c4_mpclib_anonFcn2(const real_T runtimedata_x[17], const
  real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax[130],
  real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2, real_T
  runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9], real_T
  runtimedata_Parameters_f5, const real_T z[177], real_T varargout_1_data[],
  int32_T varargout_1_size[2], real_T varargout_2[170], real_T varargout_3_data[],
  int32_T varargout_3_size[2], real_T varargout_4[30090]);
static void root_model_factoryConstruct(int32_T nVarMax, int32_T mConstrMax,
  int32_T mIneq, int32_T mNonlinIneq, s_7iu1rzNoAoHigERqeOFm2G_root_T *obj);
static void root_mode_factoryConstruct_n0cp(int32_T MaxVars, int32_T
  obj_grad_size[1], int32_T obj_Hx_size[1], boolean_T *obj_hasLinear, int32_T
  *obj_nvar, int32_T *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T
  *obj_objtype, int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
  *obj_prev_hasLinear, real_T *obj_gammaScalar);
static void root_mod_factoryConstruct_n0cp3(int32_T mIneqMax, int32_T nVarMax,
  int32_T mConstrMax, s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
static void computeObjectiveAndUserGradient(const
  s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next_ne, const real_T
  x[177], real_T grad_workspace_data[], real_T *fval, int32_T *status);
static int32_T root_model_checkVectorNonFinite(int32_T N, const real_T vec_data[],
  int32_T iv0);
static int32_T computeConstraintsAndUserJacobi(int32_T
  obj_next_next_next_next_next_b_, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *obj_next_next_next_next_next_ne, const real_T x[177], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170], real_T
  JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
  JacEqTrans_workspace_data[], int32_T ldJE);
static void evalObjAndConstrAndDerivatives(int32_T
  obj_next_next_next_next_next_b_, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *obj_next_next_next_next_next_ne, const s_pVH6DktXFODPXN9VaZK2hE_root_T
  *obj_next_next_next_next_next__0, const real_T x[177], real_T
  grad_workspace_data[], real_T Cineq_workspace_data[], int32_T ineq0, real_T
  Ceq_workspace[170], real_T JacIneqTrans_workspace_data[], int32_T iJI_col,
  int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T ldJE, real_T *fval,
  int32_T *status);
static void root_mo_modifyOverheadPhaseOne_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
static void root_model_setProblemType(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T PROBLEM_TYPE);
static void root_model_initActiveSet(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj);
static void root_model_factoryConstruct_n0(int32_T maxRows, int32_T maxCols,
  int32_T *obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T
  obj_Q_size[2], int32_T obj_jpvt_data[], int32_T obj_jpvt_size[1], int32_T
  *obj_mrows, int32_T *obj_ncols, int32_T obj_tau_size[1], int32_T
  *obj_minRowCol, boolean_T *obj_usedPivoting);
static void root_model_factoryConstruct_n0c(int32_T MaxDims, int32_T
  obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info,
  real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_,
  real_T *obj_workspace_, real_T *obj_workspace2_);
static void root_model_computeGradLag(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
static real_T root_mod_computePrimalFeasError(const real_T x[177], int32_T
  mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[177], const
  int32_T finiteUB_data[], int32_T mUB);
static void root_model_computeDualFeasError(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val);
static void root_model_test_exit(sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  const s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, const real_T lb[177], boolean_T
  *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done, boolean_T
  *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
  *Flags_stepType);
static void root_model_saveJacobian(s_7iu1rzNoAoHigERqeOFm2G_root_T *obj,
  int32_T nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T
  ineqCol0, const real_T JacCeqTrans_data[], int32_T ldJ);
static real_T root_model_computeComplError(const int32_T
  fscales_lineq_constraint_size[1], const int32_T fscales_cineq_constraint_size
  [1], const real_T xCurrent[177], int32_T mIneq, const real_T cIneq_data[],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[177], const
  int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[], int32_T iL0);
static void root_model_computeGradLag_h(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
static void root_mod_computeDualFeasError_k(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val);
static void root_m_updateWorkingSetForNewQP(const real_T xk[177],
  s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet, int32_T mIneq, int32_T
  mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170], int32_T mLB,
  const real_T lb[177], int32_T mUB, int32_T mFixed);
static void root_model_xswap(int32_T n, real_T x_data[], int32_T ix0, int32_T
  iy0);
static real_T root_model_xnrm2_o(int32_T n, const real_T x_data[], int32_T ix0);
static real_T root_model_xzlarfg(int32_T n, real_T *alpha1, real_T x_data[],
  int32_T ix0);
static void root_model_xgemv(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
static void root_model_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y_data[], real_T A_data[], int32_T ia0, int32_T lda);
static void root_model_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau,
  real_T C_data[], int32_T ic0, int32_T ldc, real_T work_data[]);
static void root_model_qrf(real_T A_data[], const int32_T A_size[2], int32_T m,
  int32_T n, int32_T nfxd, real_T tau_data[]);
static void root_model_qrpf(real_T A_data[], const int32_T A_size[2], int32_T m,
  int32_T n, int32_T nfxd, real_T tau_data[], int32_T jpvt_data[]);
static void root_model_xgeqp3(real_T A_data[], const int32_T A_size[2], int32_T
  m, int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T tau_size[1]);
static void root_model_factorQRE(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const
  real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
static void root_model_xorgqr(int32_T m, int32_T n, int32_T k, real_T A_data[],
  const int32_T A_size[2], int32_T lda, const real_T tau_data[]);
static void root_model_sortLambdaQP(real_T lambda_data[], int32_T
  WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
  WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const int32_T
  WorkingSet_Wlocalidx_data[], real_T workspace_data[]);
static void root_model_test_exit_n(s7RdrPWkr8UPAUyTdDJkLaG_root__T *Flags,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, sG8JZ69axY52WWR6RKyApQC_root__T
  *MeritFunction, const int32_T fscales_lineq_constraint_size[1], const int32_T
  fscales_cineq_constraint_size[1], s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *QRManager, const real_T lb[177]);
static boolean_T root_model_BFGSUpdate(int32_T nvar, real_T Bk[31329], const
  real_T sk_data[], real_T yk_data[], real_T workspace_data[]);
static void root_model_factorQRE_h(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, int32_T
  mrows, int32_T ncols);
static void root_model_countsort(int32_T x_data[], int32_T xLen, int32_T
  workspace_data[], int32_T xMin, int32_T xMax);
static void root_model_removeConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_global);
static int32_T root_model_RemoveDependentEq_(s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager);
static void root_model_RemoveDependentIneq_(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace);
static int32_T root_model_rank(const real_T qrmanager_QR_data[], const int32_T
  qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols);
static void root_model_xgemv_i(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[]);
static real_T root_m_maxConstraintViolation_i(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[]);
static void root_model_xgemv_it(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
static real_T root__maxConstraintViolation_i0(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[], int32_T ix0);
static boolean_T root_mo_feasibleX0ForWorkingSet(real_T workspace_data[], const
  int32_T workspace_size[2], real_T xCurrent_data[],
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager);
static void root_mod_RemoveDependentIneq__h(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace);
static void root_model_xgemv_it3(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[]);
static real_T maxConstraintViolation_AMats_no(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[]);
static real_T maxConstraintViolation_AMats_re(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[]);
static real_T root_maxConstraintViolation_i0u(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[]);
static void root_model_PresolveWorkingSet(s_7iu1rzNoAoHigERqeOFm2G_root_T
  *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace,
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager);
static void root_model_xgemv_it3a(int32_T m, int32_T n, const real_T A[31329],
  int32_T lda, const real_T x_data[], real_T y_data[]);
static void root_model_computeGrad_StoreHx(s_1ECtvwuWb7KTJMqOG7pbsD_root_T *obj,
  const real_T H[31329], const real_T f_data[], const real_T x_data[]);
static real_T root_model_computeFval_ReuseHx(const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *obj, real_T workspace_data[], const real_T
  f_data[], const real_T x_data[]);
static void root_model_xgeqrf(real_T A_data[], const int32_T A_size[2], int32_T
  m, int32_T n, real_T tau_data[], int32_T tau_size[1]);
static void root_model_factorQR(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const
  real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
static void root_model_squareQ_appendCol(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj,
  const real_T vec_data[], int32_T iv0);
static void root_model_deleteColMoveEnd(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj,
  int32_T idx);
static boolean_T root_model_strcmp(const char_T a[7]);
static void root_model_xgemm(int32_T m, int32_T n, int32_T k, const real_T A
  [31329], int32_T lda, const real_T B_data[], int32_T ib0, int32_T ldb, real_T
  C_data[], int32_T ldc);
static void root_model_xgemm_h(int32_T m, int32_T n, int32_T k, const real_T
  A_data[], int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb, real_T
  C_data[], int32_T ldc);
static void root_model_fullColLDL2_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  int32_T LD_offset, int32_T NColsRemain);
static void root_model_partialColLDL3_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  int32_T LD_offset, int32_T NColsRemain);
static int32_T root_model_xpotrf(int32_T n, real_T A_data[], int32_T lda);
static void root_model_xgemv_it3ax(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[]);
static void root_model_factor_j(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const
  real_T A[31329], int32_T ndims, int32_T ldA);
static void root_model_factor(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const real_T
  A[31329], int32_T ndims, int32_T ldA);
static void root_model_solve_o(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  real_T rhs_data[]);
static void root_model_solve(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, real_T
  rhs_data[]);
static void root_model_compute_deltax(const real_T H[31329],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, const s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *cholmanager, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, boolean_T alwaysPositiveDef);
static real_T root_model_xnrm2_ob(int32_T n, const real_T x_data[]);
static void root_model_xgemv_it3axg(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[]);
static void root_model_feasibleratiotest(const real_T solution_xstar_data[],
  const real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
  workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
  real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
  real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
  int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
  boolean_T workingset_isActiveConstr_data[], const int32_T workingset_nWConstr
  [5], boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
  *constrType, int32_T *constrIdx);
static void root_m_checkUnboundedOrIllPosed(s_7iu1rzNoAoHigERqeOFm2G_root_T
  *solution, const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective);
static void root_addBoundToActiveSetMatrix_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T TYPE, int32_T idx_local);
static void root_model_addAineqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_local);
static void root_model_compute_lambda(real_T workspace_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager);
static void root_checkStoppingAndUpdateFval(int32_T *activeSetChangeID, const
  real_T f_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *solution,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, int32_T
  runTimeOptions_MaxIterations, const boolean_T *updateFval);
static void root_model_iterate(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations);
static void root_model_phaseone(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], const somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions);
static void ro_checkStoppingAndUpdateFval_j(int32_T *activeSetChangeID,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective,
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager, int32_T runTimeOptions_MaxIterations, boolean_T *updateFval);
static void root_model_iterate_j(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations);
static void root_model_linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
  real_T workspace_data[], const real_T H[31329], const real_T f_data[], const
  real_T x_data[]);
static void root_model_driver_i(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
  somzaGboVhDG7PNQS6E98jD_root__T *options, const
  somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions);
static void root_model_addAeqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_local);
static boolean_T root_model_soc(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
static real_T root_mod_maxConstraintViolation(const
  s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, const real_T x_data[]);
static void root_model_normal(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_root__T *qpoptions,
  s7RdrPWkr8UPAUyTdDJkLaG_root__T *stepFlags);
static void root_model_relaxed(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
static void root_model_step_k(s7RdrPWkr8UPAUyTdDJkLaG_root__T *stepFlags, real_T
  Hessian[31329], const real_T lb[177], s_7iu1rzNoAoHigERqeOFm2G_root_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_root__T *qpoptions);
static void root_model_stateEvolution_c(const real_T X[187], const real_T U[33],
  real_T parameters_f1, real_T parameters_f2, real_T parameters_f3, const real_T
  parameters_f4[9], real_T parameters_f5, real_T c[170]);
static void root_model_outputBounds_n(const real_T runtimedata_OutputMin[130],
  const real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
  c_data[], int32_T c_size[2]);
static void root_model_c4_mpclib_anonFcn2_n(const real_T runtimedata_x[17],
  const real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax
  [130], real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2,
  real_T runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9],
  real_T runtimedata_Parameters_f5, const real_T z[177], real_T
  varargout_1_data[], int32_T varargout_1_size[2], real_T varargout_2[170]);
static void root_model_evalObjAndConstr(int32_T obj_next_next_next_next_next_b_,
  const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *obj_next_next_next_next_next_ne, const
  s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next__0, const real_T
  x[177], real_T Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170],
  real_T *fval, int32_T *status);
static void root_mod_computeLinearResiduals(const real_T x[177], int32_T nVar,
  real_T workspaceIneq_data[], const int32_T workspaceIneq_size[1], int32_T
  mLinIneq, const real_T AineqT_data[], const real_T bineq_data[], int32_T ldAi);
static real_T root_model_computeMeritFcn(real_T obj_penaltyParam, real_T fval,
  const real_T Cineq_workspace_data[], int32_T mIneq, const real_T
  Ceq_workspace[170], boolean_T evalWellDefined);
static void root_model_linesearch(boolean_T *evalWellDefined, const real_T
  bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const real_T
  WorkingSet_Aineq_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, real_T
  MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
  MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
  FcnEvaluator_next_next_next_nex, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *FcnEvaluator_next_next_next_n_0, const s_pVH6DktXFODPXN9VaZK2hE_root_T
  *FcnEvaluator_next_next_next_n_1, boolean_T socTaken, real_T *alpha, int32_T
  *exitflag);
static void root_model_driver(const real_T bineq_data[], const real_T lb[177],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T
  *MeritFunction, const coder_internal_stickyStruct_2_T *FcnEvaluator,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const int32_T fscales_lineq_constraint_size[1], const int32_T
  fscales_cineq_constraint_size[1], real_T Hessian[31329]);
static void root_model_fmincon(const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *fun_workspace_runtimedata, const scI1GGq4Cpah2bZUcB8s80D_root__T
  *fun_workspace_userdata, const real_T x0[177], const real_T Aineq_data[],
  const real_T bineq_data[], const int32_T bineq_size[1], const real_T lb[177],
  const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *nonlcon_workspace_runtimedata, real_T
  x[177], real_T *fval, real_T *exitflag, sttYSJM5GCi2c1Eu0R50efC_root__T
  *output);
static real_T root_model_xnrm2_lx(int32_T n, const real_T x[578], int32_T ix0);
static void root_model_qrFactor_lx(const real_T A[289], const real_T S[289],
  const real_T Ns[289], real_T b_S[289]);

/* Projection for root system: '<Root>' */
void root_model_projection(void)
{
  __m128d tmp;
  __m128d tmp_0;

  /* Projection for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  tmp_0 = _mm_div_pd(_mm_set1_pd(1.0), _mm_set1_pd(std::sqrt
    (((root_model_X.uDOFQuaternion_CSTATE[9] *
       root_model_X.uDOFQuaternion_CSTATE[9] +
       root_model_X.uDOFQuaternion_CSTATE[10] *
       root_model_X.uDOFQuaternion_CSTATE[10]) +
      root_model_X.uDOFQuaternion_CSTATE[11] *
      root_model_X.uDOFQuaternion_CSTATE[11]) +
     root_model_X.uDOFQuaternion_CSTATE[12] *
     root_model_X.uDOFQuaternion_CSTATE[12])));
  tmp = _mm_mul_pd(tmp_0, _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[9]));
  _mm_storeu_pd(&root_model_X.uDOFQuaternion_CSTATE[9], tmp);
  tmp_0 = _mm_mul_pd(tmp_0, _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[11]));
  _mm_storeu_pd(&root_model_X.uDOFQuaternion_CSTATE[11], tmp_0);
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3]{
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3]{
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t { rtsiGetT(si) };

  time_T tnew { rtsiGetSolverStopTime(si) };

  time_T h { rtsiGetStepSize(si) };

  real_T *x { rtsiGetContStates(si) };

  ODE3_IntgData *id { static_cast<ODE3_IntgData *>(rtsiGetSolverData(si)) };

  real_T *y { id->y };

  real_T *f0 { id->f[0] };

  real_T *f1 { id->f[1] };

  real_T *f2 { id->f[2] };

  real_T hB[3];
  int_T i;
  int_T nXc { 13 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  root_model_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  root_model_step();
  root_model_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  root_model_step();
  root_model_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  root_model_step();
  root_model_projection();
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static real_T root_model_xnrm2(int32_T n, const real_T x[390], int32_T ix0)
{
  real_T y;

  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T k;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
      if (std::isnan(y)) {
        k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k <= kend) {
            if (std::isnan(x[k - 1])) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            y = (rtInf);
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  /* End of Start for MATLABSystem: '<S8>/MATLAB System' */
  return y;
}

static void root_model_qrFactor(const real_T A[221], const real_T S[289], const
  real_T Ns[169], real_T b_S[169])
{
  __m128d tmp;
  real_T M[390];
  real_T b_R[169];
  real_T b_tau[13];
  real_T work[13];
  real_T M_0;
  real_T b_atmp;
  real_T xnorm;
  int32_T b_i;
  int32_T d;
  int32_T exitg1;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lastc;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T exitg2;

  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  for (i = 0; i < 13; i++) {
    for (b_i = 0; b_i < 17; b_i++) {
      b_atmp = 0.0;
      for (ii = 0; ii < 17; ii++) {
        b_atmp += S[17 * b_i + ii] * A[13 * ii + i];
      }

      M[b_i + 30 * i] = b_atmp;
    }

    for (b_i = 0; b_i < 13; b_i++) {
      M[(b_i + 30 * i) + 17] = Ns[13 * b_i + i];
    }

    b_tau[i] = 0.0;
    work[i] = 0.0;
  }

  for (b_i = 0; b_i < 13; b_i++) {
    ii = b_i * 30 + b_i;
    lastv = ii + 2;
    b_atmp = M[ii];
    b_tau[b_i] = 0.0;
    xnorm = root_model_xnrm2(29 - b_i, M, ii + 2);
    if (xnorm != 0.0) {
      M_0 = M[ii];
      xnorm = rt_hypotd_snf(M_0, xnorm);
      if (M_0 >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - b_i;
          scalarLB = (((((i - ii) + 29) / 2) << 1) + ii) + 2;
          vectorUB = scalarLB - 2;
          for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
            tmp = _mm_loadu_pd(&M[lastc - 1]);
            _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (lastc = scalarLB; lastc <= i + 30; lastc++) {
            M[lastc - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt + 1 < 20));

        xnorm = rt_hypotd_snf(b_atmp, root_model_xnrm2(29 - b_i, M, ii + 2));
        if (b_atmp >= 0.0) {
          xnorm = -xnorm;
        }

        b_tau[b_i] = (xnorm - b_atmp) / xnorm;
        b_atmp = 1.0 / (b_atmp - xnorm);
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&M[lastc - 1]);
          _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd(b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 30; lastc++) {
          M[lastc - 1] *= b_atmp;
        }

        for (lastc = 0; lastc <= knt; lastc++) {
          xnorm *= 1.0020841800044864E-292;
        }

        b_atmp = xnorm;
      } else {
        b_tau[b_i] = (xnorm - M_0) / xnorm;
        b_atmp = 1.0 / (M_0 - xnorm);
        i = ii - b_i;
        scalarLB = (((((i - ii) + 29) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&M[lastc - 1]);
          _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd(b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 30; lastc++) {
          M[lastc - 1] *= b_atmp;
        }

        b_atmp = xnorm;
      }
    }

    M[ii] = b_atmp;
    if (b_i + 1 < 13) {
      M[ii] = 1.0;
      scalarLB = ii + 31;
      if (b_tau[b_i] != 0.0) {
        lastv = 30 - b_i;
        i = ii - b_i;
        while ((lastv > 0) && (M[i + 29] == 0.0)) {
          lastv--;
          i--;
        }

        lastc = 11 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          i = lastc * 30 + ii;
          knt = i + 31;
          do {
            exitg1 = 0;
            if (knt <= (i + lastv) + 30) {
              if (M[knt - 1] != 0.0) {
                exitg1 = 1;
              } else {
                knt++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }

      if (lastv > 0) {
        if (lastc + 1 != 0) {
          if (lastc >= 0) {
            std::memset(&work[0], 0, static_cast<uint32_T>(lastc + 1) * sizeof
                        (real_T));
          }

          i = 30 * lastc + ii;
          for (vectorUB = scalarLB; vectorUB <= i + 31; vectorUB += 30) {
            xnorm = 0.0;
            d = vectorUB + lastv;
            for (knt = vectorUB; knt < d; knt++) {
              xnorm += M[(ii + knt) - vectorUB] * M[knt - 1];
            }

            knt = div_nde_s32_floor((vectorUB - ii) - 31, 30);
            work[knt] += xnorm;
          }
        }

        if (!(-b_tau[b_i] == 0.0)) {
          knt = ii;
          for (scalarLB = 0; scalarLB <= lastc; scalarLB++) {
            xnorm = work[scalarLB];
            if (xnorm != 0.0) {
              xnorm *= -b_tau[b_i];
              i = knt + 31;
              vectorUB = (lastv + knt) + 30;
              for (d = i; d <= vectorUB; d++) {
                M[d - 1] += M[((ii + d) - knt) - 31] * xnorm;
              }
            }

            knt += 30;
          }
        }
      }

      M[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 13; ii++) {
    for (b_i = 0; b_i <= ii; b_i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      b_R[b_i + 13 * ii] = M[30 * ii + b_i];
    }

    for (b_i = ii + 2; b_i < 14; b_i++) {
      b_R[(b_i + 13 * ii) - 1] = 0.0;
    }
  }

  for (b_i = 0; b_i < 13; b_i++) {
    for (ii = 0; ii < 13; ii++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      b_S[ii + 13 * b_i] = b_R[13 * ii + b_i];
    }
  }
}

static void root_model_trisolve(const real_T A[169], real_T B[221])
{
  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  for (int32_T b_j{0}; b_j < 17; b_j++) {
    int32_T jBcol;
    jBcol = 13 * b_j - 1;
    for (int32_T b_k{0}; b_k < 13; b_k++) {
      real_T B_0;
      int32_T B_tmp;
      int32_T k;
      int32_T kAcol;
      k = b_k + 1;
      kAcol = b_k * 13 - 1;
      B_tmp = (b_k + jBcol) + 1;
      B_0 = B[B_tmp];
      if (B_0 != 0.0) {
        B[B_tmp] = B_0 / A[(b_k + kAcol) + 1];
        for (int32_T i{k + 1}; i < 14; i++) {
          int32_T tmp;
          tmp = i + jBcol;
          B[tmp] -= A[i + kAcol] * B[B_tmp];
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S8>/MATLAB System' */
}

static void root_model_trisolve_l(const real_T A[169], real_T B[221])
{
  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  for (int32_T b_j{0}; b_j < 17; b_j++) {
    int32_T jBcol;
    jBcol = 13 * b_j;
    for (int32_T k{12}; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = 13 * k;
      tmp_0 = k + jBcol;
      tmp = B[tmp_0];
      if (tmp != 0.0) {
        B[tmp_0] = tmp / A[k + kAcol];
        for (int32_T b_i{0}; b_i < k; b_i++) {
          int32_T tmp_1;
          tmp_1 = b_i + jBcol;
          B[tmp_1] -= A[b_i + kAcol] * B[tmp_0];
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S8>/MATLAB System' */
}

static real_T root_model_xnrm2_l(int32_T n, const real_T x[510], int32_T ix0)
{
  real_T y;

  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T k;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
      if (std::isnan(y)) {
        k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k <= kend) {
            if (std::isnan(x[k - 1])) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            y = (rtInf);
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  /* End of Start for MATLABSystem: '<S8>/MATLAB System' */
  return y;
}

static void root_model_qrFactor_l(const real_T A[289], const real_T S[289],
  const real_T Ns[221], real_T b_S[289])
{
  __m128d tmp;
  real_T M[510];
  real_T b_R[289];
  real_T b_tau[17];
  real_T work[17];
  real_T M_0;
  real_T b_atmp;
  real_T xnorm;
  int32_T b_i;
  int32_T d;
  int32_T exitg1;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lastc;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T exitg2;

  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  for (i = 0; i < 17; i++) {
    for (b_i = 0; b_i < 17; b_i++) {
      b_atmp = 0.0;
      for (ii = 0; ii < 17; ii++) {
        b_atmp += S[17 * b_i + ii] * A[17 * ii + i];
      }

      M[b_i + 30 * i] = b_atmp;
    }

    for (b_i = 0; b_i < 13; b_i++) {
      M[(b_i + 30 * i) + 17] = Ns[17 * b_i + i];
    }

    b_tau[i] = 0.0;
    work[i] = 0.0;
  }

  for (b_i = 0; b_i < 17; b_i++) {
    ii = b_i * 30 + b_i;
    lastv = ii + 2;
    b_atmp = M[ii];
    b_tau[b_i] = 0.0;
    xnorm = root_model_xnrm2_l(29 - b_i, M, ii + 2);
    if (xnorm != 0.0) {
      M_0 = M[ii];
      xnorm = rt_hypotd_snf(M_0, xnorm);
      if (M_0 >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - b_i;
          scalarLB = (((((i - ii) + 29) / 2) << 1) + ii) + 2;
          vectorUB = scalarLB - 2;
          for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
            tmp = _mm_loadu_pd(&M[lastc - 1]);
            _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (lastc = scalarLB; lastc <= i + 30; lastc++) {
            M[lastc - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt + 1 < 20));

        xnorm = rt_hypotd_snf(b_atmp, root_model_xnrm2_l(29 - b_i, M, ii + 2));
        if (b_atmp >= 0.0) {
          xnorm = -xnorm;
        }

        b_tau[b_i] = (xnorm - b_atmp) / xnorm;
        b_atmp = 1.0 / (b_atmp - xnorm);
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&M[lastc - 1]);
          _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd(b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 30; lastc++) {
          M[lastc - 1] *= b_atmp;
        }

        for (lastc = 0; lastc <= knt; lastc++) {
          xnorm *= 1.0020841800044864E-292;
        }

        b_atmp = xnorm;
      } else {
        b_tau[b_i] = (xnorm - M_0) / xnorm;
        b_atmp = 1.0 / (M_0 - xnorm);
        i = ii - b_i;
        scalarLB = (((((i - ii) + 29) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&M[lastc - 1]);
          _mm_storeu_pd(&M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd(b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 30; lastc++) {
          M[lastc - 1] *= b_atmp;
        }

        b_atmp = xnorm;
      }
    }

    M[ii] = b_atmp;
    if (b_i + 1 < 17) {
      M[ii] = 1.0;
      scalarLB = ii + 31;
      if (b_tau[b_i] != 0.0) {
        lastv = 30 - b_i;
        i = ii - b_i;
        while ((lastv > 0) && (M[i + 29] == 0.0)) {
          lastv--;
          i--;
        }

        lastc = 15 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          i = lastc * 30 + ii;
          knt = i + 31;
          do {
            exitg1 = 0;
            if (knt <= (i + lastv) + 30) {
              if (M[knt - 1] != 0.0) {
                exitg1 = 1;
              } else {
                knt++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }

      if (lastv > 0) {
        if (lastc + 1 != 0) {
          if (lastc >= 0) {
            std::memset(&work[0], 0, static_cast<uint32_T>(lastc + 1) * sizeof
                        (real_T));
          }

          i = 30 * lastc + ii;
          for (vectorUB = scalarLB; vectorUB <= i + 31; vectorUB += 30) {
            xnorm = 0.0;
            d = vectorUB + lastv;
            for (knt = vectorUB; knt < d; knt++) {
              xnorm += M[(ii + knt) - vectorUB] * M[knt - 1];
            }

            knt = div_nde_s32_floor((vectorUB - ii) - 31, 30);
            work[knt] += xnorm;
          }
        }

        if (!(-b_tau[b_i] == 0.0)) {
          knt = ii;
          for (scalarLB = 0; scalarLB <= lastc; scalarLB++) {
            xnorm = work[scalarLB];
            if (xnorm != 0.0) {
              xnorm *= -b_tau[b_i];
              i = knt + 31;
              vectorUB = (lastv + knt) + 30;
              for (d = i; d <= vectorUB; d++) {
                M[d - 1] += M[((ii + d) - knt) - 31] * xnorm;
              }
            }

            knt += 30;
          }
        }
      }

      M[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 17; ii++) {
    for (b_i = 0; b_i <= ii; b_i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      b_R[b_i + 17 * ii] = M[30 * ii + b_i];
    }

    for (b_i = ii + 2; b_i < 18; b_i++) {
      b_R[(b_i + 17 * ii) - 1] = 0.0;
    }
  }

  for (b_i = 0; b_i < 17; b_i++) {
    for (ii = 0; ii < 17; ii++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      b_S[ii + 17 * b_i] = b_R[17 * ii + b_i];
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_getUBounds(const real_T runtimedata_lastMV[3], const
  real_T runtimedata_MVMin[30], const real_T runtimedata_MVMax[30], const real_T
  runtimedata_MVRateMin[30], const real_T runtimedata_MVRateMax[30], real_T
  A_data[], int32_T A_size[2], real_T Bu_data[], int32_T Bu_size[1])
{
  real_T b_Bu[120];
  real_T runtimedata_MVRateMin_0[3];
  real_T tmp_0[2];
  real_T ic_idx_0;
  real_T ic_idx_1;
  real_T ic_idx_2;
  real_T runtimedata_MVMax_0;
  real_T runtimedata_MVMax_1;
  real_T runtimedata_MVMax_2;
  real_T runtimedata_MVMin_0;
  real_T runtimedata_MVMin_1;
  real_T runtimedata_MVMin_2;
  real_T runtimedata_MVRateMax_0;
  real_T runtimedata_MVRateMax_1;
  real_T runtimedata_MVRateMax_2;
  real_T runtimedata_MVRateMin_1;
  real_T runtimedata_MVRateMin_2;
  real_T runtimedata_MVRateMin_3;
  int32_T Au_tmp_1;
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T k;
  int8_T Au[3600];
  int8_T Auf_data[3600];
  int8_T ii_data[120];
  int8_T As[9];
  int8_T Au_tmp[9];
  int8_T Au_tmp_0[9];
  int8_T idx_0[2];
  boolean_T x[120];
  boolean_T tmp[3];
  static const int8_T b_b[180]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    0, 1, 0, 0, 1, 0, 0, 1 };

  int32_T Auf_data_tmp;
  int32_T Auf_data_tmp_0;
  int32_T x_tmp;
  boolean_T exitg1;
  std::memset(&Au[0], 0, 3600U * sizeof(int8_T));
  std::memset(&b_Bu[0], 0, 120U * sizeof(real_T));
  std::memset(&x[0], 0, 120U * sizeof(boolean_T));
  ic_idx_0 = 1.0;
  ic_idx_1 = 2.0;
  ic_idx_2 = 3.0;
  for (i = 0; i < 9; i++) {
    As[i] = 0;
  }

  As[0] = 1;
  As[4] = 1;
  As[8] = 1;
  for (i = 0; i < 9; i++) {
    Au_tmp_1 = As[i];
    Au_tmp[i] = static_cast<int8_T>(Au_tmp_1);
    Au_tmp_0[i] = static_cast<int8_T>(-Au_tmp_1);
  }

  for (idx = 0; idx < 10; idx++) {
    runtimedata_MVRateMin_1 = runtimedata_MVRateMin[idx];
    runtimedata_MVRateMin_0[0] = runtimedata_MVRateMin_1;
    runtimedata_MVRateMin_2 = runtimedata_MVRateMin[idx + 10];
    runtimedata_MVRateMin_0[1] = runtimedata_MVRateMin_2;
    runtimedata_MVRateMin_3 = runtimedata_MVRateMin[idx + 20];
    runtimedata_MVRateMin_0[2] = runtimedata_MVRateMin_3;
    isfinite_l3no8sDX(runtimedata_MVRateMin_0, tmp);
    x[static_cast<int32_T>(ic_idx_0) - 1] = tmp[0];
    runtimedata_MVRateMax_0 = runtimedata_MVRateMax[idx];
    runtimedata_MVRateMin_0[0] = runtimedata_MVRateMax_0;
    x[static_cast<int32_T>(ic_idx_1) - 1] = tmp[1];
    runtimedata_MVRateMax_1 = runtimedata_MVRateMax[idx + 10];
    runtimedata_MVRateMin_0[1] = runtimedata_MVRateMax_1;
    x[static_cast<int32_T>(ic_idx_2) - 1] = tmp[2];
    runtimedata_MVRateMax_2 = runtimedata_MVRateMax[idx + 20];
    runtimedata_MVRateMin_0[2] = runtimedata_MVRateMax_2;
    isfinite_l3no8sDX(runtimedata_MVRateMin_0, tmp);
    x[static_cast<int32_T>(ic_idx_0 + 3.0) - 1] = tmp[0];
    runtimedata_MVMin_0 = runtimedata_MVMin[idx];
    runtimedata_MVRateMin_0[0] = runtimedata_MVMin_0;
    Au_tmp_1 = static_cast<int32_T>(ic_idx_1 + 3.0) - 1;
    x[static_cast<int32_T>(ic_idx_1 + 3.0) - 1] = tmp[1];
    runtimedata_MVMin_1 = runtimedata_MVMin[idx + 10];
    runtimedata_MVRateMin_0[1] = runtimedata_MVMin_1;
    x_tmp = static_cast<int32_T>(ic_idx_2 + 3.0) - 1;
    x[static_cast<int32_T>(ic_idx_2 + 3.0) - 1] = tmp[2];
    runtimedata_MVMin_2 = runtimedata_MVMin[idx + 20];
    runtimedata_MVRateMin_0[2] = runtimedata_MVMin_2;
    isfinite_l3no8sDX(runtimedata_MVRateMin_0, tmp);
    x[static_cast<int32_T>(ic_idx_0 + 6.0) - 1] = tmp[0];
    runtimedata_MVMax_0 = runtimedata_MVMax[idx];
    runtimedata_MVRateMin_0[0] = runtimedata_MVMax_0;
    x[static_cast<int32_T>(ic_idx_1 + 6.0) - 1] = tmp[1];
    runtimedata_MVMax_1 = runtimedata_MVMax[idx + 10];
    runtimedata_MVRateMin_0[1] = runtimedata_MVMax_1;
    x[static_cast<int32_T>(ic_idx_2 + 6.0) - 1] = tmp[2];
    runtimedata_MVMax_2 = runtimedata_MVMax[idx + 20];
    runtimedata_MVRateMin_0[2] = runtimedata_MVMax_2;
    isfinite_l3no8sDX(runtimedata_MVRateMin_0, tmp);
    x[static_cast<int32_T>(ic_idx_0 + 9.0) - 1] = tmp[0];
    x[static_cast<int32_T>(ic_idx_1 + 9.0) - 1] = tmp[1];
    x[static_cast<int32_T>(ic_idx_2 + 9.0) - 1] = tmp[2];
    for (i = 0; i < 3; i++) {
      Au[((static_cast<int32_T>(ic_idx_0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i];
      Au[((static_cast<int32_T>(ic_idx_1) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i + 1];
      Au[((static_cast<int32_T>(ic_idx_2) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i + 2];
    }

    for (i = 0; i < 3; i++) {
      Au[((static_cast<int32_T>(ic_idx_0 + 3.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i];
      Au[((static_cast<int32_T>(ic_idx_1 + 3.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i + 1];
      Au[((static_cast<int32_T>(ic_idx_2 + 3.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i + 2];
    }

    for (i = 0; i < 3; i++) {
      Au[((static_cast<int32_T>(ic_idx_0 + 6.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i];
      Au[((static_cast<int32_T>(ic_idx_1 + 6.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i + 1];
      Au[((static_cast<int32_T>(ic_idx_2 + 6.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp_0[3 * i + 2];
    }

    for (i = 0; i < 3; i++) {
      Au[((static_cast<int32_T>(ic_idx_0 + 9.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i];
      Au[((static_cast<int32_T>(ic_idx_1 + 9.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i + 1];
      Au[((static_cast<int32_T>(ic_idx_2 + 9.0) + 120 * i) + 360 * idx) - 1] =
        Au_tmp[3 * i + 2];
    }

    b_Bu[static_cast<int32_T>(ic_idx_0) - 1] = -runtimedata_MVRateMin_1;
    b_Bu[static_cast<int32_T>(ic_idx_1) - 1] = -runtimedata_MVRateMin_2;
    b_Bu[static_cast<int32_T>(ic_idx_2) - 1] = -runtimedata_MVRateMin_3;
    b_Bu[static_cast<int32_T>(ic_idx_0 + 3.0) - 1] = runtimedata_MVRateMax_0;
    b_Bu[static_cast<int32_T>(ic_idx_1 + 3.0) - 1] = runtimedata_MVRateMax_1;
    b_Bu[static_cast<int32_T>(ic_idx_2 + 3.0) - 1] = runtimedata_MVRateMax_2;
    b_Bu[static_cast<int32_T>(ic_idx_0 + 6.0) - 1] = -runtimedata_MVMin_0;
    b_Bu[static_cast<int32_T>(ic_idx_1 + 6.0) - 1] = -runtimedata_MVMin_1;
    b_Bu[static_cast<int32_T>(ic_idx_2 + 6.0) - 1] = -runtimedata_MVMin_2;
    b_Bu[static_cast<int32_T>(ic_idx_0 + 9.0) - 1] = runtimedata_MVMax_0;
    b_Bu[static_cast<int32_T>(ic_idx_1 + 9.0) - 1] = runtimedata_MVMax_1;
    b_Bu[static_cast<int32_T>(ic_idx_2 + 9.0) - 1] = runtimedata_MVMax_2;
    if (idx == 0) {
      runtimedata_MVRateMin_1 = b_Bu[static_cast<int32_T>(ic_idx_1) - 1] -
        runtimedata_lastMV[1];
      runtimedata_MVRateMin_2 = b_Bu[static_cast<int32_T>(ic_idx_2) - 1] -
        runtimedata_lastMV[2];
      b_Bu[static_cast<int32_T>(ic_idx_0) - 1] -= runtimedata_lastMV[0];
      b_Bu[static_cast<int32_T>(ic_idx_1) - 1] = runtimedata_MVRateMin_1;
      b_Bu[static_cast<int32_T>(ic_idx_2) - 1] = runtimedata_MVRateMin_2;
      _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_set_pd(b_Bu[x_tmp], b_Bu[Au_tmp_1]),
        _mm_set_pd(runtimedata_lastMV[2], runtimedata_lastMV[1])));
      b_Bu[static_cast<int32_T>(ic_idx_0 + 3.0) - 1] += runtimedata_lastMV[0];
      b_Bu[static_cast<int32_T>(ic_idx_1 + 3.0) - 1] = tmp_0[0];
      b_Bu[static_cast<int32_T>(ic_idx_2 + 3.0) - 1] = tmp_0[1];
    } else {
      for (i = 0; i < 3; i++) {
        Au_tmp_1 = (idx - 1) * 360;
        Au[((static_cast<int32_T>(ic_idx_0) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp[3 * i];
        Au[((static_cast<int32_T>(ic_idx_1) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp[3 * i + 1];
        Au[((static_cast<int32_T>(ic_idx_2) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp[3 * i + 2];
      }

      for (i = 0; i < 3; i++) {
        Au_tmp_1 = (idx - 1) * 360;
        Au[((static_cast<int32_T>(ic_idx_0 + 3.0) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp_0[3 * i];
        Au[((static_cast<int32_T>(ic_idx_1 + 3.0) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp_0[3 * i + 1];
        Au[((static_cast<int32_T>(ic_idx_2 + 3.0) + 120 * i) + Au_tmp_1) - 1] =
          Au_tmp_0[3 * i + 2];
      }
    }

    ic_idx_0 += 12.0;
    ic_idx_1 += 12.0;
    ic_idx_2 += 12.0;
  }

  idx = 0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 120)) {
    if (x[i]) {
      idx++;
      ii_data[idx - 1] = static_cast<int8_T>(i + 1);
      if (idx >= 120) {
        exitg1 = true;
      } else {
        i++;
      }
    } else {
      i++;
    }
  }

  if (idx < 1) {
    idx = 0;
  }

  if (idx > 0) {
    Bu_size[0] = idx;
    Au_tmp_1 = (idx / 2) << 1;
    x_tmp = Au_tmp_1 - 2;
    for (i = 0; i <= x_tmp; i += 2) {
      Bu_data[i] = b_Bu[ii_data[i] - 1];
      Bu_data[i + 1] = b_Bu[ii_data[i + 1] - 1];
    }

    for (i = Au_tmp_1; i < idx; i++) {
      Bu_data[i] = b_Bu[ii_data[i] - 1];
    }

    for (i = 0; i < 3; i++) {
      for (k = 0; k < 10; k++) {
        Au_tmp_1 = (idx / 16) << 4;
        x_tmp = Au_tmp_1 - 16;
        for (b_i = 0; b_i <= x_tmp; b_i += 16) {
          Auf_data_tmp = idx * i;
          Auf_data_tmp_0 = idx * 3 * k;
          Auf_data[(b_i + Auf_data_tmp) + Auf_data_tmp_0] = Au[((120 * i +
            ii_data[b_i]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 1] = Au[((120 * i +
            ii_data[b_i + 1]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 2] = Au[((120 * i +
            ii_data[b_i + 2]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 3] = Au[((120 * i +
            ii_data[b_i + 3]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 4] = Au[((120 * i +
            ii_data[b_i + 4]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 5] = Au[((120 * i +
            ii_data[b_i + 5]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 6] = Au[((120 * i +
            ii_data[b_i + 6]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 7] = Au[((120 * i +
            ii_data[b_i + 7]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 8] = Au[((120 * i +
            ii_data[b_i + 8]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 9] = Au[((120 * i +
            ii_data[b_i + 9]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 10] = Au[((120 * i
            + ii_data[b_i + 10]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 11] = Au[((120 * i
            + ii_data[b_i + 11]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 12] = Au[((120 * i
            + ii_data[b_i + 12]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 13] = Au[((120 * i
            + ii_data[b_i + 13]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 14] = Au[((120 * i
            + ii_data[b_i + 14]) + 360 * k) - 1];
          Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0) + 15] = Au[((120 * i
            + ii_data[b_i + 15]) + 360 * k) - 1];
        }

        for (b_i = Au_tmp_1; b_i < idx; b_i++) {
          Auf_data[(b_i + idx * i) + idx * 3 * k] = Au[((120 * i + ii_data[b_i])
            + 360 * k) - 1];
        }
      }
    }

    idx_0[0] = static_cast<int8_T>(idx);
    for (i = 0; i < 6; i++) {
      for (Au_tmp_1 = 0; Au_tmp_1 < idx; Au_tmp_1++) {
        root_model_B.Auf_data[Au_tmp_1 + idx * i] = 0.0;
      }

      for (Au_tmp_1 = 0; Au_tmp_1 < 30; Au_tmp_1++) {
        x_tmp = b_b[30 * i + Au_tmp_1];
        for (k = 0; k < idx; k++) {
          Auf_data_tmp = idx * i + k;
          root_model_B.Auf_data[Auf_data_tmp] += static_cast<real_T>
            (Auf_data[idx_0[0] * Au_tmp_1 + k] * x_tmp);
        }
      }
    }

    A_size[0] = idx;
    A_size[1] = 177;
    std::memset(&A_data[0], 0, static_cast<uint32_T>(idx * 170) * sizeof(real_T));
    Au_tmp_1 = idx * 6;
    for (i = 0; i < Au_tmp_1; i++) {
      A_data[i + idx * 170] = root_model_B.Auf_data[i];
    }

    std::memset(&A_data[176 * idx], 0, static_cast<uint32_T>(idx) * sizeof
                (real_T));
  } else {
    Bu_size[0] = 0;
    A_size[0] = 0;
    A_size[1] = 201;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_computeJacobianState(real_T parameters_f1, real_T
  parameters_f2, real_T parameters_f3, const real_T parameters_f4[9], real_T
  parameters_f5, const real_T f0[17], real_T x0[17], real_T u0[3], real_T Jx[289],
  real_T Jmv[51])
{
  __m128d tmp;
  real_T f[17];
  real_T xa[17];
  real_T parameters_f4_0[9];
  real_T ua[3];
  real_T dx;
  int32_T i;
  int32_T k;
  for (k = 0; k < 17; k++) {
    dx = std::abs(x0[k]);
    xa[k] = dx;
    if (dx < 1.0) {
      xa[k] = 1.0;
    }
  }

  for (k = 0; k < 17; k++) {
    dx = 1.0E-6 * xa[k];
    x0[k] += dx;
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(x0, u0, parameters_f1, parameters_f2,
      parameters_f3, parameters_f4_0, parameters_f5, f);
    x0[k] -= dx;
    for (i = 0; i <= 14; i += 2) {
      tmp = _mm_loadu_pd(&f[i]);
      _mm_storeu_pd(&Jx[i + 17 * k], _mm_div_pd(_mm_sub_pd(tmp, _mm_loadu_pd
        (&f0[i])), _mm_set1_pd(dx)));
    }

    for (i = 16; i < 17; i++) {
      Jx[i + 17 * k] = (f[i] - f0[i]) / dx;
    }
  }

  ua[0] = std::abs(u0[0]);
  ua[1] = std::abs(u0[1]);
  ua[2] = std::abs(u0[2]);
  for (k = 0; k < 3; k++) {
    if (ua[k] < 1.0) {
      ua[k] = 1.0;
    }
  }

  for (k = 0; k < 3; k++) {
    dx = 1.0E-6 * ua[k];
    u0[k] += dx;
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(x0, u0, parameters_f1, parameters_f2,
      parameters_f3, parameters_f4_0, parameters_f5, f);
    u0[k] -= dx;
    for (i = 0; i <= 14; i += 2) {
      tmp = _mm_loadu_pd(&f[i]);
      _mm_storeu_pd(&Jmv[i + 17 * k], _mm_div_pd(_mm_sub_pd(tmp, _mm_loadu_pd
        (&f0[i])), _mm_set1_pd(dx)));
    }

    for (i = 16; i < 17; i++) {
      Jmv[i + 17 * k] = (f[i] - f0[i]) / dx;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_stateEvolution(const real_T X[187], const real_T U[33],
  real_T parameters_f1, real_T parameters_f2, real_T parameters_f3, const real_T
  parameters_f4[9], real_T parameters_f5, real_T c[170], real_T J[30090])
{
  __m128d tmp;
  __m128d tmp_1;
  real_T Ak[289];
  real_T Ak1[289];
  real_T b_X[187];
  real_T Bk1[51];
  real_T val[51];
  real_T b_U[33];
  real_T fk[17];
  real_T ic[17];
  real_T xk[17];
  real_T tmp_0[2];
  int32_T Jx_tmp;
  int32_T i;
  int32_T k;
  int32_T xk_tmp;
  static const int8_T b[180]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
    1, 0, 0, 1, 0, 0, 1 };

  real_T b_X_0[17];
  real_T parameters_f4_0[9];
  real_T b_U_0[3];
  int32_T c_tmp;
  std::memset(&root_model_B.Jx_m[0], 0, 28900U * sizeof(real_T));
  std::memset(&root_model_B.Jmv[0], 0, 5100U * sizeof(real_T));
  std::memset(&c[0], 0, 170U * sizeof(real_T));
  for (Jx_tmp = 0; Jx_tmp <= 14; Jx_tmp += 2) {
    tmp_0[0] = Jx_tmp;
    tmp_0[1] = Jx_tmp + 1;
    tmp_1 = _mm_loadu_pd(&tmp_0[0]);
    _mm_storeu_pd(&ic[Jx_tmp], _mm_add_pd(tmp_1, _mm_set1_pd(1.0)));
  }

  for (Jx_tmp = 16; Jx_tmp < 17; Jx_tmp++) {
    ic[Jx_tmp] = static_cast<real_T>(Jx_tmp) + 1.0;
  }

  for (Jx_tmp = 0; Jx_tmp < 11; Jx_tmp++) {
    b_U[3 * Jx_tmp] = U[Jx_tmp];
    b_U[3 * Jx_tmp + 1] = U[Jx_tmp + 11];
    b_U[3 * Jx_tmp + 2] = U[Jx_tmp + 22];
    for (i = 0; i < 17; i++) {
      b_X[i + 17 * Jx_tmp] = X[11 * i + Jx_tmp];
    }
  }

  for (i = 0; i < 10; i++) {
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(&b_X[17 * i], &b_U[3 * i], parameters_f1,
      parameters_f2, parameters_f3, parameters_f4_0, parameters_f5, fk);
    std::memcpy(&b_X_0[0], &(*(real_T (*)[17])&b_X[17 * i])[0], 17U * sizeof
                (real_T));
    for (Jx_tmp = 0; Jx_tmp < 3; Jx_tmp++) {
      b_U_0[Jx_tmp] = (*(real_T (*)[3])&b_U[3 * i])[Jx_tmp];
    }

    root_model_computeJacobianState(parameters_f1, parameters_f2, parameters_f3,
      parameters_f4, parameters_f5, fk, b_X_0, b_U_0, Ak, val);
    xk_tmp = (i + 1) * 17;
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(&b_X[xk_tmp], &b_U[3 * i], parameters_f1,
      parameters_f2, parameters_f3, parameters_f4_0, parameters_f5, xk);
    std::memcpy(&b_X_0[0], &(*(real_T (*)[17])&b_X[xk_tmp])[0], 17U * sizeof
                (real_T));
    for (Jx_tmp = 0; Jx_tmp < 3; Jx_tmp++) {
      b_U_0[Jx_tmp] = (*(real_T (*)[3])&b_U[3 * i])[Jx_tmp];
    }

    root_model_computeJacobianState(parameters_f1, parameters_f2, parameters_f3,
      parameters_f4, parameters_f5, xk, b_X_0, b_U_0, Ak1, Bk1);
    for (k = 0; k < 17; k++) {
      c_tmp = static_cast<int32_T>(ic[k]);
      c[c_tmp - 1] = (b_X[17 * i + k] + (fk[k] + xk[k]) * 0.5) - b_X[xk_tmp + k];
      if (i > 0) {
        for (Jx_tmp = 0; Jx_tmp < 17; Jx_tmp++) {
          root_model_B.Jx_m[((static_cast<int32_T>(ic[Jx_tmp]) + 170 * k) + 2890
                             * (i - 1)) - 1] = Ak[17 * k + Jx_tmp] * 0.5;
        }

        Jx_tmp = ((170 * k + c_tmp) + (i - 1) * 2890) - 1;
        root_model_B.Jx_m[Jx_tmp]++;
      }
    }

    for (k = 0; k < 17; k++) {
      for (Jx_tmp = 0; Jx_tmp < 17; Jx_tmp++) {
        root_model_B.Jx_m[((static_cast<int32_T>(ic[Jx_tmp]) + 170 * k) + 2890 *
                           i) - 1] = Ak1[17 * k + Jx_tmp] * 0.5;
      }

      Jx_tmp = ((170 * k + static_cast<int32_T>(ic[k])) + 2890 * i) - 1;
      root_model_B.Jx_m[Jx_tmp]--;
    }

    for (Jx_tmp = 0; Jx_tmp <= 48; Jx_tmp += 2) {
      tmp_1 = _mm_loadu_pd(&val[Jx_tmp]);
      tmp = _mm_loadu_pd(&Bk1[Jx_tmp]);
      _mm_storeu_pd(&val[Jx_tmp], _mm_mul_pd(_mm_add_pd(tmp_1, tmp), _mm_set1_pd
        (0.5)));
    }

    for (Jx_tmp = 50; Jx_tmp < 51; Jx_tmp++) {
      val[Jx_tmp] = (val[Jx_tmp] + Bk1[Jx_tmp]) * 0.5;
    }

    for (k = 0; k < 3; k++) {
      for (Jx_tmp = 0; Jx_tmp < 17; Jx_tmp++) {
        root_model_B.Jmv[((static_cast<int32_T>(ic[Jx_tmp]) + 170 * k) + 510 * i)
          - 1] = val[17 * k + Jx_tmp];
      }
    }

    for (Jx_tmp = 0; Jx_tmp <= 14; Jx_tmp += 2) {
      tmp_1 = _mm_loadu_pd(&ic[Jx_tmp]);
      _mm_storeu_pd(&ic[Jx_tmp], _mm_add_pd(tmp_1, _mm_set1_pd(17.0)));
    }

    for (Jx_tmp = 16; Jx_tmp < 17; Jx_tmp++) {
      ic[Jx_tmp] += 17.0;
    }
  }

  for (Jx_tmp = 0; Jx_tmp < 6; Jx_tmp++) {
    std::memset(&root_model_B.dv[Jx_tmp * 170], 0, 170U * sizeof(real_T));
    for (i = 0; i < 30; i++) {
      xk_tmp = b[30 * Jx_tmp + i];
      for (k = 0; k <= 168; k += 2) {
        tmp_1 = _mm_loadu_pd(&root_model_B.Jmv[170 * i + k]);
        c_tmp = 170 * Jx_tmp + k;
        tmp = _mm_loadu_pd(&root_model_B.dv[c_tmp]);
        _mm_storeu_pd(&root_model_B.dv[c_tmp], _mm_add_pd(_mm_mul_pd(tmp_1,
          _mm_set1_pd(static_cast<real_T>(xk_tmp))), tmp));
      }
    }
  }

  for (Jx_tmp = 0; Jx_tmp < 170; Jx_tmp++) {
    for (i = 0; i < 6; i++) {
      root_model_B.Jmv_g[i + 6 * Jx_tmp] = root_model_B.dv[170 * i + Jx_tmp];
    }

    for (i = 0; i < 170; i++) {
      J[i + 177 * Jx_tmp] = root_model_B.Jx_m[170 * i + Jx_tmp];
    }

    for (i = 0; i < 6; i++) {
      J[(i + 177 * Jx_tmp) + 170] = root_model_B.Jmv_g[6 * Jx_tmp + i];
    }

    J[177 * Jx_tmp + 176] = 0.0;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static boolean_T root_model_any(const boolean_T x[26])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 25)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_reformJacobian(const real_T Jx_data[], const int32_T
  Jx_size[3], const real_T Jmv_data[], const real_T Je_data[], const int32_T
  Je_size[1], real_T Jc_data[], int32_T Jc_size[2])
{
  real_T Je_data_0[260];
  int16_T Je[2];
  int16_T Jx[2];
  int16_T tmp[2];
  static const int8_T b_b[180]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    0, 1, 0, 0, 1, 0, 0, 1 };

  if (Jx_size[0] == 0) {
    Jc_size[0] = 0;
    Jc_size[1] = 0;
  } else {
    int32_T loop_ub;
    int32_T loop_ub_0;
    int32_T tmp_size_idx_0;
    Jx[0] = static_cast<int16_T>(Jx_size[0]);
    loop_ub = Jx_size[0];
    for (int32_T i_1{0}; i_1 < loop_ub; i_1++) {
      for (int32_T i_0{0}; i_0 < 170; i_0++) {
        root_model_B.varargin_1_data[i_0 + 170 * i_1] = Jx_data[Jx[0] * i_0 +
          i_1];
      }
    }

    tmp[0] = 170;
    Jx[0] = static_cast<int16_T>(Jx_size[0]);
    tmp_size_idx_0 = Jx_size[0];
    for (int32_T i_1{0}; i_1 < 6; i_1++) {
      for (int32_T i_0{0}; i_0 < loop_ub; i_0++) {
        root_model_B.tmp_data_f[i_0 + tmp_size_idx_0 * i_1] = 0.0;
      }

      for (int32_T i_0{0}; i_0 < 30; i_0++) {
        loop_ub_0 = b_b[30 * i_1 + i_0];
        for (int32_T i{0}; i < loop_ub; i++) {
          int32_T tmp_data_tmp;
          tmp_data_tmp = tmp_size_idx_0 * i_1 + i;
          root_model_B.tmp_data_f[tmp_data_tmp] += Jmv_data[Jx[0] * i_0 + i] *
            static_cast<real_T>(loop_ub_0);
        }
      }
    }

    for (int32_T i_1{0}; i_1 < loop_ub; i_1++) {
      for (int32_T i_0{0}; i_0 < 6; i_0++) {
        root_model_B.Jmv_data[i_0 + 6 * i_1] =
          root_model_B.tmp_data_f[tmp_size_idx_0 * i_0 + i_1];
      }
    }

    Jx[0] = 6;
    loop_ub_0 = Je_size[0];
    if (loop_ub_0 - 1 >= 0) {
      std::memcpy(&Je_data_0[0], &Je_data[0], static_cast<uint32_T>(loop_ub_0) *
                  sizeof(real_T));
    }

    Je[0] = (Je_size[0] != 0);
    Jc_size[0] = Je[0] + 176;
    Jc_size[1] = Jx_size[0];
    loop_ub_0 = Je[0];
    for (int32_T i_1{0}; i_1 < loop_ub; i_1++) {
      for (int32_T i_0{0}; i_0 < 170; i_0++) {
        Jc_data[i_0 + Jc_size[0] * i_1] = root_model_B.varargin_1_data[tmp[0] *
          i_1 + i_0];
      }

      for (int32_T i_0{0}; i_0 < 6; i_0++) {
        Jc_data[(i_0 + Jc_size[0] * i_1) + 170] = root_model_B.Jmv_data[Jx[0] *
          i_1 + i_0];
      }

      if (loop_ub_0 - 1 >= 0) {
        Jc_data[176 + Jc_size[0] * i_1] = Je_data_0[Je[0] * i_1];
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_outputBounds(const real_T runtimedata_OutputMin[130],
  const real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
  c_data[], int32_T c_size[2], real_T Jc_data[], int32_T Jc_size[2])
{
  __m128d tmp_1;
  real_T b_c[260];
  real_T Ck[221];
  real_T val[221];
  real_T xa[17];
  real_T xk[17];
  real_T ic[13];
  real_T icf_tmp[13];
  real_T tmp_0[2];
  real_T dx;
  real_T runtimedata_OutputMin_0;
  int32_T icf_tmp_2[26];
  int32_T icf_tmp_0[13];
  int32_T icf_tmp_1[13];
  int32_T Jx_size[3];
  int32_T Je_size[1];
  int32_T Jx_data_tmp;
  int32_T Jx_data_tmp_0;
  int32_T c_k;
  int32_T i;
  int32_T i_0;
  int32_T icf_tmp_3;
  int32_T tmp_size_idx_0;
  int32_T vectorUB;
  int16_T tmp_data[260];
  int8_T Je[260];
  boolean_T icf[260];
  boolean_T tmp[130];
  boolean_T icf_0[26];
  boolean_T x[13];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T y;
  for (i = 0; i < 130; i++) {
    tmp[i] = std::isinf(runtimedata_OutputMin[i]);
  }

  all_Dt0A7QFv(tmp, x);
  y = true;
  c_k = 0;
  exitg1 = false;
  while ((!exitg1) && (c_k < 13)) {
    if (!x[c_k]) {
      y = false;
      exitg1 = true;
    } else {
      c_k++;
    }
  }

  guard1 = false;
  if (y) {
    for (i = 0; i < 130; i++) {
      tmp[i] = std::isinf(runtimedata_OutputMax[i]);
    }

    all_Dt0A7QFv(tmp, x);
    c_k = 0;
    exitg1 = false;
    while ((!exitg1) && (c_k < 13)) {
      if (!x[c_k]) {
        y = false;
        exitg1 = true;
      } else {
        c_k++;
      }
    }

    if (y) {
      c_size[0] = 0;
      c_size[1] = 0;
      Jc_size[0] = 0;
      Jc_size[1] = 0;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    for (i = 0; i < 260; i++) {
      b_c[i] = 0.0;
      icf[i] = true;
    }

    std::memset(&root_model_B.Jx[0], 0, 44200U * sizeof(real_T));
    std::memset(&Je[0], 0, 260U * sizeof(int8_T));
    for (i = 0; i <= 10; i += 2) {
      tmp_0[0] = i;
      tmp_0[1] = i + 1;
      tmp_1 = _mm_loadu_pd(&tmp_0[0]);
      _mm_storeu_pd(&ic[i], _mm_add_pd(tmp_1, _mm_set1_pd(1.0)));
    }

    for (i = 12; i < 13; i++) {
      ic[i] = static_cast<real_T>(i) + 1.0;
    }

    for (c_k = 0; c_k < 10; c_k++) {
      for (i = 0; i < 13; i++) {
        dx = ic[i];
        icf_tmp_0[i] = static_cast<int32_T>(dx);
        runtimedata_OutputMin_0 = runtimedata_OutputMin[10 * i + c_k];
        icf[static_cast<int32_T>(dx) - 1] = ((!std::isinf
          (runtimedata_OutputMin_0)) && (!std::isnan(runtimedata_OutputMin_0)));
        icf_tmp[i] = dx + 13.0;
        icf_tmp_1[i] = static_cast<int32_T>(dx + 13.0);
      }

      for (i = 0; i < 13; i++) {
        icf_tmp_3 = icf_tmp_1[i];
        dx = runtimedata_OutputMax[10 * i + c_k];
        icf[icf_tmp_3 - 1] = ((!std::isinf(dx)) && (!std::isnan(dx)));
        icf_tmp_2[i] = icf_tmp_0[i] - 1;
        icf_tmp_2[i + 13] = icf_tmp_3 - 1;
      }

      for (i = 0; i < 26; i++) {
        icf_0[i] = icf[icf_tmp_2[i]];
      }

      if (root_model_any(icf_0)) {
        for (icf_tmp_3 = 0; icf_tmp_3 < 17; icf_tmp_3++) {
          dx = X[(11 * icf_tmp_3 + c_k) + 1];
          xk[icf_tmp_3] = dx;
          dx = std::abs(dx);
          xa[icf_tmp_3] = dx;
          if (dx < 1.0) {
            xa[icf_tmp_3] = 1.0;
          }
        }

        for (icf_tmp_3 = 0; icf_tmp_3 < 17; icf_tmp_3++) {
          dx = 1.0E-6 * xa[icf_tmp_3];
          xk[icf_tmp_3] += dx;
          for (i = 0; i < 13; i++) {
            Ck[i + 13 * icf_tmp_3] = (xk[i] - X[(11 * i + c_k) + 1]) / dx;
          }

          xk[icf_tmp_3] -= dx;
        }

        for (i = 0; i < 13; i++) {
          b_c[icf_tmp_0[i] - 1] = (runtimedata_OutputMin[10 * i + c_k] - e) - X
            [(11 * i + c_k) + 1];
        }

        for (i = 0; i < 13; i++) {
          b_c[icf_tmp_1[i] - 1] = (X[(11 * i + c_k) + 1] -
            runtimedata_OutputMax[10 * i + c_k]) - e;
        }

        for (i = 0; i <= 218; i += 2) {
          tmp_1 = _mm_loadu_pd(&Ck[i]);
          _mm_storeu_pd(&val[i], _mm_mul_pd(tmp_1, _mm_set1_pd(-1.0)));
        }

        for (i = 220; i < 221; i++) {
          val[i] = -Ck[i];
        }

        for (icf_tmp_3 = 0; icf_tmp_3 < 17; icf_tmp_3++) {
          for (i = 0; i < 13; i++) {
            root_model_B.Jx[((static_cast<int32_T>(ic[i]) + 260 * icf_tmp_3) +
                             4420 * c_k) - 1] = val[13 * icf_tmp_3 + i];
          }
        }

        for (icf_tmp_3 = 0; icf_tmp_3 < 17; icf_tmp_3++) {
          for (i = 0; i < 13; i++) {
            root_model_B.Jx[((static_cast<int32_T>(ic[i] + 13.0) + 260 *
                              icf_tmp_3) + 4420 * c_k) - 1] = Ck[13 * icf_tmp_3
              + i];
          }
        }

        for (i = 0; i < 13; i++) {
          Je[static_cast<int32_T>(ic[i]) - 1] = -1;
        }

        for (i = 0; i < 13; i++) {
          Je[static_cast<int32_T>(icf_tmp[i]) - 1] = -1;
        }
      }

      for (i = 0; i <= 10; i += 2) {
        tmp_1 = _mm_loadu_pd(&ic[i]);
        _mm_storeu_pd(&ic[i], _mm_add_pd(tmp_1, _mm_set1_pd(26.0)));
      }

      for (i = 12; i < 13; i++) {
        ic[i] += 26.0;
      }
    }

    c_k = 0;
    for (i = 0; i < 260; i++) {
      if (icf[i]) {
        c_k++;
      }
    }

    tmp_size_idx_0 = c_k;
    c_k = 0;
    for (i = 0; i < 260; i++) {
      if (icf[i]) {
        tmp_data[c_k] = static_cast<int16_T>(i);
        c_k++;
      }
    }

    c_size[0] = tmp_size_idx_0;
    c_size[1] = 1;
    for (i = 0; i < tmp_size_idx_0; i++) {
      c_data[i] = b_c[tmp_data[i]];
    }

    Jx_size[0] = tmp_size_idx_0;
    Jx_size[1] = 17;
    Jx_size[2] = 10;
    for (i = 0; i < 10; i++) {
      for (icf_tmp_3 = 0; icf_tmp_3 < 17; icf_tmp_3++) {
        c_k = (tmp_size_idx_0 / 2) << 1;
        vectorUB = c_k - 2;
        for (i_0 = 0; i_0 <= vectorUB; i_0 += 2) {
          Jx_data_tmp = tmp_size_idx_0 * icf_tmp_3;
          Jx_data_tmp_0 = tmp_size_idx_0 * 17 * i;
          root_model_B.Jx_data[(i_0 + Jx_data_tmp) + Jx_data_tmp_0] =
            root_model_B.Jx[(260 * icf_tmp_3 + tmp_data[i_0]) + 4420 * i];
          root_model_B.Jx_data[((i_0 + Jx_data_tmp) + Jx_data_tmp_0) + 1] =
            root_model_B.Jx[(260 * icf_tmp_3 + tmp_data[i_0 + 1]) + 4420 * i];
        }

        for (i_0 = c_k; i_0 < tmp_size_idx_0; i_0++) {
          root_model_B.Jx_data[(i_0 + tmp_size_idx_0 * icf_tmp_3) +
            tmp_size_idx_0 * 17 * i] = root_model_B.Jx[(260 * icf_tmp_3 +
            tmp_data[i_0]) + 4420 * i];
        }
      }
    }

    icf_tmp_3 = tmp_size_idx_0 * 3 * 10;
    if (icf_tmp_3 - 1 >= 0) {
      std::memset(&root_model_B.tmp_data[0], 0, static_cast<uint32_T>(icf_tmp_3)
                  * sizeof(real_T));
    }

    Je_size[0] = tmp_size_idx_0;
    for (i = 0; i < tmp_size_idx_0; i++) {
      b_c[i] = Je[tmp_data[i]];
    }

    root_model_reformJacobian(root_model_B.Jx_data, Jx_size,
      root_model_B.tmp_data, b_c, Je_size, Jc_data, Jc_size);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_c4_mpclib_anonFcn2(const real_T runtimedata_x[17], const
  real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax[130],
  real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2, real_T
  runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9], real_T
  runtimedata_Parameters_f5, const real_T z[177], real_T varargout_1_data[],
  int32_T varargout_1_size[2], real_T varargout_2[170], real_T varargout_3_data[],
  int32_T varargout_3_size[2], real_T varargout_4[30090])
{
  real_T varargin_1_data[260];
  real_T X[187];
  real_T U[33];
  real_T e;
  int32_T b_varargin_1_size[2];
  int32_T varargin_1_size[2];
  int32_T loop_ub;
  int32_T varargout_1_size_tmp;
  int16_T input_sizes_idx_0;
  uint8_T sizes_idx_0;
  boolean_T sizes_idx_1_tmp;
  getXUe_FXX9Wcuz(z, runtimedata_x, X, U, &e);
  root_model_stateEvolution(X, U, runtimedata_Parameters_f1,
    runtimedata_Parameters_f2, runtimedata_Parameters_f3,
    runtimedata_Parameters_f4, runtimedata_Parameters_f5, varargout_2,
    varargout_4);
  root_model_outputBounds(runtimedata_OutputMin, runtimedata_OutputMax, X, e,
    varargin_1_data, varargin_1_size, root_model_B.b_varargin_1_data,
    b_varargin_1_size);
  sizes_idx_1_tmp = ((varargin_1_size[0] != 0) && (varargin_1_size[1] != 0));
  if ((!sizes_idx_1_tmp) || sizes_idx_1_tmp) {
    input_sizes_idx_0 = static_cast<int16_T>(varargin_1_size[0]);
  } else {
    input_sizes_idx_0 = 0;
  }

  varargout_1_size[0] = input_sizes_idx_0;
  varargout_1_size_tmp = sizes_idx_1_tmp;
  varargout_1_size[1] = sizes_idx_1_tmp;
  for (loop_ub = 0; loop_ub < varargout_1_size_tmp; loop_ub++) {
    if (input_sizes_idx_0 - 1 >= 0) {
      std::memcpy(&varargout_1_data[0], &varargin_1_data[0],
                  static_cast<uint32_T>(input_sizes_idx_0) * sizeof(real_T));
    }
  }

  if ((b_varargin_1_size[0] != 0) && (b_varargin_1_size[1] != 0)) {
    sizes_idx_0 = static_cast<uint8_T>(b_varargin_1_size[0]);
  } else {
    sizes_idx_0 = 0U;
  }

  varargout_3_size[0] = sizes_idx_0;
  varargout_3_size[1] = b_varargin_1_size[1];
  loop_ub = sizes_idx_0 * b_varargin_1_size[1];
  if (loop_ub - 1 >= 0) {
    std::memcpy(&varargout_3_data[0], &root_model_B.b_varargin_1_data[0],
                static_cast<uint32_T>(loop_ub) * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factoryConstruct(int32_T nVarMax, int32_T mConstrMax,
  int32_T mIneq, int32_T mNonlinIneq, s_7iu1rzNoAoHigERqeOFm2G_root_T *obj)
{
  obj->nVarMax = nVarMax;
  obj->mNonlinIneq = mNonlinIneq;
  obj->mNonlinEq = 170;
  obj->mIneq = mIneq;
  obj->mEq = 170;
  obj->iNonIneq0 = (mIneq - mNonlinIneq) + 1;
  obj->iNonEq0 = 1;
  obj->sqpFval = 0.0;
  obj->sqpFval_old = 0.0;
  obj->cIneq.size[0] = mIneq;
  obj->cIneq_old.size[0] = mIneq;
  obj->grad.size[0] = nVarMax;
  obj->grad_old.size[0] = nVarMax;
  obj->FunctionEvaluations = 0;
  obj->sqpIterations = 0;
  obj->sqpExitFlag = 0;
  obj->lambdasqp.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    std::memset(&obj->lambdasqp.data[0], 0, static_cast<uint32_T>(mConstrMax) *
                sizeof(real_T));
  }

  obj->lambdaStopTest.size[0] = mConstrMax;
  obj->lambdaStopTestPrev.size[0] = mConstrMax;
  obj->steplength = 1.0;
  obj->delta_x.size[0] = nVarMax;
  if (nVarMax - 1 >= 0) {
    std::memset(&obj->delta_x.data[0], 0, static_cast<uint32_T>(nVarMax) *
                sizeof(real_T));
  }

  obj->socDirection.size[0] = nVarMax;
  obj->workingset_old.size[0] = mConstrMax;
  if (mNonlinIneq > 0) {
    obj->JacCineqTrans_old.size[0] = nVarMax;
    obj->JacCineqTrans_old.size[1] = mNonlinIneq;
  } else {
    obj->JacCineqTrans_old.size[0] = 0;
    obj->JacCineqTrans_old.size[1] = 0;
  }

  obj->JacCeqTrans_old.size[0] = nVarMax;
  obj->JacCeqTrans_old.size[1] = 170;
  obj->gradLag.size[0] = nVarMax;
  obj->delta_gradLag.size[0] = nVarMax;
  obj->xstar.size[0] = nVarMax;
  obj->fstar = 0.0;
  obj->firstorderopt = 0.0;
  obj->lambda.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    std::memset(&obj->lambda.data[0], 0, static_cast<uint32_T>(mConstrMax) *
                sizeof(real_T));
  }

  obj->state = 0;
  obj->maxConstr = 0.0;
  obj->iterations = 0;
  obj->searchDir.size[0] = nVarMax;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mode_factoryConstruct_n0cp(int32_T MaxVars, int32_T
  obj_grad_size[1], int32_T obj_Hx_size[1], boolean_T *obj_hasLinear, int32_T
  *obj_nvar, int32_T *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T
  *obj_objtype, int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
  *obj_prev_hasLinear, real_T *obj_gammaScalar)
{
  obj_grad_size[0] = MaxVars;
  obj_Hx_size[0] = MaxVars - 1;
  *obj_hasLinear = false;
  *obj_nvar = 0;
  *obj_maxVar = MaxVars;
  *obj_beta = 0.0;
  *obj_rho = 0.0;
  *obj_objtype = 3;
  *obj_prev_objtype = 3;
  *obj_prev_nvar = 0;
  *obj_prev_hasLinear = false;
  *obj_gammaScalar = 0.0;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mod_factoryConstruct_n0cp3(int32_T mIneqMax, int32_T nVarMax,
  int32_T mConstrMax, s_Jrk1gF3jitJmkPBgphEknE_root_T *obj)
{
  obj->mConstr = 0;
  obj->mConstrOrig = 0;
  obj->mConstrMax = mConstrMax;
  obj->nVar = 177;
  obj->nVarOrig = 177;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  obj->Aineq.size[0] = mIneqMax * nVarMax;
  obj->bineq.size[0] = mIneqMax;
  obj->Aeq.size[0] = 170 * nVarMax;
  obj->lb.size[0] = nVarMax;
  obj->ub.size[0] = nVarMax;
  obj->indexLB.size[0] = nVarMax;
  obj->indexUB.size[0] = nVarMax;
  obj->indexFixed.size[0] = nVarMax;
  obj->mEqRemoved = 0;
  obj->ATwset.size[0] = nVarMax * mConstrMax;
  obj->bwset.size[0] = mConstrMax;
  obj->nActiveConstr = 0;
  obj->maxConstrWorkspace.size[0] = mConstrMax;
  for (int32_T i{0}; i < 5; i++) {
    obj->sizes[i] = 0;
    obj->sizesNormal[i] = 0;
    obj->sizesPhaseOne[i] = 0;
    obj->sizesRegularized[i] = 0;
    obj->sizesRegPhaseOne[i] = 0;
  }

  for (int32_T i{0}; i < 6; i++) {
    obj->isActiveIdx[i] = 0;
    obj->isActiveIdxNormal[i] = 0;
    obj->isActiveIdxPhaseOne[i] = 0;
    obj->isActiveIdxRegularized[i] = 0;
    obj->isActiveIdxRegPhaseOne[i] = 0;
  }

  obj->isActiveConstr.size[0] = mConstrMax;
  obj->Wid.size[0] = mConstrMax;
  obj->Wlocalidx.size[0] = mConstrMax;
  for (int32_T i{0}; i < 5; i++) {
    obj->nWConstr[i] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void computeObjectiveAndUserGradient(const
  s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next_ne, const real_T
  x[177], real_T grad_workspace_data[], real_T *fval, int32_T *status)
{
  __m128d tmp;
  __m128d tmp_1;
  real_T Ck[221];
  real_T X[187];
  real_T b_X[187];
  real_T b_x[177];
  real_T gfX[170];
  real_T U[33];
  real_T b_U[33];
  real_T gfU[30];
  real_T ix[17];
  real_T x0[17];
  real_T xa[17];
  real_T obj_next_next_next_next_next__0[13];
  real_T c[6];
  real_T tmp_0[2];
  real_T tmp_2[2];
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T duk_idx_2;
  real_T dx;
  real_T e;
  real_T fs;
  real_T gfU_idx_0;
  real_T gfU_idx_1;
  real_T iu_idx_0;
  real_T iu_idx_1;
  real_T iu_idx_2;
  real_T obj_next_next_next_next_next__1;
  real_T umvk;
  real_T umvk_idx_1;
  real_T umvk_idx_2;
  int32_T b_k;
  int32_T i;
  int32_T idx_current;
  boolean_T allFinite;
  static const int8_T c_0[180]{ 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1 };

  getXUe_FXX9Wcuz(x, obj_next_next_next_next_next_ne->runtimedata.x, X, U, &e);
  std::memset(&gfX[0], 0, 170U * sizeof(real_T));
  std::memset(&gfU[0], 0, 30U * sizeof(real_T));
  fs = 0.0;
  for (i = 0; i <= 14; i += 2) {
    tmp_0[0] = i;
    tmp_0[1] = i + 1;
    tmp_1 = _mm_loadu_pd(&tmp_0[0]);
    _mm_storeu_pd(&ix[i], _mm_add_pd(tmp_1, _mm_set1_pd(1.0)));
  }

  for (i = 16; i < 17; i++) {
    ix[i] = static_cast<real_T>(i) + 1.0;
  }

  iu_idx_0 = 1.0;
  iu_idx_1 = 2.0;
  iu_idx_2 = 3.0;
  for (i = 0; i < 11; i++) {
    for (b_k = 0; b_k < 17; b_k++) {
      b_X[b_k + 17 * i] = X[11 * b_k + i];
    }

    b_U[3 * i] = U[i];
    b_U[3 * i + 1] = U[i + 11];
    b_U[3 * i + 2] = U[i + 22];
  }

  for (idx_current = 0; idx_current < 10; idx_current++) {
    for (b_k = 0; b_k < 17; b_k++) {
      dx = b_X[(idx_current + 1) * 17 + b_k];
      x0[b_k] = dx;
      dx = std::abs(dx);
      xa[b_k] = dx;
      if (dx < 1.0) {
        xa[b_k] = 1.0;
      }
    }

    for (b_k = 0; b_k < 17; b_k++) {
      dx = 1.0E-6 * xa[b_k];
      x0[b_k] += dx;
      for (i = 0; i <= 10; i += 2) {
        tmp_1 = _mm_loadu_pd(&x0[i]);
        tmp = _mm_loadu_pd(&b_X[(idx_current + 1) * 17 + i]);
        _mm_storeu_pd(&Ck[i + 13 * b_k], _mm_div_pd(_mm_sub_pd(tmp_1, tmp),
          _mm_set1_pd(dx)));
      }

      for (i = 12; i < 13; i++) {
        Ck[i + 13 * b_k] = (x0[i] - b_X[(idx_current + 1) * 17 + i]) / dx;
      }

      x0[b_k] -= dx;
    }

    dx = 0.0;
    for (i = 0; i < 13; i++) {
      b_k = 10 * i + idx_current;
      umvk_idx_1 = obj_next_next_next_next_next_ne->
        runtimedata.OutputWeights[b_k];
      duk_idx_1 = (b_X[(idx_current + 1) * 17 + i] -
                   obj_next_next_next_next_next_ne->runtimedata.ref[b_k]) *
        umvk_idx_1;
      dx += duk_idx_1 * duk_idx_1;
      obj_next_next_next_next_next__0[i] = umvk_idx_1 * duk_idx_1;
    }

    fs += dx;
    for (i = 0; i < 17; i++) {
      dx = 0.0;
      for (b_k = 0; b_k < 13; b_k++) {
        dx += Ck[13 * i + b_k] * obj_next_next_next_next_next__0[b_k];
      }

      x0[i] = gfX[static_cast<int32_T>(ix[i]) - 1] + dx;
    }

    for (i = 0; i < 17; i++) {
      dx = ix[i];
      gfX[static_cast<int32_T>(dx) - 1] = x0[i];
      ix[i] = dx + 17.0;
    }

    dx = b_U[3 * idx_current];
    umvk_idx_1 = b_U[3 * idx_current + 1];
    umvk_idx_2 = b_U[3 * idx_current + 2];
    if (idx_current == 0) {
      _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(umvk_idx_1, dx),
        _mm_loadu_pd(&obj_next_next_next_next_next_ne->runtimedata.lastMV[0])));
      duk_idx_0 = tmp_2[0];
      duk_idx_1 = tmp_2[1];
      duk_idx_2 = umvk_idx_2 -
        obj_next_next_next_next_next_ne->runtimedata.lastMV[2];
    } else {
      i = (idx_current - 1) * 3;
      tmp_1 = _mm_sub_pd(_mm_set_pd(umvk_idx_1, dx), _mm_loadu_pd(&b_U[i]));
      _mm_storeu_pd(&tmp_2[0], tmp_1);
      duk_idx_0 = tmp_2[0];
      duk_idx_1 = tmp_2[1];
      duk_idx_2 = umvk_idx_2 - b_U[i + 2];
    }

    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVWeights[idx_current];
    umvk = (dx - obj_next_next_next_next_next_ne->
            runtimedata.MVScaledTarget[idx_current]) *
      obj_next_next_next_next_next__1;
    dx = umvk;
    gfU_idx_0 = gfU[static_cast<int32_T>(iu_idx_0) - 1] +
      obj_next_next_next_next_next__1 * umvk;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVWeights[idx_current + 10];
    umvk = (umvk_idx_1 -
            obj_next_next_next_next_next_ne->
            runtimedata.MVScaledTarget[idx_current + 10]) *
      obj_next_next_next_next_next__1;
    umvk_idx_1 = umvk;
    i = static_cast<int32_T>(iu_idx_1) - 1;
    gfU_idx_1 = gfU[static_cast<int32_T>(iu_idx_1) - 1] +
      obj_next_next_next_next_next__1 * umvk;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVWeights[idx_current + 20];
    umvk = (umvk_idx_2 -
            obj_next_next_next_next_next_ne->
            runtimedata.MVScaledTarget[idx_current + 20]) *
      obj_next_next_next_next_next__1;
    b_k = static_cast<int32_T>(iu_idx_2) - 1;
    umvk_idx_2 = gfU[static_cast<int32_T>(iu_idx_2) - 1] +
      obj_next_next_next_next_next__1 * umvk;
    gfU[static_cast<int32_T>(iu_idx_0) - 1] = gfU_idx_0;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVRateWeights[idx_current];
    duk_idx_0 *= obj_next_next_next_next_next__1;
    gfU_idx_0 = dx * dx;
    duk = duk_idx_0 * duk_idx_0;
    dx = obj_next_next_next_next_next__1 * duk_idx_0;
    gfU[static_cast<int32_T>(iu_idx_1) - 1] = gfU_idx_1;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVRateWeights[idx_current +
      10];
    duk_idx_0 = obj_next_next_next_next_next__1 * duk_idx_1;
    gfU_idx_0 += umvk_idx_1 * umvk_idx_1;
    duk += duk_idx_0 * duk_idx_0;
    umvk_idx_1 = obj_next_next_next_next_next__1 * duk_idx_0;
    gfU[static_cast<int32_T>(iu_idx_2) - 1] = umvk_idx_2;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVRateWeights[idx_current +
      20];
    duk_idx_0 = obj_next_next_next_next_next__1 * duk_idx_2;
    fs = ((umvk * umvk + gfU_idx_0) + fs) + (duk_idx_0 * duk_idx_0 + duk);
    tmp_1 = _mm_set_pd(obj_next_next_next_next_next__1 * duk_idx_0, umvk_idx_1);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(gfU[b_k], gfU[i]), tmp_1));
    gfU[static_cast<int32_T>(iu_idx_0) - 1] += dx;
    gfU[static_cast<int32_T>(iu_idx_1) - 1] = tmp_2[0];
    gfU[static_cast<int32_T>(iu_idx_2) - 1] = tmp_2[1];
    if (idx_current > 0) {
      i = static_cast<int32_T>(iu_idx_1 - 3.0) - 1;
      b_k = static_cast<int32_T>(iu_idx_2 - 3.0) - 1;
      _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(gfU[b_k], gfU[i]), tmp_1));
      gfU[static_cast<int32_T>(iu_idx_0 - 3.0) - 1] -= dx;
      gfU[static_cast<int32_T>(iu_idx_1 - 3.0) - 1] = tmp_2[0];
      gfU[static_cast<int32_T>(iu_idx_2 - 3.0) - 1] = tmp_2[1];
    }

    iu_idx_0 += 3.0;
    iu_idx_1 += 3.0;
    iu_idx_2 += 3.0;
  }

  *fval = 100000.0 * e * e + fs;
  for (i = 0; i < 6; i++) {
    dx = 0.0;
    for (b_k = 0; b_k < 30; b_k++) {
      dx += static_cast<real_T>(c_0[6 * b_k + i]) * (2.0 * gfU[b_k]);
    }

    c[i] = dx;
  }

  for (i = 0; i <= 168; i += 2) {
    tmp_1 = _mm_loadu_pd(&gfX[i]);
    _mm_storeu_pd(&b_x[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_1));
  }

  for (i = 0; i < 6; i++) {
    b_x[i + 170] = c[i];
  }

  b_x[176] = 200000.0 * e;
  std::memcpy(&grad_workspace_data[0], &b_x[0], 177U * sizeof(real_T));
  *status = 1;
  allFinite = std::isnan(*fval);
  if (std::isinf(*fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    allFinite = true;
    idx_current = -1;
    while (allFinite && (idx_current + 2 <= 177)) {
      allFinite = ((!std::isinf(grad_workspace_data[idx_current + 1])) && (!std::
        isnan(grad_workspace_data[idx_current + 1])));
      idx_current++;
    }

    if (!allFinite) {
      if (std::isnan(grad_workspace_data[idx_current])) {
        *status = -3;
      } else if (grad_workspace_data[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static int32_T root_model_checkVectorNonFinite(int32_T N, const real_T vec_data[],
  int32_T iv0)
{
  int32_T idx_current;
  int32_T idx_end;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0 - 2;
  idx_end = (iv0 + N) - 1;
  while (allFinite && (idx_current + 2 <= idx_end)) {
    real_T allFinite_tmp;
    allFinite_tmp = vec_data[idx_current + 1];
    allFinite = ((!std::isinf(allFinite_tmp)) && (!std::isnan(allFinite_tmp)));
    idx_current++;
  }

  if (!allFinite) {
    if (std::isnan(vec_data[idx_current])) {
      status = -3;
    } else if (vec_data[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }

  return status;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static int32_T computeConstraintsAndUserJacobi(int32_T
  obj_next_next_next_next_next_b_, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *obj_next_next_next_next_next_ne, const real_T x[177], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170], real_T
  JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
  JacEqTrans_workspace_data[], int32_T ldJE)
{
  real_T a__3_data[260];
  real_T b_x[170];
  int32_T a__3_size[2];
  int32_T a__4_size[2];
  int32_T col;
  int32_T col_end;
  int32_T idx_mat;
  int32_T row;
  int32_T status;
  boolean_T allFinite;
  if (obj_next_next_next_next_next_b_ > 0) {
    root_model_c4_mpclib_anonFcn2(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax,
      obj_next_next_next_next_next_ne->Parameters.f1,
      obj_next_next_next_next_next_ne->Parameters.f2,
      obj_next_next_next_next_next_ne->Parameters.f3,
      obj_next_next_next_next_next_ne->Parameters.f4,
      obj_next_next_next_next_next_ne->Parameters.f5, x, a__3_data, a__3_size,
      b_x, root_model_B.a__4_data, a__4_size, root_model_B.JacEqTrans_tmp);
    col = static_cast<uint16_T>(obj_next_next_next_next_next_b_);
    for (row = 0; row < col; row++) {
      Cineq_workspace_data[(ineq0 + row) - 1] = a__3_data[row];
    }

    std::memcpy(&Ceq_workspace[0], &b_x[0], 170U * sizeof(real_T));
    col_end = a__4_size[0];
    for (row = 0; row < col_end; row++) {
      idx_mat = a__4_size[1];
      for (col = 0; col < idx_mat; col++) {
        JacIneqTrans_workspace_data[row + ldJI * ((iJI_col + col) - 1)] =
          root_model_B.a__4_data[a__4_size[0] * col + row];
      }
    }

    for (row = 0; row < 177; row++) {
      for (col = 0; col < 170; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          root_model_B.JacEqTrans_tmp[177 * col + row];
      }
    }
  } else {
    root_model_c4_mpclib_anonFcn2(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax,
      obj_next_next_next_next_next_ne->Parameters.f1,
      obj_next_next_next_next_next_ne->Parameters.f2,
      obj_next_next_next_next_next_ne->Parameters.f3,
      obj_next_next_next_next_next_ne->Parameters.f4,
      obj_next_next_next_next_next_ne->Parameters.f5, x, a__3_data, a__3_size,
      b_x, root_model_B.a__4_data, a__4_size, root_model_B.JacEqTrans_tmp);
    std::memcpy(&Ceq_workspace[0], &b_x[0], 170U * sizeof(real_T));
    for (row = 0; row < 177; row++) {
      for (col = 0; col < 170; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          root_model_B.JacEqTrans_tmp[177 * col + row];
      }
    }
  }

  status = root_model_checkVectorNonFinite(obj_next_next_next_next_next_b_,
    Cineq_workspace_data, ineq0);
  if (status == 1) {
    status = checkVectorNonFinite_yVQywJ02(Ceq_workspace);
    if (status == 1) {
      allFinite = true;
      row = -1;
      col = iJI_col;
      col_end = (iJI_col + obj_next_next_next_next_next_b_) - 1;
      while (allFinite && (col <= col_end)) {
        row = -1;
        while (allFinite && (row + 2 <= 177)) {
          idx_mat = ((col - 1) * ldJI + row) + 1;
          allFinite = ((!std::isinf(JacIneqTrans_workspace_data[idx_mat])) &&
                       (!std::isnan(JacIneqTrans_workspace_data[idx_mat])));
          row++;
        }

        col++;
      }

      if (!allFinite) {
        idx_mat = (col - 2) * ldJI + row;
        if (std::isnan(JacIneqTrans_workspace_data[idx_mat])) {
          status = -3;
        } else if (JacIneqTrans_workspace_data[idx_mat] < 0.0) {
          status = -1;
        } else {
          status = -2;
        }
      } else {
        row = -1;
        col = -1;
        while (allFinite && (col + 2 <= 170)) {
          row = -1;
          while (allFinite && (row + 2 <= 177)) {
            col_end = ((col + 1) * ldJE + row) + 1;
            allFinite = ((!std::isinf(JacEqTrans_workspace_data[col_end])) &&
                         (!std::isnan(JacEqTrans_workspace_data[col_end])));
            row++;
          }

          col++;
        }

        if (!allFinite) {
          col_end = ldJE * col + row;
          if (std::isnan(JacEqTrans_workspace_data[col_end])) {
            status = -3;
          } else if (JacEqTrans_workspace_data[col_end] < 0.0) {
            status = -1;
          } else {
            status = -2;
          }
        }
      }
    }
  }

  return status;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void evalObjAndConstrAndDerivatives(int32_T
  obj_next_next_next_next_next_b_, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *obj_next_next_next_next_next_ne, const s_pVH6DktXFODPXN9VaZK2hE_root_T
  *obj_next_next_next_next_next__0, const real_T x[177], real_T
  grad_workspace_data[], real_T Cineq_workspace_data[], int32_T ineq0, real_T
  Ceq_workspace[170], real_T JacIneqTrans_workspace_data[], int32_T iJI_col,
  int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T ldJE, real_T *fval,
  int32_T *status)
{
  computeObjectiveAndUserGradient(obj_next_next_next_next_next__0, x,
    grad_workspace_data, fval, status);
  if (*status == 1) {
    *status = computeConstraintsAndUserJacobi(obj_next_next_next_next_next_b_,
      obj_next_next_next_next_next_ne, x, Cineq_workspace_data, ineq0,
      Ceq_workspace, JacIneqTrans_workspace_data, iJI_col, ldJI,
      JacEqTrans_workspace_data, ldJE);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mo_modifyOverheadPhaseOne_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj)
{
  int32_T d;
  int32_T idxEq;
  idxEq = static_cast<uint16_T>(obj->sizes[0]);
  for (int32_T idx{0}; idx < idxEq; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
  }

  for (int32_T idx{0}; idx < 170; idx++) {
    idxEq = (obj->ldA * idx + obj->nVar) - 1;
    obj->Aeq.data[idxEq] = 0.0;
    obj->ATwset.data[idxEq + obj->ldA * (obj->isActiveIdx[1] - 1)] = 0.0;
  }

  idxEq = static_cast<uint16_T>(obj->sizes[2]);
  for (int32_T idx{0}; idx < idxEq; idx++) {
    obj->Aineq.data[(obj->nVar + obj->ldA * idx) - 1] = -1.0;
  }

  obj->indexLB.data[obj->sizes[3] - 1] = obj->nVar;
  obj->lb.data[obj->nVar - 1] = 1.0E-5;
  idxEq = obj->isActiveIdx[2];
  d = obj->nActiveConstr;
  for (int32_T idx{idxEq}; idx <= d; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
  }

  idxEq = obj->isActiveIdx[4] - 1;
  if (obj->nWConstr[4] > 0) {
    d = obj->sizesNormal[4];
    for (int32_T idx{d}; idx >= 1; idx--) {
      int32_T tmp;
      tmp = idxEq + idx;
      obj->isActiveConstr.data[tmp] = obj->isActiveConstr.data[tmp - 1];
    }
  } else {
    obj->isActiveConstr.data[(obj->isActiveIdx[4] + obj->sizesNormal[4]) - 1] =
      false;
  }

  obj->isActiveConstr.data[obj->isActiveIdx[4] - 1] = false;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_setProblemType(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T PROBLEM_TYPE)
{
  int32_T colOffsetATw;
  int32_T colOffsetAineq;
  int32_T d;
  int32_T idxUpperExisting;
  int32_T idx_col;
  int32_T k;
  int32_T offsetEq1;
  int32_T offsetEq2;
  int32_T tmp;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 177;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      idxUpperExisting = obj->isActiveIdx[4] - 1;
      offsetEq1 = static_cast<uint16_T>(obj->sizesNormal[4]);
      for (colOffsetATw = 0; colOffsetATw < offsetEq1; colOffsetATw++) {
        k = idxUpperExisting + colOffsetATw;
        obj->isActiveConstr.data[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1]
          = obj->isActiveConstr.data[k];
        obj->isActiveConstr.data[k] = false;
      }
    }

    for (k = 0; k < 5; k++) {
      obj->sizes[k] = obj->sizesNormal[k];
    }

    for (k = 0; k < 6; k++) {
      obj->isActiveIdx[k] = obj->isActiveIdxNormal[k];
    }
    break;

   case 1:
    obj->nVar = 178;
    obj->mConstr = obj->mConstrOrig + 1;
    for (k = 0; k < 5; k++) {
      obj->sizes[k] = obj->sizesPhaseOne[k];
    }

    root_mo_modifyOverheadPhaseOne_(obj);
    for (k = 0; k < 6; k++) {
      obj->isActiveIdx[k] = obj->isActiveIdxPhaseOne[k];
    }
    break;

   case 2:
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (k = 0; k < 5; k++) {
      obj->sizes[k] = obj->sizesRegularized[k];
    }

    if (obj->probType != 4) {
      idxUpperExisting = obj->sizes[2] + 347;
      offsetEq1 = obj->sizes[2] + 177;
      offsetEq2 = obj->sizes[2] + 347;
      k = static_cast<uint16_T>(obj->sizes[0]);
      for (idx_col = 0; idx_col < k; idx_col++) {
        std::memset(&obj->ATwset.data[obj->ldA * idx_col + 177], 0,
                    static_cast<uint32_T>(obj->nVar - 177) * sizeof(real_T));
      }

      idx_col = static_cast<uint16_T>(obj->sizes[2]);
      for (colOffsetATw = 0; colOffsetATw < idx_col; colOffsetATw++) {
        colOffsetAineq = obj->ldA * colOffsetATw - 1;
        for (k = 178; k <= colOffsetATw + 177; k++) {
          obj->Aineq.data[k + colOffsetAineq] = 0.0;
        }

        obj->Aineq.data[(colOffsetATw + colOffsetAineq) + 178] = -1.0;
        d = obj->nVar;
        for (k = colOffsetATw + 179; k <= d; k++) {
          obj->Aineq.data[k + colOffsetAineq] = 0.0;
        }
      }

      for (idx_col = 0; idx_col < 170; idx_col++) {
        colOffsetAineq = obj->ldA * idx_col - 1;
        colOffsetATw = (obj->isActiveIdx[1] - 1) * obj->ldA + colOffsetAineq;
        if (offsetEq1 >= 178) {
          std::memset(&obj->Aeq.data[colOffsetAineq + 178], 0,
                      static_cast<uint32_T>(offsetEq1 - 177) * sizeof(real_T));
        }

        if (offsetEq1 >= 178) {
          std::memset(&obj->ATwset.data[colOffsetATw + 178], 0,
                      static_cast<uint32_T>(offsetEq1 - 177) * sizeof(real_T));
        }

        d = idxUpperExisting + idx_col;
        if (idxUpperExisting - 169 <= d - 170) {
          std::memset(&obj->Aeq.data[(idxUpperExisting + colOffsetAineq) + -169],
                      0, static_cast<uint32_T>(((((d - 170) + colOffsetAineq) -
            idxUpperExisting) - colOffsetAineq) + 170) * sizeof(real_T));
        }

        if (idxUpperExisting - 169 <= d - 170) {
          std::memset(&obj->ATwset.data[(idxUpperExisting + colOffsetATw) + -169],
                      0, static_cast<uint32_T>(((((d - 170) + colOffsetATw) -
            idxUpperExisting) - colOffsetATw) + 170) * sizeof(real_T));
        }

        k = d + colOffsetAineq;
        obj->Aeq.data[k - 169] = -1.0;
        tmp = d + colOffsetATw;
        obj->ATwset.data[tmp - 169] = -1.0;
        if (d - 168 <= offsetEq2) {
          std::memset(&obj->Aeq.data[(d - 168) + colOffsetAineq], 0,
                      static_cast<uint32_T>((((offsetEq2 + colOffsetAineq) - (d
            - 168)) - colOffsetAineq) + 1) * sizeof(real_T));
        }

        if (d - 168 <= offsetEq2) {
          std::memset(&obj->ATwset.data[(d - 168) + colOffsetATw], 0,
                      static_cast<uint32_T>((((offsetEq2 + colOffsetATw) - (d -
            168)) - colOffsetATw) + 1) * sizeof(real_T));
        }

        if (idxUpperExisting + 1 <= d) {
          std::memset(&obj->Aeq.data[(idxUpperExisting + colOffsetAineq) + 1], 0,
                      static_cast<uint32_T>(((d + colOffsetAineq) -
            idxUpperExisting) - colOffsetAineq) * sizeof(real_T));
        }

        if (idxUpperExisting + 1 <= d) {
          std::memset(&obj->ATwset.data[(idxUpperExisting + colOffsetATw) + 1],
                      0, static_cast<uint32_T>(((d + colOffsetATw) -
            idxUpperExisting) - colOffsetATw) * sizeof(real_T));
        }

        obj->Aeq.data[k + 1] = 1.0;
        obj->ATwset.data[tmp + 1] = 1.0;
        if (d + 2 <= obj->nVar) {
          std::memset(&obj->Aeq.data[(d + 2) + colOffsetAineq], 0,
                      static_cast<uint32_T>((((obj->nVar + colOffsetAineq) - (d
            + 2)) - colOffsetAineq) + 1) * sizeof(real_T));
        }

        if (d + 2 <= obj->nVar) {
          std::memset(&obj->ATwset.data[(d + 2) + colOffsetATw], 0, static_cast<
                      uint32_T>((((obj->nVar + colOffsetATw) - (d + 2)) -
            colOffsetATw) + 1) * sizeof(real_T));
        }
      }

      idxUpperExisting = 177;
      offsetEq1 = obj->sizesNormal[3] + 1;
      offsetEq2 = obj->sizesRegularized[3];
      for (colOffsetATw = offsetEq1; colOffsetATw <= offsetEq2; colOffsetATw++)
      {
        idxUpperExisting++;
        obj->indexLB.data[colOffsetATw - 1] = idxUpperExisting;
      }

      if (obj->nWConstr[4] > 0) {
        idxUpperExisting = static_cast<uint16_T>(obj->sizesRegularized[4]);
        for (colOffsetATw = 0; colOffsetATw < idxUpperExisting; colOffsetATw++)
        {
          obj->isActiveConstr.data[obj->isActiveIdxRegularized[4] + colOffsetATw]
            = obj->isActiveConstr.data[(obj->isActiveIdx[4] + colOffsetATw) - 1];
        }
      }

      idxUpperExisting = obj->isActiveIdx[4];
      offsetEq1 = obj->isActiveIdxRegularized[4];
      if (idxUpperExisting <= offsetEq1 - 1) {
        std::memset(&obj->isActiveConstr.data[idxUpperExisting + -1], 0,
                    static_cast<uint32_T>(offsetEq1 - idxUpperExisting) * sizeof
                    (boolean_T));
      }

      idxUpperExisting = obj->sizes[2] + 517;
      std::memset(&obj->lb.data[177], 0, static_cast<uint32_T>(idxUpperExisting
        - 177) * sizeof(real_T));
      offsetEq1 = obj->isActiveIdx[2];
      offsetEq2 = obj->nActiveConstr;
      for (idxUpperExisting = offsetEq1; idxUpperExisting <= offsetEq2;
           idxUpperExisting++) {
        colOffsetATw = (idxUpperExisting - 1) * obj->ldA - 1;
        if (obj->Wid.data[idxUpperExisting - 1] == 3) {
          colOffsetAineq = obj->Wlocalidx.data[idxUpperExisting - 1] + 176;
          if (colOffsetAineq >= 178) {
            std::memset(&obj->ATwset.data[colOffsetATw + 178], 0,
                        static_cast<uint32_T>(colOffsetAineq - 177) * sizeof
                        (real_T));
          }

          obj->ATwset.data[(obj->Wlocalidx.data[idxUpperExisting - 1] +
                            colOffsetATw) + 177] = -1.0;
          colOffsetAineq = obj->Wlocalidx.data[idxUpperExisting - 1] + 178;
          if (colOffsetAineq <= obj->nVar) {
            std::memset(&obj->ATwset.data[colOffsetAineq + colOffsetATw], 0,
                        static_cast<uint32_T>((((obj->nVar + colOffsetATw) -
              colOffsetAineq) - colOffsetATw) + 1) * sizeof(real_T));
          }
        } else {
          std::memset(&obj->ATwset.data[colOffsetATw + 178], 0,
                      static_cast<uint32_T>(obj->nVar - 177) * sizeof(real_T));
        }
      }
    }

    for (k = 0; k < 6; k++) {
      obj->isActiveIdx[k] = obj->isActiveIdxRegularized[k];
    }
    break;

   default:
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (k = 0; k < 5; k++) {
      obj->sizes[k] = obj->sizesRegPhaseOne[k];
    }

    root_mo_modifyOverheadPhaseOne_(obj);
    for (k = 0; k < 6; k++) {
      obj->isActiveIdx[k] = obj->isActiveIdxRegPhaseOne[k];
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_initActiveSet(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj)
{
  int32_T colOffsetATw;
  int32_T f;
  int32_T iATw0;
  int32_T iAeq0;
  int32_T idx;
  int32_T idxFillStart;
  root_model_setProblemType(obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  if (idxFillStart <= obj->mConstrMax) {
    std::memset(&obj->isActiveConstr.data[idxFillStart + -1], 0,
                static_cast<uint32_T>((obj->mConstrMax - idxFillStart) + 1) *
                sizeof(boolean_T));
  }

  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 170;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + 170;
  idxFillStart = static_cast<uint16_T>(obj->sizes[0]);
  for (idx = 0; idx < idxFillStart; idx++) {
    obj->Wid.data[idx] = 1;
    obj->Wlocalidx.data[idx] = idx + 1;
    obj->isActiveConstr.data[idx] = true;
    colOffsetATw = obj->ldA * idx;
    iATw0 = static_cast<uint8_T>(obj->indexFixed.data[idx] - 1);
    if (iATw0 - 1 >= 0) {
      std::memset(&obj->ATwset.data[colOffsetATw], 0, static_cast<uint32_T>
                  (iATw0) * sizeof(real_T));
    }

    obj->ATwset.data[(obj->indexFixed.data[idx] + colOffsetATw) - 1] = 1.0;
    iATw0 = obj->indexFixed.data[idx] + 1;
    if (iATw0 <= obj->nVar) {
      std::memset(&obj->ATwset.data[(iATw0 + colOffsetATw) + -1], 0,
                  static_cast<uint32_T>((((obj->nVar + colOffsetATw) - iATw0) -
        colOffsetATw) + 1) * sizeof(real_T));
    }

    obj->bwset.data[idx] = obj->ub.data[obj->indexFixed.data[idx] - 1];
  }

  for (idx = 0; idx < 170; idx++) {
    colOffsetATw = obj->sizes[0] + idx;
    obj->Wid.data[colOffsetATw] = 2;
    obj->Wlocalidx.data[colOffsetATw] = idx + 1;
    obj->isActiveConstr.data[colOffsetATw] = true;
    iAeq0 = obj->ldA * idx;
    iATw0 = obj->ldA * colOffsetATw;
    f = obj->nVar;
    for (idxFillStart = 0; idxFillStart < f; idxFillStart++) {
      obj->ATwset.data[iATw0 + idxFillStart] = obj->Aeq.data[iAeq0 +
        idxFillStart];
    }

    obj->bwset.data[colOffsetATw] = obj->beq[idx];
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factoryConstruct_n0(int32_T maxRows, int32_T maxCols,
  int32_T *obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T
  obj_Q_size[2], int32_T obj_jpvt_data[], int32_T obj_jpvt_size[1], int32_T
  *obj_mrows, int32_T *obj_ncols, int32_T obj_tau_size[1], int32_T
  *obj_minRowCol, boolean_T *obj_usedPivoting)
{
  int32_T loop_ub;
  *obj_ldq = maxRows;
  obj_QR_size[0] = maxRows;
  obj_QR_size[1] = maxCols;
  obj_Q_size[0] = maxRows;
  obj_Q_size[1] = maxRows;
  loop_ub = maxRows * maxRows;
  if (loop_ub - 1 >= 0) {
    std::memset(&obj_Q_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof
                (real_T));
  }

  obj_jpvt_size[0] = maxCols;
  if (maxCols - 1 >= 0) {
    std::memset(&obj_jpvt_data[0], 0, static_cast<uint32_T>(maxCols) * sizeof
                (int32_T));
  }

  *obj_mrows = 0;
  *obj_ncols = 0;
  if (maxRows <= maxCols) {
    obj_tau_size[0] = maxRows;
  } else {
    obj_tau_size[0] = maxCols;
  }

  *obj_minRowCol = 0;
  *obj_usedPivoting = false;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factoryConstruct_n0c(int32_T MaxDims, int32_T
  obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info,
  real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_,
  real_T *obj_workspace_, real_T *obj_workspace2_)
{
  obj_FMat_size[0] = MaxDims;
  obj_FMat_size[1] = MaxDims;
  *obj_ldm = MaxDims;
  *obj_ndims = 0;
  *obj_info = 0;
  *obj_scaleFactor = 0.0;
  *obj_ConvexCheck = true;
  *obj_regTol_ = (rtInf);
  *obj_workspace_ = (rtInf);
  *obj_workspace2_ = (rtInf);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_computeGradLag(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[])
{
  int32_T b;
  int32_T f;
  int32_T g;
  int32_T iL0;
  int32_T ix;
  int32_T tmp;
  std::memcpy(&workspace_data[0], &grad_data[0], static_cast<uint16_T>(nVar) *
              sizeof(real_T));
  b = static_cast<uint16_T>(mFixed);
  for (iL0 = 0; iL0 < b; iL0++) {
    ix = finiteFixed_data[iL0];
    workspace_data[ix - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 169 + 1;
  for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
    g = iL0 + nVar;
    for (b = iL0; b < g; b++) {
      tmp = b - iL0;
      workspace_data[tmp] += AeqTrans_data[b - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 170;
    f = (mIneq - 1) * ldA + 1;
    for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
      g = iL0 + nVar;
      for (b = iL0; b < g; b++) {
        tmp = b - iL0;
        workspace_data[tmp] += AineqTrans_data[b - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 170;
  ix = static_cast<uint16_T>(mLB) - 1;
  for (b = 0; b <= ix; b++) {
    f = finiteLB_data[b];
    workspace_data[f - 1] -= lambda_data[iL0 + b];
  }

  if (static_cast<uint16_T>(mLB) - 1 >= 0) {
    iL0 += static_cast<uint16_T>(mLB);
  }

  ix = static_cast<uint16_T>(mUB);
  for (b = 0; b < ix; b++) {
    f = finiteUB_data[b];
    workspace_data[f - 1] += lambda_data[iL0 + b];
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_mod_computePrimalFeasError(const real_T x[177], int32_T
  mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[177], const
  int32_T finiteUB_data[], int32_T mUB)
{
  real_T feasError;
  int32_T mIneq;
  feasError = 0.0;
  mIneq = mNonlinIneq + mLinIneq;
  for (int32_T idx{0}; idx < 170; idx++) {
    feasError = std::fmax(feasError, std::abs(cEq[idx]));
  }

  for (int32_T idx{0}; idx < mIneq; idx++) {
    feasError = std::fmax(feasError, cIneq_data[idx]);
  }

  mIneq = static_cast<uint16_T>(mLB);
  for (int32_T idx{0}; idx < mIneq; idx++) {
    int32_T finiteLB;
    finiteLB = finiteLB_data[idx];
    feasError = std::fmax(feasError, lb[finiteLB - 1] - x[finiteLB - 1]);
  }

  mIneq = static_cast<uint16_T>(mUB);
  for (int32_T idx{0}; idx < mIneq; idx++) {
    feasError = std::fmax(feasError, x[finiteUB_data[idx] - 1] - (rtInf));
  }

  return feasError;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_computeDualFeasError(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= static_cast<uint16_T>(nVar) - 1)) {
    *gradOK = ((!std::isinf(gradLag_data[idx])) && (!std::isnan(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      *val = std::fmax(*val, std::abs(gradLag_data[idx]));
      idx++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_test_exit(sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  const s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, const real_T lb[177], boolean_T
  *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done, boolean_T
  *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
  *Flags_stepType)
{
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T k;
  int32_T mLambda;
  int32_T nVar;
  boolean_T isFeasible;
  *Flags_fevalOK = true;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 169;
  if (mLambda >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->
                lambdasqp.data[0], static_cast<uint32_T>(mLambda + 1) * sizeof
                (real_T));
  }

  root_model_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad.data[0]);
      for (k = 2; k <= nVar; k++) {
        s = std::abs(TrialState->grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  smax = std::fmax(1.0, std::abs(TrialState->grad.data[idx_max - 1]));
  if (std::isinf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = root_mod_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  MeritFunction->feasRelativeFactor = std::fmax(1.0,
    MeritFunction->nlpPrimalFeasError);
  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  root_model_computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    Flags_gradOK, &MeritFunction->nlpDualFeasError);
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = std::fmax(MeritFunction->nlpDualFeasError,
      0.0);
    if (mLambda >= 0) {
      std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                  &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                  (mLambda + 1) * sizeof(real_T));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 * smax)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      *Flags_done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        *Flags_done = true;
        TrialState->sqpExitFlag = -3;
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_saveJacobian(s_7iu1rzNoAoHigERqeOFm2G_root_T *obj,
  int32_T nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T
  ineqCol0, const real_T JacCeqTrans_data[], int32_T ldJ)
{
  int32_T b;
  int32_T iCol;
  int32_T iCol_old;
  iCol = (ineqCol0 - 1) * ldJ;
  iCol_old = 0;
  b = mIneq - ineqCol0;
  for (int32_T idx_col{0}; idx_col <= b; idx_col++) {
    int32_T c;
    int32_T loop_ub;
    loop_ub = obj->JacCineqTrans_old.size[0] * obj->JacCineqTrans_old.size[1];
    if (loop_ub - 1 >= 0) {
      std::memcpy(&root_model_B.y_data[0], &obj->JacCineqTrans_old.data[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    c = static_cast<uint16_T>(nVar);
    for (int32_T k{0}; k < c; k++) {
      root_model_B.y_data[iCol_old + k] = JacCineqTrans_data[iCol + k];
    }

    if (loop_ub - 1 >= 0) {
      std::memcpy(&obj->JacCineqTrans_old.data[0], &root_model_B.y_data[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    iCol += ldJ;
    iCol_old += ldJ;
  }

  iCol = 0;
  iCol_old = 0;
  b = static_cast<uint16_T>(nVar);
  for (int32_T idx_col{0}; idx_col < 170; idx_col++) {
    for (int32_T k{0}; k < b; k++) {
      obj->JacCeqTrans_old.data[iCol_old + k] = JacCeqTrans_data[iCol + k];
    }

    iCol += ldJ;
    iCol_old = iCol;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_computeComplError(const int32_T
  fscales_lineq_constraint_size[1], const int32_T fscales_cineq_constraint_size
  [1], const real_T xCurrent[177], int32_T mIneq, const real_T cIneq_data[],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[177], const
  int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[], int32_T iL0)
{
  real_T nlpComplError;
  int32_T mLinIneq;
  nlpComplError = 0.0;
  mLinIneq = fscales_lineq_constraint_size[0];
  if ((mIneq + mLB) + mUB > 0) {
    real_T lbDelta;
    real_T lbLambda;
    int32_T iLineq0;
    int32_T idx;
    for (idx = 0; idx < mLinIneq; idx++) {
      lbDelta = cIneq_data[idx];
      lbLambda = lambda_data[(iL0 + idx) - 1];
      nlpComplError = std::fmax(nlpComplError, std::fmin(std::abs(lbDelta *
        lbLambda), std::fmin(std::abs(lbDelta), lbLambda)));
    }

    iLineq0 = (iL0 + fscales_lineq_constraint_size[0]) - 1;
    idx = fscales_cineq_constraint_size[0];
    for (int32_T b_idx{0}; b_idx < idx; b_idx++) {
      lbDelta = cIneq_data[mLinIneq + b_idx];
      lbLambda = lambda_data[iLineq0 + b_idx];
      nlpComplError = std::fmax(nlpComplError, std::fmin(std::abs(lbDelta *
        lbLambda), std::fmin(std::abs(lbDelta), lbLambda)));
    }

    iLineq0 = (iL0 + mIneq) - 1;
    mLinIneq = iLineq0 + mLB;
    idx = static_cast<uint16_T>(mLB);
    for (int32_T b_idx{0}; b_idx < idx; b_idx++) {
      int32_T finiteLB;
      finiteLB = finiteLB_data[b_idx];
      lbDelta = xCurrent[finiteLB - 1] - lb[finiteLB - 1];
      lbLambda = lambda_data[iLineq0 + b_idx];
      nlpComplError = std::fmax(nlpComplError, std::fmin(std::abs(lbDelta *
        lbLambda), std::fmin(std::abs(lbDelta), lbLambda)));
    }

    iLineq0 = static_cast<uint16_T>(mUB);
    for (idx = 0; idx < iLineq0; idx++) {
      lbDelta = lambda_data[mLinIneq + idx];
      lbLambda = (rtInf) - xCurrent[finiteUB_data[idx] - 1];
      nlpComplError = std::fmax(nlpComplError, std::fmin(std::abs(lbLambda *
        lbDelta), std::fmin(lbLambda, lbDelta)));
    }
  }

  return nlpComplError;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_computeGradLag_h(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[])
{
  int32_T f;
  int32_T g;
  int32_T i;
  int32_T iL0;
  int32_T ix;
  int32_T tmp;
  std::memcpy(&workspace_data[0], &grad_data[0], static_cast<uint16_T>(nVar) *
              sizeof(real_T));
  i = static_cast<uint16_T>(mFixed);
  for (iL0 = 0; iL0 < i; iL0++) {
    ix = finiteFixed_data[iL0];
    workspace_data[ix - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 169 + 1;
  for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
    g = iL0 + nVar;
    for (i = iL0; i < g; i++) {
      tmp = i - iL0;
      workspace_data[tmp] += AeqTrans_data[i - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 170;
    f = (mIneq - 1) * ldA + 1;
    for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
      g = iL0 + nVar;
      for (i = iL0; i < g; i++) {
        tmp = i - iL0;
        workspace_data[tmp] += AineqTrans_data[i - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 170;
  ix = static_cast<uint16_T>(mLB) - 1;
  for (i = 0; i <= ix; i++) {
    f = finiteLB_data[i];
    workspace_data[f - 1] -= lambda_data[iL0 + i];
  }

  if (static_cast<uint16_T>(mLB) - 1 >= 0) {
    iL0 += static_cast<uint16_T>(mLB);
  }

  ix = static_cast<uint16_T>(mUB);
  for (i = 0; i < ix; i++) {
    f = finiteUB_data[i];
    workspace_data[f - 1] += lambda_data[iL0 + i];
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mod_computeDualFeasError_k(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= static_cast<uint16_T>(nVar) - 1)) {
    *gradOK = ((!std::isinf(gradLag_data[idx])) && (!std::isnan(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      *val = std::fmax(*val, std::abs(gradLag_data[idx]));
      idx++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_m_updateWorkingSetForNewQP(const real_T xk[177],
  s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet, int32_T mIneq, int32_T
  mNonlinIneq, const real_T cIneq_data[], const real_T cEq[170], int32_T mLB,
  const real_T lb[177], int32_T mUB, int32_T mFixed)
{
  real_T tmp[2];
  int32_T i;
  int32_T iEq0;
  int32_T iw0;
  int32_T nVar;
  nVar = WorkingSet->nVar;
  iw0 = WorkingSet->ldA * mFixed;
  iEq0 = 0;
  for (int32_T idx{0}; idx < 170; idx++) {
    real_T WorkingSet_beq;
    WorkingSet_beq = -cEq[idx];
    WorkingSet->beq[idx] = WorkingSet_beq;
    WorkingSet->bwset.data[mFixed + idx] = WorkingSet_beq;
    for (i = 0; i < nVar; i++) {
      WorkingSet->ATwset.data[iw0 + i] = WorkingSet->Aeq.data[iEq0 + i];
    }

    iw0 += WorkingSet->ldA;
    iEq0 += WorkingSet->ldA;
  }

  i = static_cast<uint16_T>(mIneq);
  iw0 = (static_cast<uint16_T>(mIneq) / 2) << 1;
  iEq0 = iw0 - 2;
  for (int32_T idx{0}; idx <= iEq0; idx += 2) {
    _mm_storeu_pd(&WorkingSet->bineq.data[idx], _mm_mul_pd(_mm_loadu_pd
      (&cIneq_data[idx]), _mm_set1_pd(-1.0)));
  }

  for (int32_T idx{iw0}; idx < i; idx++) {
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }

  i = static_cast<uint16_T>(mLB);
  for (int32_T idx{0}; idx < i; idx++) {
    WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1] = -lb
      [WorkingSet->indexLB.data[idx] - 1] + xk[WorkingSet->indexLB.data[idx] - 1];
  }

  i = static_cast<uint16_T>(mUB);
  for (int32_T idx{0}; idx < i; idx++) {
    WorkingSet->ub.data[WorkingSet->indexUB.data[idx] - 1] = (rtInf) -
      xk[WorkingSet->indexUB.data[idx] - 1];
  }

  i = static_cast<uint16_T>(mFixed);
  for (int32_T idx{0}; idx < i; idx++) {
    _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set1_pd((rtInf)), _mm_set1_pd
      (xk[WorkingSet->indexFixed.data[idx] - 1])));
    WorkingSet->ub.data[WorkingSet->indexFixed.data[idx] - 1] = tmp[0];
    WorkingSet->bwset.data[idx] = tmp[1];
  }

  if (WorkingSet->nActiveConstr > mFixed + 170) {
    iw0 = WorkingSet->nActiveConstr;
    for (int32_T idx{mFixed + 171}; idx <= iw0; idx++) {
      switch (WorkingSet->Wid.data[idx - 1]) {
       case 4:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->lb.data
          [WorkingSet->indexLB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       case 5:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->ub.data
          [WorkingSet->indexUB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       default:
        {
          WorkingSet->bwset.data[idx - 1] = WorkingSet->bineq.data
            [WorkingSet->Wlocalidx.data[idx - 1] - 1];
          if ((mNonlinIneq > 0) && (WorkingSet->Wlocalidx.data[idx - 1] > mIneq
               - mNonlinIneq)) {
            int32_T g;
            int32_T ix0;
            iEq0 = (idx - 1) * WorkingSet->ldA;
            ix0 = (WorkingSet->Wlocalidx.data[idx - 1] - 1) * WorkingSet->ldA;
            g = static_cast<uint16_T>(nVar);
            for (i = 0; i < g; i++) {
              WorkingSet->ATwset.data[iEq0 + i] = WorkingSet->Aineq.data[ix0 + i];
            }
          }
        }
        break;
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xswap(int32_T n, real_T x_data[], int32_T ix0, int32_T
  iy0)
{
  for (int32_T k{0}; k < n; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x_data[temp_tmp];
    tmp = (iy0 + k) - 1;
    x_data[temp_tmp] = x_data[tmp];
    x_data[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_xnrm2_o(int32_T n, const real_T x_data[], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[ix0 - 1]);
    } else {
      real_T scale;
      int32_T k;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
      if (std::isnan(y)) {
        k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k <= kend) {
            if (std::isnan(x_data[k - 1])) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            y = (rtInf);
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_xzlarfg(int32_T n, real_T *alpha1, real_T x_data[],
  int32_T ix0)
{
  __m128d tmp;
  real_T a;
  real_T tau;
  real_T xnorm;
  int32_T c;
  int32_T d;
  int32_T knt;
  int32_T scalarLB;
  int32_T vectorUB;
  tau = 0.0;
  if (n > 0) {
    xnorm = root_model_xnrm2_o(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          d = (ix0 + n) - 2;
          scalarLB = ((((d - ix0) + 1) / 2) << 1) + ix0;
          vectorUB = scalarLB - 2;
          for (c = ix0; c <= vectorUB; c += 2) {
            tmp = _mm_loadu_pd(&x_data[c - 1]);
            _mm_storeu_pd(&x_data[c - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (c = scalarLB; c <= d; c++) {
            x_data[c - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

        xnorm = rt_hypotd_snf(*alpha1, root_model_xnrm2_o(n - 1, x_data, ix0));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        vectorUB = scalarLB - 2;
        for (c = ix0; c <= vectorUB; c += 2) {
          tmp = _mm_loadu_pd(&x_data[c - 1]);
          _mm_storeu_pd(&x_data[c - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (c = scalarLB; c <= d; c++) {
          x_data[c - 1] *= a;
        }

        for (c = 0; c < knt; c++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        c = (ix0 + n) - 2;
        scalarLB = ((((c - ix0) + 1) / 2) << 1) + ix0;
        vectorUB = scalarLB - 2;
        for (knt = ix0; knt <= vectorUB; knt += 2) {
          tmp = _mm_loadu_pd(&x_data[knt - 1]);
          _mm_storeu_pd(&x_data[knt - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (knt = scalarLB; knt <= c; knt++) {
          x_data[knt - 1] *= a;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T iy;
    if (n - 1 >= 0) {
      std::memset(&y_data[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
    }

    iy = 0;
    b = (n - 1) * lda + ia0;
    for (int32_T b_iy{ia0}; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = b_iy + m;
      for (int32_T ia{b_iy}; ia < d; ia++) {
        c += x_data[((ix0 + ia) - b_iy) - 1] * A_data[ia - 1];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y_data[], real_T A_data[], int32_T ia0, int32_T lda)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j{0}; j < n; j++) {
      real_T temp;
      temp = y_data[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (int32_T ijA{jA}; ijA < b; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += lda;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau,
  real_T C_data[], int32_T ic0, int32_T ldc, real_T work_data[])
{
  int32_T coltop;
  int32_T exitg1;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg2;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * ldc + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    root_model_xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0, work_data);
    root_model_xgerc(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_qrf(real_T A_data[], const int32_T A_size[2], int32_T m,
  int32_T n, int32_T nfxd, real_T tau_data[])
{
  real_T b_atmp;
  real_T tau;
  int32_T i;
  int32_T ii;
  int32_T lda;
  int32_T loop_ub;
  int32_T mmi;
  lda = A_size[0];
  loop_ub = A_size[1];
  if (loop_ub - 1 >= 0) {
    std::memset(&root_model_B.work_data[0], 0, static_cast<uint32_T>(loop_ub) *
                sizeof(real_T));
  }

  loop_ub = static_cast<uint16_T>(nfxd);
  for (i = 0; i < loop_ub; i++) {
    ii = i * lda + i;
    mmi = m - i;
    if (i + 1 < m) {
      b_atmp = A_data[ii];
      tau = root_model_xzlarfg(mmi, &b_atmp, A_data, ii + 2);
      tau_data[i] = tau;
      A_data[ii] = b_atmp;
    } else {
      tau = 0.0;
      tau_data[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = A_data[ii];
      A_data[ii] = 1.0;
      root_model_xzlarf(mmi, (n - i) - 1, ii + 1, tau, A_data, (ii + lda) + 1,
                        lda, root_model_B.work_data);
      A_data[ii] = b_atmp;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_qrpf(real_T A_data[], const int32_T A_size[2], int32_T m,
  int32_T n, int32_T nfxd, real_T tau_data[], int32_T jpvt_data[])
{
  real_T s;
  real_T smax;
  real_T vn1;
  int32_T ii;
  int32_T ii_tmp;
  int32_T itemp;
  int32_T j;
  int32_T ma;
  int32_T minmn;
  int32_T mmi;
  int32_T nmi;
  int32_T pvt;
  ma = A_size[0];
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  pvt = A_size[1];
  if (pvt - 1 >= 0) {
    std::memset(&root_model_B.work_data_c[0], 0, static_cast<uint32_T>(pvt) *
                sizeof(real_T));
  }

  if (pvt - 1 >= 0) {
    std::memset(&root_model_B.vn1_data[0], 0, static_cast<uint32_T>(pvt) *
                sizeof(real_T));
  }

  if (pvt - 1 >= 0) {
    std::memset(&root_model_B.vn2_data[0], 0, static_cast<uint32_T>(pvt) *
                sizeof(real_T));
  }

  for (j = nfxd + 1; j <= n; j++) {
    vn1 = root_model_xnrm2_o(m - nfxd, A_data, ((j - 1) * ma + nfxd) + 1);
    root_model_B.vn1_data[j - 1] = vn1;
    root_model_B.vn2_data[j - 1] = vn1;
  }

  for (j = nfxd + 1; j <= minmn; j++) {
    ii_tmp = (j - 1) * ma;
    ii = (ii_tmp + j) - 1;
    nmi = (n - j) + 1;
    mmi = m - j;
    if (nmi < 1) {
      itemp = -2;
    } else {
      itemp = -1;
      if (nmi > 1) {
        smax = std::abs(root_model_B.vn1_data[j - 1]);
        for (pvt = 2; pvt <= nmi; pvt++) {
          s = std::abs(root_model_B.vn1_data[(j + pvt) - 2]);
          if (s > smax) {
            itemp = pvt - 2;
            smax = s;
          }
        }
      }
    }

    pvt = j + itemp;
    if (pvt + 1 != j) {
      root_model_xswap(m, A_data, pvt * ma + 1, ii_tmp + 1);
      itemp = jpvt_data[pvt];
      jpvt_data[pvt] = jpvt_data[j - 1];
      jpvt_data[j - 1] = itemp;
      root_model_B.vn1_data[pvt] = root_model_B.vn1_data[j - 1];
      root_model_B.vn2_data[pvt] = root_model_B.vn2_data[j - 1];
    }

    if (j < m) {
      smax = A_data[ii];
      vn1 = root_model_xzlarfg(mmi + 1, &smax, A_data, ii + 2);
      tau_data[j - 1] = vn1;
      A_data[ii] = smax;
    } else {
      vn1 = 0.0;
      tau_data[j - 1] = 0.0;
    }

    if (j < n) {
      smax = A_data[ii];
      A_data[ii] = 1.0;
      root_model_xzlarf(mmi + 1, nmi - 1, ii + 1, vn1, A_data, (ii + ma) + 1, ma,
                        root_model_B.work_data_c);
      A_data[ii] = smax;
    }

    for (pvt = j + 1; pvt <= n; pvt++) {
      ii = (pvt - 1) * ma + j;
      vn1 = root_model_B.vn1_data[pvt - 1];
      if (vn1 != 0.0) {
        smax = std::abs(A_data[ii - 1]) / vn1;
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        s = vn1 / root_model_B.vn2_data[pvt - 1];
        s = s * s * smax;
        if (s <= 1.4901161193847656E-8) {
          if (j < m) {
            vn1 = root_model_xnrm2_o(mmi, A_data, ii + 1);
            root_model_B.vn1_data[pvt - 1] = vn1;
            root_model_B.vn2_data[pvt - 1] = vn1;
          } else {
            root_model_B.vn1_data[pvt - 1] = 0.0;
            root_model_B.vn2_data[pvt - 1] = 0.0;
          }
        } else {
          root_model_B.vn1_data[pvt - 1] = vn1 * std::sqrt(smax);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgeqp3(real_T A_data[], const int32_T A_size[2], int32_T
  m, int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T tau_size[1])
{
  int32_T ma;
  int32_T minmana;
  int32_T minmn;
  int32_T nfxd;
  static const int32_T offsets[4]{ 0, 1, 2, 3 };

  ma = A_size[0];
  if (A_size[0] <= A_size[1]) {
    minmana = A_size[0];
  } else {
    minmana = A_size[1];
  }

  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  tau_size[0] = minmana;
  if (minmana - 1 >= 0) {
    std::memset(&tau_data[0], 0, static_cast<uint32_T>(minmana) * sizeof(real_T));
  }

  if (minmn < 1) {
    nfxd = (n / 4) << 2;
    ma = nfxd - 4;
    for (minmana = 0; minmana <= ma; minmana += 4) {
      _mm_storeu_si128((__m128i *)&jpvt_data[minmana], _mm_add_epi32
                       (_mm_add_epi32(_mm_set1_epi32(minmana), _mm_loadu_si128((
        const __m128i *)&offsets[0])), _mm_set1_epi32(1)));
    }

    for (minmana = nfxd; minmana < n; minmana++) {
      jpvt_data[minmana] = minmana + 1;
    }
  } else {
    nfxd = -1;
    for (minmana = 0; minmana < n; minmana++) {
      if (jpvt_data[minmana] != 0) {
        nfxd++;
        if (minmana + 1 != nfxd + 1) {
          root_model_xswap(m, A_data, minmana * ma + 1, nfxd * ma + 1);
          jpvt_data[minmana] = jpvt_data[nfxd];
          jpvt_data[nfxd] = minmana + 1;
        } else {
          jpvt_data[minmana] = minmana + 1;
        }
      } else {
        jpvt_data[minmana] = minmana + 1;
      }
    }

    if (nfxd + 1 <= minmn) {
      nfxd++;
    } else {
      nfxd = minmn;
    }

    root_model_qrf(A_data, A_size, m, n, nfxd, tau_data);
    if (nfxd < minmn) {
      root_model_qrpf(A_data, A_size, m, n, nfxd, tau_data, jpvt_data);
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factorQRE(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const
  real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = static_cast<uint16_T>(mrows);
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    root_model_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols, obj->jpvt.data,
                      obj->tau.data, obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xorgqr(int32_T m, int32_T n, int32_T k, real_T A_data[],
  const int32_T A_size[2], int32_T lda, const real_T tau_data[])
{
  __m128d tmp;
  int32_T b_k;
  int32_T d;
  int32_T i;
  int32_T ia;
  int32_T itau;
  int32_T scalarLB;
  int32_T vectorUB;
  if (n >= 1) {
    for (itau = k; itau < n; itau++) {
      ia = itau * lda;
      std::memset(&A_data[ia], 0, static_cast<uint32_T>(m) * sizeof(real_T));
      A_data[ia + itau] = 1.0;
    }

    itau = k - 1;
    i = A_size[1];
    if (i - 1 >= 0) {
      std::memset(&root_model_B.work_data_b[0], 0, static_cast<uint32_T>(i) *
                  sizeof(real_T));
    }

    for (i = k; i >= 1; i--) {
      ia = (i - 1) * lda + i;
      if (i < n) {
        A_data[ia - 1] = 1.0;
        root_model_xzlarf((m - i) + 1, n - i, ia, tau_data[itau], A_data, ia +
                          lda, lda, root_model_B.work_data_b);
      }

      if (i < m) {
        d = (ia + m) - i;
        scalarLB = ((((d - ia) / 2) << 1) + ia) + 1;
        vectorUB = scalarLB - 2;
        for (b_k = ia + 1; b_k <= vectorUB; b_k += 2) {
          tmp = _mm_loadu_pd(&A_data[b_k - 1]);
          _mm_storeu_pd(&A_data[b_k - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (-tau_data[itau])));
        }

        for (b_k = scalarLB; b_k <= d; b_k++) {
          A_data[b_k - 1] *= -tau_data[itau];
        }
      }

      A_data[ia - 1] = 1.0 - tau_data[itau];
      d = static_cast<uint16_T>(i - 1);
      for (b_k = 0; b_k < d; b_k++) {
        A_data[(ia - b_k) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_sortLambdaQP(real_T lambda_data[], int32_T
  WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
  WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const int32_T
  WorkingSet_Wlocalidx_data[], real_T workspace_data[])
{
  if (WorkingSet_nActiveConstr != 0) {
    int32_T currentMplier;
    int32_T idxOffset;
    int32_T mAll;
    mAll = (((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4])
            + WorkingSet_sizes[2]) + 169;
    if (static_cast<uint16_T>(mAll + 1) - 1 >= 0) {
      std::memcpy(&workspace_data[0], &lambda_data[0], static_cast<uint16_T>
                  (mAll + 1) * sizeof(real_T));
    }

    if (mAll >= 0) {
      std::memset(&lambda_data[0], 0, static_cast<uint32_T>(mAll + 1) * sizeof
                  (real_T));
    }

    currentMplier = 0;
    mAll = 0;
    while ((mAll + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid_data[mAll] <=
            2)) {
      if (WorkingSet_Wid_data[mAll] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }

    while (mAll + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid_data[mAll]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_test_exit_n(s7RdrPWkr8UPAUyTdDJkLaG_root__T *Flags,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, sG8JZ69axY52WWR6RKyApQC_root__T
  *MeritFunction, const int32_T fscales_lineq_constraint_size[1], const int32_T
  fscales_cineq_constraint_size[1], s_Jrk1gF3jitJmkPBgphEknE_root_T *WorkingSet,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *QRManager, const real_T lb[177])
{
  real_T nlpComplErrorTmp;
  real_T s;
  real_T smax;
  real_T tmp;
  int32_T c_ix;
  int32_T iQR0;
  int32_T idx_max;
  int32_T ix;
  int32_T mLambda;
  int32_T n;
  int32_T nVar;
  int32_T rankR;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T isFeasible;
  nVar = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 169;
  if (mLambda >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->
                lambdasqp.data[0], static_cast<uint32_T>(mLambda + 1) * sizeof
                (real_T));
  }

  root_model_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad.data[0]);
      for (ix = 2; ix <= nVar; ix++) {
        s = std::abs(TrialState->grad.data[ix - 1]);
        if (s > smax) {
          idx_max = ix;
          smax = s;
        }
      }
    }
  }

  smax = std::fmax(1.0, std::abs(TrialState->grad.data[idx_max - 1]));
  if (std::isinf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = root_mod_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = std::fmax(1.0,
      MeritFunction->nlpPrimalFeasError);
  }

  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  root_model_computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    &Flags->gradOK, &MeritFunction->nlpDualFeasError);
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = root_model_computeComplError
      (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
       TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
       WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
       WorkingSet->indexUB.data, WorkingSet->sizes[4],
       TrialState->lambdaStopTest.data, WorkingSet->sizes[0] + 171);
    MeritFunction->firstOrderOpt = std::fmax(MeritFunction->nlpDualFeasError,
      MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      root_model_computeGradLag_h(memspace->workspace_float.data,
        WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
        WorkingSet->sizes[2], WorkingSet->Aineq.data, WorkingSet->Aeq.data,
        WorkingSet->indexFixed.data, WorkingSet->sizes[0],
        WorkingSet->indexLB.data, WorkingSet->sizes[3], WorkingSet->indexUB.data,
        WorkingSet->sizes[4], TrialState->lambdaStopTestPrev.data);
      root_mod_computeDualFeasError_k(WorkingSet->nVar,
        memspace->workspace_float.data, &dxTooSmall, &s);
      nlpComplErrorTmp = root_model_computeComplError
        (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
         TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
         WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
         WorkingSet->indexUB.data, WorkingSet->sizes[4],
         TrialState->lambdaStopTestPrev.data, WorkingSet->sizes[0] + 171);
      if ((s < MeritFunction->nlpDualFeasError) && (nlpComplErrorTmp <
           MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = std::fmax(s, nlpComplErrorTmp);
        if (mLambda >= 0) {
          std::memcpy(&TrialState->lambdaStopTest.data[0],
                      &TrialState->lambdaStopTestPrev.data[0], static_cast<
                      uint32_T>(mLambda + 1) * sizeof(real_T));
        }
      } else if (mLambda >= 0) {
        std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                    &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                    (mLambda + 1) * sizeof(real_T));
      }
    } else if (mLambda >= 0) {
      std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                  &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                  (mLambda + 1) * sizeof(real_T));
    }

    smax *= 1.0E-6;
    if (isFeasible && (MeritFunction->nlpDualFeasError <= smax) &&
        (MeritFunction->nlpComplError <= smax)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          ix = 0;
          exitg1 = false;
          while ((!exitg1) && (ix <= static_cast<uint16_T>(WorkingSet->nVar) - 1))
          {
            if (1.0E-6 * std::fmax(1.0, std::abs(TrialState->xstarsqp[ix])) <=
                std::abs(TrialState->delta_x.data[ix])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              ix++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType == 2) {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              } else {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              }
            } else if (WorkingSet->nActiveConstr == 0) {
              Flags->done = true;
              TrialState->sqpExitFlag = 2;
            } else {
              root_m_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet,
                WorkingSet->sizes[2], TrialState->mNonlinIneq,
                TrialState->cIneq.data, TrialState->cEq, WorkingSet->sizes[3],
                lb, WorkingSet->sizes[4], WorkingSet->sizes[0]);
              if (WorkingSet->nActiveConstr - 1 >= 0) {
                std::memset(&TrialState->lambda.data[0], 0, static_cast<uint32_T>
                            (WorkingSet->nActiveConstr) * sizeof(real_T));
              }

              root_model_factorQRE(QRManager, WorkingSet->ATwset.data,
                                   WorkingSet->nVar, WorkingSet->nActiveConstr,
                                   WorkingSet->ldA);
              rankR = QRManager->minRowCol;
              for (idx_max = 0; idx_max < rankR; idx_max++) {
                iQR0 = QRManager->ldq * idx_max + idx_max;
                n = QRManager->mrows - idx_max;
                if (n - 2 >= 0) {
                  std::memcpy(&QRManager->Q.data[iQR0 + 1], &QRManager->
                              QR.data[iQR0 + 1], static_cast<uint32_T>(n - 1) *
                              sizeof(real_T));
                }
              }

              root_model_xorgqr(QRManager->mrows, QRManager->mrows,
                                QRManager->minRowCol, QRManager->Q.data,
                                QRManager->Q.size, QRManager->ldq,
                                QRManager->tau.data);
              rankR = QRManager->ldq;
              std::memset(&memspace->workspace_float.data[0], 0,
                          static_cast<uint16_T>(WorkingSet->nVar) * sizeof
                          (real_T));
              iQR0 = 0;
              n = (WorkingSet->nVar - 1) * QRManager->ldq + 1;
              for (idx_max = 1; rankR < 0 ? idx_max >= n : idx_max <= n; idx_max
                   += rankR) {
                s = 0.0;
                c_ix = idx_max + nVar;
                for (ix = idx_max; ix < c_ix; ix++) {
                  s += QRManager->Q.data[ix - 1] * TrialState->grad.data[ix -
                    idx_max];
                }

                memspace->workspace_float.data[iQR0] -= s;
                iQR0++;
              }

              if (WorkingSet->nVar >= WorkingSet->nActiveConstr) {
                nVar = WorkingSet->nVar;
              } else {
                nVar = WorkingSet->nActiveConstr;
              }

              s = std::fmin(1.4901161193847656E-8, static_cast<real_T>(nVar) *
                            2.2204460492503131E-16) * std::abs
                (QRManager->QR.data[0]);
              if (WorkingSet->nVar <= WorkingSet->nActiveConstr) {
                ix = WorkingSet->nVar;
              } else {
                ix = WorkingSet->nActiveConstr;
              }

              rankR = 0;
              nVar = 0;
              while ((rankR < ix) && (std::abs(QRManager->QR.data[nVar]) > s)) {
                rankR++;
                nVar = (nVar + QRManager->ldq) + 1;
              }

              if (rankR != 0) {
                for (nVar = rankR; nVar >= 1; nVar--) {
                  iQR0 = ((nVar - 1) * QRManager->ldq + nVar) - 2;
                  memspace->workspace_float.data[nVar - 1] /= QRManager->
                    QR.data[iQR0 + 1];
                  for (idx_max = 0; idx_max <= nVar - 2; idx_max++) {
                    c_ix = (nVar - idx_max) - 2;
                    memspace->workspace_float.data[c_ix] -=
                      memspace->workspace_float.data[nVar - 1] *
                      QRManager->QR.data[iQR0 - idx_max];
                  }
                }
              }

              if (WorkingSet->nActiveConstr <= ix) {
                ix = WorkingSet->nActiveConstr;
              }

              for (nVar = 0; nVar < ix; nVar++) {
                TrialState->lambda.data[QRManager->jpvt.data[nVar] - 1] =
                  memspace->workspace_float.data[nVar];
              }

              root_model_sortLambdaQP(TrialState->lambda.data,
                WorkingSet->nActiveConstr, WorkingSet->sizes,
                WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
              root_model_computeGradLag_h(memspace->workspace_float.data,
                WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
                WorkingSet->sizes[2], WorkingSet->Aineq.data,
                WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
                WorkingSet->sizes[0], WorkingSet->indexLB.data,
                WorkingSet->sizes[3], WorkingSet->indexUB.data,
                WorkingSet->sizes[4], TrialState->lambda.data);
              root_mod_computeDualFeasError_k(WorkingSet->nVar,
                memspace->workspace_float.data, &isFeasible, &s);
              nlpComplErrorTmp = root_model_computeComplError
                (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
                 TrialState->xstarsqp, WorkingSet->sizes[2],
                 TrialState->cIneq.data, WorkingSet->indexLB.data,
                 WorkingSet->sizes[3], lb, WorkingSet->indexUB.data,
                 WorkingSet->sizes[4], TrialState->lambda.data,
                 WorkingSet->sizes[0] + 171);
              tmp = std::fmax(s, nlpComplErrorTmp);
              if (tmp <= std::fmax(MeritFunction->nlpDualFeasError,
                                   MeritFunction->nlpComplError)) {
                MeritFunction->nlpDualFeasError = s;
                MeritFunction->nlpComplError = nlpComplErrorTmp;
                MeritFunction->firstOrderOpt = tmp;
                if (mLambda >= 0) {
                  std::memcpy(&TrialState->lambdaStopTest.data[0],
                              &TrialState->lambda.data[0], static_cast<uint32_T>
                              (mLambda + 1) * sizeof(real_T));
                }
              }

              if ((MeritFunction->nlpDualFeasError <= smax) &&
                  (MeritFunction->nlpComplError <= smax)) {
                TrialState->sqpExitFlag = 1;
              } else {
                TrialState->sqpExitFlag = 2;
              }

              Flags->done = true;
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 17700) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static boolean_T root_model_BFGSUpdate(int32_T nvar, real_T Bk[31329], const
  real_T sk_data[], real_T yk_data[], real_T workspace_data[])
{
  __m128d tmp;
  __m128d tmp_0;
  real_T curvatureS;
  real_T dotSY;
  int32_T b_ix;
  int32_T b_jA;
  int32_T b_tmp;
  int32_T ix;
  int32_T jy;
  int32_T vectorUB;
  boolean_T success;
  dotSY = 0.0;
  if (nvar >= 1) {
    for (ix = 0; ix < nvar; ix++) {
      dotSY += sk_data[ix] * yk_data[ix];
    }
  }

  b_tmp = static_cast<uint16_T>(nvar);
  std::memset(&workspace_data[0], 0, static_cast<uint16_T>(nvar) * sizeof(real_T));
  ix = 0;
  jy = (nvar - 1) * 177 + 1;
  for (int32_T b{1}; b <= jy; b += 177) {
    b_ix = b + nvar;
    for (b_jA = b; b_jA < b_ix; b_jA++) {
      vectorUB = b_jA - b;
      workspace_data[vectorUB] += Bk[b_jA - 1] * sk_data[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (int32_T b{0}; b < nvar; b++) {
      curvatureS += workspace_data[b] * sk_data[b];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    dotSY = 0.8 * curvatureS / (curvatureS - dotSY);
    jy = (static_cast<uint16_T>(nvar) / 2) << 1;
    vectorUB = jy - 2;
    for (ix = 0; ix <= vectorUB; ix += 2) {
      tmp_0 = _mm_loadu_pd(&yk_data[ix]);
      _mm_storeu_pd(&yk_data[ix], _mm_mul_pd(_mm_set1_pd(dotSY), tmp_0));
    }

    for (ix = jy; ix < b_tmp; ix++) {
      yk_data[ix] *= dotSY;
    }

    if (!(1.0 - dotSY == 0.0)) {
      jy = (nvar / 2) << 1;
      vectorUB = jy - 2;
      for (int32_T b{0}; b <= vectorUB; b += 2) {
        tmp_0 = _mm_loadu_pd(&workspace_data[b]);
        tmp = _mm_loadu_pd(&yk_data[b]);
        _mm_storeu_pd(&yk_data[b], _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(1.0
          - dotSY)), tmp));
      }

      for (int32_T b{jy}; b < nvar; b++) {
        yk_data[b] += (1.0 - dotSY) * workspace_data[b];
      }
    }

    dotSY = 0.0;
    if (nvar >= 1) {
      for (ix = 0; ix < nvar; ix++) {
        dotSY += sk_data[ix] * yk_data[ix];
      }
    }
  }

  success = ((curvatureS > 2.2204460492503131E-16) && (dotSY >
              2.2204460492503131E-16));
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 1;
      for (int32_T b{0}; b < b_tmp; b++) {
        if (workspace_data[b] != 0.0) {
          real_T temp;
          temp = workspace_data[b] * curvatureS;
          b_ix = nvar + ix;
          jy = (((b_ix - ix) / 2) << 1) + ix;
          vectorUB = jy - 2;
          for (b_jA = ix; b_jA <= vectorUB; b_jA += 2) {
            tmp_0 = _mm_loadu_pd(&workspace_data[b_jA - ix]);
            tmp = _mm_loadu_pd(&Bk[b_jA - 1]);
            _mm_storeu_pd(&Bk[b_jA - 1], _mm_add_pd(_mm_mul_pd(tmp_0,
              _mm_set1_pd(temp)), tmp));
          }

          for (b_jA = jy; b_jA < b_ix; b_jA++) {
            Bk[b_jA - 1] += workspace_data[b_jA - ix] * temp;
          }
        }

        ix += 177;
      }
    }

    dotSY = 1.0 / dotSY;
    if (!(dotSY == 0.0)) {
      b_jA = 1;
      for (ix = 0; ix < b_tmp; ix++) {
        curvatureS = yk_data[ix];
        if (curvatureS != 0.0) {
          curvatureS *= dotSY;
          b_ix = nvar + b_jA;
          jy = (((b_ix - b_jA) / 2) << 1) + b_jA;
          vectorUB = jy - 2;
          for (int32_T b{b_jA}; b <= vectorUB; b += 2) {
            tmp_0 = _mm_loadu_pd(&yk_data[b - b_jA]);
            tmp = _mm_loadu_pd(&Bk[b - 1]);
            _mm_storeu_pd(&Bk[b - 1], _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
              (curvatureS)), tmp));
          }

          for (int32_T b{jy}; b < b_ix; b++) {
            Bk[b - 1] += yk_data[b - b_jA] * curvatureS;
          }
        }

        b_jA += 177;
      }
    }
  }

  return success;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factorQRE_h(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, int32_T
  mrows, int32_T ncols)
{
  if (mrows * ncols == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    root_model_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols, obj->jpvt.data,
                      obj->tau.data, obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_countsort(int32_T x_data[], int32_T xLen, int32_T
  workspace_data[], int32_T xMin, int32_T xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T b;
    int32_T idxEnd;
    int32_T idxFill;
    int32_T idxStart;
    int32_T maxOffset;
    b = xMax - xMin;
    if (b >= 0) {
      std::memset(&workspace_data[0], 0, static_cast<uint32_T>(b + 1) * sizeof
                  (int32_T));
    }

    maxOffset = b - 1;
    for (b = 0; b < xLen; b++) {
      idxFill = x_data[b] - xMin;
      workspace_data[idxFill]++;
    }

    for (b = 2; b <= maxOffset + 2; b++) {
      workspace_data[b - 1] += workspace_data[b - 2];
    }

    idxStart = 1;
    idxEnd = workspace_data[0];
    for (b = 0; b <= maxOffset; b++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = b + xMin;
      }

      idxStart = workspace_data[b] + 1;
      idxEnd = workspace_data[b + 1];
    }

    for (maxOffset = idxStart; maxOffset <= idxEnd; maxOffset++) {
      x_data[maxOffset - 1] = xMax;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_removeConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_global)
{
  int32_T TYPE;
  TYPE = obj->Wid.data[idx_global - 1] - 1;
  obj->isActiveConstr.data[(obj->isActiveIdx[obj->Wid.data[idx_global - 1] - 1]
    + obj->Wlocalidx.data[idx_global - 1]) - 2] = false;
  if (idx_global < obj->nActiveConstr) {
    int32_T b;
    obj->Wid.data[idx_global - 1] = obj->Wid.data[obj->nActiveConstr - 1];
    obj->Wlocalidx.data[idx_global - 1] = obj->Wlocalidx.data[obj->nActiveConstr
      - 1];
    b = static_cast<uint16_T>(obj->nVar);
    for (int32_T idx{0}; idx < b; idx++) {
      obj->ATwset.data[idx + obj->ldA * (idx_global - 1)] = obj->ATwset.data
        [(obj->nActiveConstr - 1) * obj->ldA + idx];
    }

    obj->bwset.data[idx_global - 1] = obj->bwset.data[obj->nActiveConstr - 1];
  }

  obj->nActiveConstr--;
  obj->nWConstr[TYPE]--;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static int32_T root_model_RemoveDependentEq_(s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T c_tmp;
  int32_T idx_col;
  int32_T ix;
  int32_T mTotalWorkingEq;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T nVar;
  int32_T totalRank;
  boolean_T exitg1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq > 0) {
    c_tmp = static_cast<uint16_T>(workingset->nVar);
    for (totalRank = 0; totalRank < mTotalWorkingEq; totalRank++) {
      for (idx_col = 0; idx_col < c_tmp; idx_col++) {
        qrmanager->QR.data[totalRank + qrmanager->ldq * idx_col] =
          workingset->ATwset.data[workingset->ldA * totalRank + idx_col];
      }
    }

    idx_col = mTotalWorkingEq - workingset->nVar;
    if (idx_col > 0) {
      nDepInd = idx_col;
    }

    std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint16_T>
                (workingset->nVar) * sizeof(int32_T));
    root_model_factorQRE_h(qrmanager, mTotalWorkingEq, workingset->nVar);
    if (mTotalWorkingEq >= workingset->nVar) {
      totalRank = mTotalWorkingEq;
    } else {
      totalRank = workingset->nVar;
    }

    tol = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 *
                    static_cast<real_T>(totalRank));
    if (workingset->nVar <= mTotalWorkingEq) {
      totalRank = workingset->nVar;
    } else {
      totalRank = mTotalWorkingEq;
    }

    totalRank += (totalRank - 1) * qrmanager->ldq;
    while ((totalRank > 0) && (std::abs(qrmanager->QR.data[totalRank - 1]) <=
            std::abs(qrmanager->QR.data[0]) * tol)) {
      totalRank = (totalRank - qrmanager->ldq) - 1;
      nDepInd++;
    }

    if (nDepInd > 0) {
      ix = qrmanager->minRowCol;
      for (idx_col = 0; idx_col < ix; idx_col++) {
        totalRank = qrmanager->ldq * idx_col + idx_col;
        nVar = qrmanager->mrows - idx_col;
        if (nVar - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[totalRank + 1], &qrmanager->
                      QR.data[totalRank + 1], static_cast<uint32_T>(nVar - 1) *
                      sizeof(real_T));
        }
      }

      root_model_xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
                        qrmanager->Q.data, qrmanager->Q.size, qrmanager->ldq,
                        qrmanager->tau.data);
      nVar = 0;
      exitg1 = false;
      while ((!exitg1) && (nVar <= nDepInd - 1)) {
        ix = ((mTotalWorkingEq - nVar) - 1) * qrmanager->ldq;
        qtb = 0.0;
        for (idx_col = 0; idx_col < mTotalWorkingEq; idx_col++) {
          qtb += qrmanager->Q.data[ix + idx_col] * workingset->
            bwset.data[idx_col];
        }

        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          nVar++;
        }
      }
    }

    if (nDepInd > 0) {
      for (totalRank = 0; totalRank < mTotalWorkingEq; totalRank++) {
        nVar = qrmanager->ldq * totalRank;
        ix = workingset->ldA * totalRank;
        for (idx_col = 0; idx_col < c_tmp; idx_col++) {
          qrmanager->QR.data[nVar + idx_col] = workingset->ATwset.data[ix +
            idx_col];
        }
      }

      for (nVar = 0; nVar < mWorkingFixed; nVar++) {
        qrmanager->jpvt.data[nVar] = 1;
      }

      nVar = workingset->nWConstr[0] + 1;
      if (nVar <= mTotalWorkingEq) {
        std::memset(&qrmanager->jpvt.data[nVar + -1], 0, static_cast<uint32_T>
                    ((mTotalWorkingEq - nVar) + 1) * sizeof(int32_T));
      }

      root_model_factorQRE_h(qrmanager, workingset->nVar, mTotalWorkingEq);
      for (mWorkingFixed = 0; mWorkingFixed < nDepInd; mWorkingFixed++) {
        memspace->workspace_int.data[mWorkingFixed] = qrmanager->jpvt.data
          [(mTotalWorkingEq - nDepInd) + mWorkingFixed];
      }

      root_model_countsort(memspace->workspace_int.data, nDepInd,
                           memspace->workspace_sort.data, 1, mTotalWorkingEq);
      for (mTotalWorkingEq = nDepInd; mTotalWorkingEq >= 1; mTotalWorkingEq--) {
        nVar = (workingset->nWConstr[0] + workingset->nWConstr[1]) - 1;
        if (nVar + 1 != 0) {
          c_tmp = memspace->workspace_int.data[mTotalWorkingEq - 1];
          if (c_tmp <= nVar + 1) {
            if ((nVar + 1 == workingset->nActiveConstr) || (nVar + 1 == c_tmp))
            {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx.data[c_tmp - 1];
              root_model_removeConstr(workingset, memspace->
                workspace_int.data[mTotalWorkingEq - 1]);
            } else {
              workingset->mEqRemoved++;
              totalRank = workingset->Wid.data[c_tmp - 1] - 1;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx.data[c_tmp - 1];
              workingset->isActiveConstr.data[(workingset->
                isActiveIdx[workingset->Wid.data[c_tmp - 1] - 1] +
                workingset->Wlocalidx.data[c_tmp - 1]) - 2] = false;
              workingset->Wid.data[c_tmp - 1] = workingset->Wid.data[nVar];
              workingset->Wlocalidx.data[c_tmp - 1] = workingset->
                Wlocalidx.data[nVar];
              idx_col = static_cast<uint16_T>(workingset->nVar);
              for (mWorkingFixed = 0; mWorkingFixed < idx_col; mWorkingFixed++)
              {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA * (c_tmp
                  - 1)] = workingset->ATwset.data[workingset->ldA * nVar +
                  mWorkingFixed];
              }

              workingset->bwset.data[c_tmp - 1] = workingset->bwset.data[nVar];
              workingset->Wid.data[nVar] = workingset->Wid.data
                [workingset->nActiveConstr - 1];
              workingset->Wlocalidx.data[nVar] = workingset->
                Wlocalidx.data[workingset->nActiveConstr - 1];
              idx_col = static_cast<uint16_T>(workingset->nVar);
              for (mWorkingFixed = 0; mWorkingFixed < idx_col; mWorkingFixed++)
              {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA * nVar] =
                  workingset->ATwset.data[(workingset->nActiveConstr - 1) *
                  workingset->ldA + mWorkingFixed];
              }

              workingset->bwset.data[nVar] = workingset->bwset.data
                [workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[totalRank]--;
            }
          }
        }
      }
    }
  }

  return nDepInd;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_RemoveDependentIneq_(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  int32_T b;
  int32_T d;
  int32_T ix0;
  int32_T iy0;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      nDepIneq = workingset->nVar;
    } else {
      nDepIneq = workingset->nActiveConstr;
    }

    tol = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * static_cast<
                    real_T>(nDepIneq));
    for (nDepIneq = 0; nDepIneq < nFixedConstr; nDepIneq++) {
      qrmanager->jpvt.data[nDepIneq] = 1;
    }

    if (nFixedConstr + 1 <= workingset->nActiveConstr) {
      std::memset(&qrmanager->jpvt.data[nFixedConstr], 0, static_cast<uint32_T>
                  (workingset->nActiveConstr - nFixedConstr) * sizeof(int32_T));
    }

    b = workingset->nActiveConstr;
    for (nDepIneq = 0; nDepIneq < b; nDepIneq++) {
      iy0 = qrmanager->ldq * nDepIneq;
      ix0 = workingset->ldA * nDepIneq;
      d = static_cast<uint16_T>(nVar);
      for (nActiveConstr = 0; nActiveConstr < d; nActiveConstr++) {
        qrmanager->QR.data[iy0 + nActiveConstr] = workingset->ATwset.data[ix0 +
          nActiveConstr];
      }
    }

    root_model_factorQRE_h(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (nActiveConstr = workingset->nActiveConstr - 1; nActiveConstr + 1 > nVar;
         nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
        jpvt.data[nActiveConstr];
    }

    maxDiag = std::abs(qrmanager->QR.data[0]);
    for (nVar = 0; nVar < nActiveConstr; nVar++) {
      maxDiag = std::fmax(maxDiag, std::abs(qrmanager->QR.data[((nVar + 1) *
        qrmanager->ldq + nVar) + 1]));
    }

    if (nActiveConstr + 1 <= workingset->nVar) {
      nVar = qrmanager->ldq * nActiveConstr + nActiveConstr;
      while ((nActiveConstr + 1 > nFixedConstr) && (std::abs(qrmanager->
               QR.data[nVar]) < tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
          jpvt.data[nActiveConstr];
        nActiveConstr--;
        nVar = (nVar - qrmanager->ldq) - 1;
      }
    }

    root_model_countsort(memspace->workspace_int.data, nDepIneq,
                         memspace->workspace_sort.data, nFixedConstr + 1,
                         workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      root_model_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static int32_T root_model_rank(const real_T qrmanager_QR_data[], const int32_T
  qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols)
{
  int32_T minmn;
  int32_T r;
  r = 0;
  if (qrmanager_mrows <= qrmanager_ncols) {
    minmn = qrmanager_mrows;
  } else {
    minmn = qrmanager_ncols;
  }

  if (minmn > 0) {
    real_T tol;
    int32_T tmp;
    if (qrmanager_mrows >= qrmanager_ncols) {
      tmp = qrmanager_mrows;
    } else {
      tmp = qrmanager_ncols;
    }

    tol = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * static_cast<
                    real_T>(tmp)) * std::abs(qrmanager_QR_data[0]);
    while ((r < minmn) && (!(std::abs(qrmanager_QR_data[qrmanager_QR_size[0] * r
              + r]) <= tol))) {
      r++;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_i(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T scalarLB;
    int32_T vectorUB;
    b = static_cast<uint16_T>(n);
    scalarLB = (static_cast<uint16_T>(n) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T b_iy{0}; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (int32_T b_iy{scalarLB}; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy +=
         lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = b_iy + m;
      for (b = b_iy; b < e; b++) {
        c += x_data[b - b_iy] * A_data[b - 1];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_m_maxConstraintViolation_i(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T f;
  int32_T k;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      root_model_xgemv_i(177, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
                         obj->maxConstrWorkspace.data);
      f = static_cast<uint16_T>(obj->sizes[2]);
      for (k = 0; k < f; k++) {
        obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[k] - x_data[k +
          177];
        obj->maxConstrWorkspace.data[k] = obj_maxConstrWorkspace;
        v = std::fmax(v, obj_maxConstrWorkspace);
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
                (real_T));
    root_model_xgemv_i(177, 170, obj->Aeq.data, obj->ldA, x_data,
                       obj->maxConstrWorkspace.data);
    for (k = 0; k < 170; k++) {
      obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] -
        x_data[(mIneq + k) + 177]) + x_data[(obj->sizes[2] + k) + 347];
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      root_model_xgemv_i(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
                         x_data, obj->maxConstrWorkspace.data);
      mIneq = static_cast<uint16_T>(obj->sizes[2]);
      for (k = 0; k < mIneq; k++) {
        v = std::fmax(v, obj->maxConstrWorkspace.data[k]);
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
                (real_T));
    root_model_xgemv_i(obj->nVar, 170, obj->Aeq.data, obj->ldA, x_data,
                       obj->maxConstrWorkspace.data);
    for (k = 0; k < 170; k++) {
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, -x_data[obj->indexLB.data[mIneq] - 1] - obj->lb.data
                    [obj->indexLB.data[mIneq] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    k = static_cast<uint16_T>(obj->sizes[4]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, x_data[obj->indexUB.data[mIneq] - 1] - obj->ub.data
                    [obj->indexUB.data[mIneq] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    k = static_cast<uint16_T>(obj->sizes[0]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, std::abs(x_data[obj->indexFixed.data[mIneq] - 1] -
        obj->ub.data[obj->indexFixed.data[mIneq] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_it(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T scalarLB;
    int32_T vectorUB;
    b = static_cast<uint16_T>(n);
    scalarLB = (static_cast<uint16_T>(n) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T b_iy{0}; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (int32_T b_iy{scalarLB}; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy +=
         lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = b_iy + m;
      for (b = b_iy; b < e; b++) {
        c += x_data[((ix0 + b) - b_iy) - 1] * A_data[b - 1];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root__maxConstraintViolation_i0(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[], int32_T ix0)
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T f;
  int32_T k;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      root_model_xgemv_it(177, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
                          ix0, obj->maxConstrWorkspace.data);
      f = static_cast<uint16_T>(obj->sizes[2]);
      for (k = 0; k < f; k++) {
        obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[k] - x_data[(ix0 +
          k) + 176];
        obj->maxConstrWorkspace.data[k] = obj_maxConstrWorkspace;
        v = std::fmax(v, obj_maxConstrWorkspace);
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
                (real_T));
    root_model_xgemv_it(177, 170, obj->Aeq.data, obj->ldA, x_data, ix0,
                        obj->maxConstrWorkspace.data);
    for (k = 0; k < 170; k++) {
      obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] -
        x_data[((ix0 + mIneq) + k) + 176]) + x_data[((ix0 + obj->sizes[2]) + k)
        + 346];
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      root_model_xgemv_it(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
                          x_data, ix0, obj->maxConstrWorkspace.data);
      mIneq = static_cast<uint16_T>(obj->sizes[2]);
      for (k = 0; k < mIneq; k++) {
        v = std::fmax(v, obj->maxConstrWorkspace.data[k]);
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
                (real_T));
    root_model_xgemv_it(obj->nVar, 170, obj->Aeq.data, obj->ldA, x_data, ix0,
                        obj->maxConstrWorkspace.data);
    for (k = 0; k < 170; k++) {
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, -x_data[(ix0 + obj->indexLB.data[mIneq]) - 2] -
                    obj->lb.data[obj->indexLB.data[mIneq] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    k = static_cast<uint16_T>(obj->sizes[4]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, x_data[(ix0 + obj->indexUB.data[mIneq]) - 2] -
                    obj->ub.data[obj->indexUB.data[mIneq] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    k = static_cast<uint16_T>(obj->sizes[0]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      v = std::fmax(v, std::abs(x_data[(ix0 + obj->indexFixed.data[mIneq]) - 2]
        - obj->ub.data[obj->indexFixed.data[mIneq] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static boolean_T root_mo_feasibleX0ForWorkingSet(real_T workspace_data[], const
  int32_T workspace_size[2], real_T xCurrent_data[],
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T constrViolation_basicX;
  real_T temp;
  int32_T b_idx;
  int32_T b_n;
  int32_T br;
  int32_T exitg1;
  int32_T iAcol;
  int32_T iQR0;
  int32_T ix;
  int32_T jBcol;
  int32_T ldq;
  int32_T mWConstr;
  int32_T n;
  int32_T nVar;
  int32_T rankQR;
  int32_T scalarLB;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    if (workingset->nActiveConstr >= workingset->nVar) {
      scalarLB = static_cast<uint16_T>(workingset->nVar);
      for (rankQR = 0; rankQR < scalarLB; rankQR++) {
        ix = qrmanager->ldq * rankQR;
        for (b_idx = 0; b_idx < mWConstr; b_idx++) {
          qrmanager->QR.data[b_idx + ix] = workingset->ATwset.data
            [workingset->ldA * b_idx + rankQR];
        }
      }

      std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint16_T>
                  (workingset->nVar) * sizeof(int32_T));
      root_model_factorQRE_h(qrmanager, workingset->nActiveConstr,
        workingset->nVar);
      ix = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < ix; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR + rankQR;
        b_n = qrmanager->mrows - rankQR;
        if (b_n - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0 + 1],
                      static_cast<uint32_T>(b_n - 1) * sizeof(real_T));
        }
      }

      root_model_xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
                        qrmanager->Q.data, qrmanager->Q.size, qrmanager->ldq,
                        qrmanager->tau.data);
      rankQR = root_model_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (b_idx = 0; b_idx < mWConstr; b_idx++) {
        workspace_data[b_idx] = workingset->bwset.data[b_idx];
        workspace_data[b_idx + workspace_size[0]] = workingset->bwset.data[b_idx];
      }

      ix = workingset->ldA;
      iQR0 = 0;
      b_n = (workingset->nActiveConstr - 1) * workingset->ldA + 1;
      for (ldq = 1; ix < 0 ? ldq >= b_n : ldq <= b_n; ldq += ix) {
        temp = 0.0;
        jBcol = ldq + nVar;
        for (b_idx = ldq; b_idx < jBcol; b_idx++) {
          temp += workingset->ATwset.data[b_idx - 1] * xCurrent_data[b_idx - ldq];
        }

        workspace_data[iQR0] -= temp;
        iQR0++;
      }

      ldq = qrmanager->ldq;
      ix = workspace_size[0];
      b_idx = workspace_size[0] * workspace_size[1];
      if (b_idx - 1 >= 0) {
        std::memcpy(&root_model_B.B_data[0], &workspace_data[0],
                    static_cast<uint32_T>(b_idx) * sizeof(real_T));
      }

      for (b_idx = 0; ix < 0 ? b_idx >= ix : b_idx <= ix; b_idx += ix) {
        b_n = b_idx + nVar;
        for (iQR0 = b_idx + 1; iQR0 <= b_n; iQR0++) {
          workspace_data[iQR0 - 1] = 0.0;
        }
      }

      br = -1;
      for (b_n = 0; ix < 0 ? b_n >= ix : b_n <= ix; b_n += ix) {
        iAcol = -1;
        n = b_n + nVar;
        for (iQR0 = b_n + 1; iQR0 <= n; iQR0++) {
          temp = 0.0;
          for (jBcol = 0; jBcol < mWConstr; jBcol++) {
            temp += qrmanager->Q.data[(jBcol + iAcol) + 1] *
              root_model_B.B_data[(jBcol + br) + 1];
          }

          workspace_data[iQR0 - 1] += temp;
          iAcol += ldq;
        }

        br += ix;
      }

      for (b_n = 0; b_n < 2; b_n++) {
        br = ix * b_n - 1;
        for (iQR0 = rankQR; iQR0 >= 1; iQR0--) {
          iAcol = (iQR0 - 1) * ldq;
          b_idx = iQR0 + br;
          temp = workspace_data[b_idx];
          if (temp != 0.0) {
            workspace_data[b_idx] = temp / qrmanager->QR.data[(iQR0 + iAcol) - 1];
            n = static_cast<uint16_T>(iQR0 - 1);
            for (jBcol = 0; jBcol < n; jBcol++) {
              mWConstr = (jBcol + br) + 1;
              workspace_data[mWConstr] -= qrmanager->QR.data[jBcol + iAcol] *
                workspace_data[b_idx];
            }
          }
        }
      }

      for (b_idx = rankQR + 1; b_idx <= nVar; b_idx++) {
        workspace_data[b_idx - 1] = 0.0;
        workspace_data[(b_idx + workspace_size[0]) - 1] = 0.0;
      }

      for (rankQR = 0; rankQR < scalarLB; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR];
      }

      for (rankQR = 0; rankQR < scalarLB; rankQR++) {
        workspace_data[rankQR] = workspace_data[(workspace_size[0] << 1) +
          rankQR];
      }

      for (rankQR = 0; rankQR < scalarLB; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR + workspace_size[0]];
      }

      for (rankQR = 0; rankQR < scalarLB; rankQR++) {
        workspace_data[rankQR + workspace_size[0]] = workspace_data
          [(workspace_size[0] << 1) + rankQR];
      }
    } else {
      if (workingset->nActiveConstr - 1 >= 0) {
        std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint32_T>
                    (workingset->nActiveConstr) * sizeof(int32_T));
      }

      root_model_factorQRE(qrmanager, workingset->ATwset.data, workingset->nVar,
                           workingset->nActiveConstr, workingset->ldA);
      ix = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < ix; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR + rankQR;
        b_n = qrmanager->mrows - rankQR;
        if (b_n - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0 + 1],
                      static_cast<uint32_T>(b_n - 1) * sizeof(real_T));
        }
      }

      root_model_xorgqr(qrmanager->mrows, qrmanager->minRowCol,
                        qrmanager->minRowCol, qrmanager->Q.data,
                        qrmanager->Q.size, qrmanager->ldq, qrmanager->tau.data);
      rankQR = root_model_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (b_idx = 0; b_idx < mWConstr; b_idx++) {
        ix = (qrmanager->jpvt.data[b_idx] - 1) * workingset->ldA;
        temp = 0.0;
        b_n = static_cast<uint16_T>(nVar);
        for (ldq = 0; ldq < b_n; ldq++) {
          temp += workingset->ATwset.data[ix + ldq] * xCurrent_data[ldq];
        }

        workspace_data[b_idx] = workingset->bwset.data[qrmanager->
          jpvt.data[b_idx] - 1] - temp;
        workspace_data[b_idx + workspace_size[0]] = workingset->
          bwset.data[qrmanager->jpvt.data[b_idx] - 1];
      }

      ldq = qrmanager->ldq;
      ix = workspace_size[0];
      br = static_cast<uint16_T>(rankQR);
      for (mWConstr = 0; mWConstr < 2; mWConstr++) {
        jBcol = ix * mWConstr;
        for (b_n = 0; b_n < br; b_n++) {
          iAcol = ldq * b_n;
          b_idx = b_n + jBcol;
          temp = workspace_data[b_idx];
          for (iQR0 = 0; iQR0 < b_n; iQR0++) {
            temp -= qrmanager->QR.data[iQR0 + iAcol] * workspace_data[iQR0 +
              jBcol];
          }

          workspace_data[b_idx] = temp / qrmanager->QR.data[b_n + iAcol];
        }
      }

      b_idx = workspace_size[0] * workspace_size[1];
      if (b_idx - 1 >= 0) {
        std::memcpy(&root_model_B.B_data[0], &workspace_data[0], static_cast<
                    uint32_T>(b_idx) * sizeof(real_T));
      }

      for (b_idx = 0; ix < 0 ? b_idx >= ix : b_idx <= ix; b_idx += ix) {
        iQR0 = b_idx + nVar;
        for (mWConstr = b_idx + 1; mWConstr <= iQR0; mWConstr++) {
          workspace_data[mWConstr - 1] = 0.0;
        }
      }

      br = 1;
      for (b_n = 0; ix < 0 ? b_n >= ix : b_n <= ix; b_n += ix) {
        iAcol = -1;
        n = br + rankQR;
        for (iQR0 = br; iQR0 < n; iQR0++) {
          mWConstr = b_n + nVar;
          scalarLB = ((((mWConstr - b_n) / 2) << 1) + b_n) + 1;
          b_idx = scalarLB - 2;
          for (jBcol = b_n + 1; jBcol <= b_idx; jBcol += 2) {
            tmp = _mm_loadu_pd(&qrmanager->Q.data[(iAcol + jBcol) - b_n]);
            tmp_0 = _mm_loadu_pd(&workspace_data[jBcol - 1]);
            _mm_storeu_pd(&workspace_data[jBcol - 1], _mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(root_model_B.B_data[iQR0 - 1]), tmp), tmp_0));
          }

          for (jBcol = scalarLB; jBcol <= mWConstr; jBcol++) {
            workspace_data[jBcol - 1] += qrmanager->Q.data[(iAcol + jBcol) - b_n]
              * root_model_B.B_data[iQR0 - 1];
          }

          iAcol += ldq;
        }

        br += ix;
      }
    }

    rankQR = 0;
    do {
      exitg1 = 0;
      if (rankQR <= static_cast<uint16_T>(nVar) - 1) {
        temp = workspace_data[rankQR];
        if (std::isinf(temp) || std::isnan(temp)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          temp = workspace_data[rankQR + workspace_size[0]];
          if (std::isinf(temp) || std::isnan(temp)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            rankQR++;
          }
        }
      } else {
        scalarLB = (nVar / 2) << 1;
        b_idx = scalarLB - 2;
        for (rankQR = 0; rankQR <= b_idx; rankQR += 2) {
          tmp = _mm_loadu_pd(&workspace_data[rankQR]);
          tmp_0 = _mm_loadu_pd(&xCurrent_data[rankQR]);
          _mm_storeu_pd(&workspace_data[rankQR], _mm_add_pd(tmp, tmp_0));
        }

        for (rankQR = scalarLB; rankQR < nVar; rankQR++) {
          workspace_data[rankQR] += xCurrent_data[rankQR];
        }

        temp = root_m_maxConstraintViolation_i(workingset, workspace_data);
        constrViolation_basicX = root__maxConstraintViolation_i0(workingset,
          workspace_data, workspace_size[0] + 1);
        if ((temp <= 2.2204460492503131E-16) || (temp < constrViolation_basicX))
        {
          std::memcpy(&xCurrent_data[0], &workspace_data[0],
                      static_cast<uint16_T>(nVar) * sizeof(real_T));
        } else {
          rankQR = static_cast<uint16_T>(nVar);
          for (nVar = 0; nVar < rankQR; nVar++) {
            xCurrent_data[nVar] = workspace_data[workspace_size[0] + nVar];
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mod_RemoveDependentIneq__h(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  int32_T b;
  int32_T d;
  int32_T ix0;
  int32_T iy0;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      nDepIneq = workingset->nVar;
    } else {
      nDepIneq = workingset->nActiveConstr;
    }

    tol = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 *
                    static_cast<real_T>(nDepIneq)) * 10.0;
    for (nDepIneq = 0; nDepIneq < nFixedConstr; nDepIneq++) {
      qrmanager->jpvt.data[nDepIneq] = 1;
    }

    if (nFixedConstr + 1 <= workingset->nActiveConstr) {
      std::memset(&qrmanager->jpvt.data[nFixedConstr], 0, static_cast<uint32_T>
                  (workingset->nActiveConstr - nFixedConstr) * sizeof(int32_T));
    }

    b = workingset->nActiveConstr;
    for (nDepIneq = 0; nDepIneq < b; nDepIneq++) {
      iy0 = qrmanager->ldq * nDepIneq;
      ix0 = workingset->ldA * nDepIneq;
      d = static_cast<uint16_T>(nVar);
      for (nActiveConstr = 0; nActiveConstr < d; nActiveConstr++) {
        qrmanager->QR.data[iy0 + nActiveConstr] = workingset->ATwset.data[ix0 +
          nActiveConstr];
      }
    }

    root_model_factorQRE_h(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (nActiveConstr = workingset->nActiveConstr - 1; nActiveConstr + 1 > nVar;
         nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
        jpvt.data[nActiveConstr];
    }

    maxDiag = std::abs(qrmanager->QR.data[0]);
    for (nVar = 0; nVar < nActiveConstr; nVar++) {
      maxDiag = std::fmax(maxDiag, std::abs(qrmanager->QR.data[((nVar + 1) *
        qrmanager->ldq + nVar) + 1]));
    }

    if (nActiveConstr + 1 <= workingset->nVar) {
      nVar = qrmanager->ldq * nActiveConstr + nActiveConstr;
      while ((nActiveConstr + 1 > nFixedConstr) && (std::abs(qrmanager->
               QR.data[nVar]) < tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
          jpvt.data[nActiveConstr];
        nActiveConstr--;
        nVar = (nVar - qrmanager->ldq) - 1;
      }
    }

    root_model_countsort(memspace->workspace_int.data, nDepIneq,
                         memspace->workspace_sort.data, nFixedConstr + 1,
                         workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      root_model_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_it3(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T scalarLB;
    int32_T vectorUB;
    b = static_cast<uint16_T>(n);
    scalarLB = (static_cast<uint16_T>(n) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T b_iy{0}; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (int32_T b_iy{scalarLB}; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy +=
         lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = b_iy + m;
      for (b = b_iy; b < e; b++) {
        c += A_data[b - 1] * x_data[b - b_iy];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T maxConstraintViolation_AMats_no(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[])
{
  real_T v;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size[0] != 0) {
    if (mIneq - 1 >= 0) {
      std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                  static_cast<uint32_T>(mIneq) * sizeof(real_T));
    }

    root_model_xgemv_it3(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
                         x_data, obj->maxConstrWorkspace.data);
    mIneq = static_cast<uint16_T>(obj->sizes[2]);
    for (k = 0; k < mIneq; k++) {
      v = std::fmax(v, obj->maxConstrWorkspace.data[k]);
    }
  }

  std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
              (real_T));
  root_model_xgemv_it3(obj->nVar, 170, obj->Aeq.data, obj->ldA, x_data,
                       obj->maxConstrWorkspace.data);
  for (k = 0; k < 170; k++) {
    v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T maxConstraintViolation_AMats_re(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size[0] != 0) {
    if (mIneq - 1 >= 0) {
      std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                  static_cast<uint32_T>(mIneq) * sizeof(real_T));
    }

    root_model_xgemv_it3(177, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
                         obj->maxConstrWorkspace.data);
    b = static_cast<uint16_T>(obj->sizes[2]);
    for (k = 0; k < b; k++) {
      obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[k] - x_data[k + 177];
      obj->maxConstrWorkspace.data[k] = obj_maxConstrWorkspace;
      v = std::fmax(v, obj_maxConstrWorkspace);
    }
  }

  std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 170U * sizeof
              (real_T));
  root_model_xgemv_it3(177, 170, obj->Aeq.data, obj->ldA, x_data,
                       obj->maxConstrWorkspace.data);
  for (k = 0; k < 170; k++) {
    obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] - x_data
      [(mIneq + k) + 177]) + x_data[(obj->sizes[2] + k) + 347];
    v = std::fmax(v, std::abs(obj->maxConstrWorkspace.data[k]));
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_maxConstraintViolation_i0u(s_Jrk1gF3jitJmkPBgphEknE_root_T
  *obj, const real_T x_data[])
{
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    v = maxConstraintViolation_AMats_re(obj, x_data);
  } else {
    v = maxConstraintViolation_AMats_no(obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = static_cast<uint16_T>(obj->sizes[3]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, -x_data[obj->indexLB.data[idx] - 1] - obj->lb.data
                    [obj->indexLB.data[idx] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    b = static_cast<uint16_T>(obj->sizes[4]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, x_data[obj->indexUB.data[idx] - 1] - obj->ub.data
                    [obj->indexUB.data[idx] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    b = static_cast<uint16_T>(obj->sizes[0]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, std::abs(x_data[obj->indexFixed.data[idx] - 1] -
        obj->ub.data[obj->indexFixed.data[idx] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_PresolveWorkingSet(s_7iu1rzNoAoHigERqeOFm2G_root_T
  *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace,
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager)
{
  real_T constrViolation;
  int32_T b;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  boolean_T guard1;
  boolean_T okWorkingSet;
  solution->state = 82;
  b = root_model_RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((b != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    root_model_RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = root_mo_feasibleX0ForWorkingSet
      (memspace->workspace_float.data, memspace->workspace_float.size,
       solution->xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      root_mod_RemoveDependentIneq__h(workingset, qrmanager, memspace);
      okWorkingSet = root_mo_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (workingset->nWConstr[0] + workingset->nWConstr[1] == workingset->nVar)
      {
        constrViolation = root_maxConstraintViolation_i0u(workingset,
          solution->xstar.data);
        if (constrViolation > 1.0E-6) {
          solution->state = -2;
        }
      }
    }
  } else {
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (b = idxStartIneq; b <= idxEndIneq; b++) {
      workingset->isActiveConstr.data[(workingset->isActiveIdx
        [workingset->Wid.data[b - 1] - 1] + workingset->Wlocalidx.data[b - 1]) -
        2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_it3a(int32_T m, int32_T n, const real_T A[31329],
  int32_T lda, const real_T x_data[], real_T y_data[])
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T ix;
    if (m - 1 >= 0) {
      std::memset(&y_data[0], 0, static_cast<uint32_T>(m) * sizeof(real_T));
    }

    ix = 0;
    b = (n - 1) * lda + 1;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = b_iy + m;
      for (int32_T ia{b_iy}; ia < c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_computeGrad_StoreHx(s_1ECtvwuWb7KTJMqOG7pbsD_root_T *obj,
  const real_T H[31329], const real_T f_data[], const real_T x_data[])
{
  __m128d tmp_0;
  int32_T i;
  int32_T ixlast;
  int32_T iy;
  int32_T scalarLB;
  int32_T tmp;
  int32_T vectorUB;
  switch (obj->objtype) {
   case 5:
    if (obj->nvar - 2 >= 0) {
      std::memset(&obj->grad.data[0], 0, static_cast<uint32_T>(obj->nvar - 1) *
                  sizeof(real_T));
    }

    obj->grad.data[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    root_model_xgemv_it3a(obj->nvar, obj->nvar, H, obj->nvar, x_data,
                          obj->Hx.data);
    if (obj->nvar - 1 >= 0) {
      std::memcpy(&obj->grad.data[0], &obj->Hx.data[0], static_cast<uint32_T>
                  (obj->nvar) * sizeof(real_T));
    }

    if (obj->hasLinear) {
      iy = obj->grad.size[0];
      i = obj->grad.size[0];
      if (i - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_p[0], &obj->grad.data[0],
                    static_cast<uint32_T>(i) * sizeof(real_T));
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        scalarLB = (obj->nvar / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i = 0; i <= vectorUB; i += 2) {
          tmp_0 = _mm_loadu_pd(&root_model_B.y_data_p[i]);
          _mm_storeu_pd(&root_model_B.y_data_p[i], _mm_add_pd(tmp_0,
            _mm_loadu_pd(&f_data[i])));
        }

        for (i = scalarLB; i < ixlast; i++) {
          root_model_B.y_data_p[i] += f_data[i];
        }
      }

      if (iy - 1 >= 0) {
        std::memcpy(&obj->grad.data[0], &root_model_B.y_data_p[0],
                    static_cast<uint32_T>(iy) * sizeof(real_T));
      }
    }
    break;

   case 4:
    ixlast = obj->maxVar;
    root_model_xgemv_it3a(obj->nvar, obj->nvar, H, obj->nvar, x_data,
                          obj->Hx.data);
    iy = obj->nvar + 1;
    scalarLB = (((((obj->maxVar - obj->nvar) - 1) / 2) << 1) + obj->nvar) + 1;
    vectorUB = scalarLB - 2;
    for (i = iy; i <= vectorUB; i += 2) {
      _mm_storeu_pd(&obj->Hx.data[i - 1], _mm_mul_pd(_mm_loadu_pd(&x_data[i - 1]),
        _mm_set1_pd(obj->beta)));
    }

    for (i = scalarLB; i < ixlast; i++) {
      obj->Hx.data[i - 1] = x_data[i - 1] * obj->beta;
    }

    std::memcpy(&obj->grad.data[0], &obj->Hx.data[0], static_cast<uint16_T>
                (obj->maxVar - 1) * sizeof(real_T));
    if (obj->hasLinear) {
      iy = obj->grad.size[0];
      i = obj->grad.size[0];
      if (i - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_p[0], &obj->grad.data[0],
                    static_cast<uint32_T>(i) * sizeof(real_T));
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        scalarLB = (obj->nvar / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i = 0; i <= vectorUB; i += 2) {
          tmp_0 = _mm_loadu_pd(&root_model_B.y_data_p[i]);
          _mm_storeu_pd(&root_model_B.y_data_p[i], _mm_add_pd(tmp_0,
            _mm_loadu_pd(&f_data[i])));
        }

        for (i = scalarLB; i < ixlast; i++) {
          root_model_B.y_data_p[i] += f_data[i];
        }
      }

      if (iy - 1 >= 0) {
        std::memcpy(&obj->grad.data[0], &root_model_B.y_data_p[0],
                    static_cast<uint32_T>(iy) * sizeof(real_T));
      }
    }

    ixlast = (obj->maxVar - obj->nvar) - 1;
    if (ixlast >= 1) {
      iy = obj->nvar;
      scalarLB = (ixlast / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        tmp = iy + i;
        tmp_0 = _mm_loadu_pd(&obj->grad.data[tmp]);
        _mm_storeu_pd(&obj->grad.data[tmp], _mm_add_pd(tmp_0, _mm_set1_pd
          (obj->rho)));
      }

      for (i = scalarLB; i < ixlast; i++) {
        tmp = iy + i;
        obj->grad.data[tmp] += obj->rho;
      }
    }
    break;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_computeFval_ReuseHx(const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *obj, real_T workspace_data[], const real_T
  f_data[], const real_T x_data[])
{
  real_T val;
  val = 0.0;
  switch (obj->objtype) {
   case 5:
    val = x_data[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T ixlast;
        int32_T scalarLB;
        int32_T vectorUB;
        ixlast = obj->nvar;
        scalarLB = (obj->nvar / 2) << 1;
        vectorUB = scalarLB - 2;
        for (int32_T maxRegVar{0}; maxRegVar <= vectorUB; maxRegVar += 2) {
          __m128d tmp_0;
          tmp_0 = _mm_loadu_pd(&obj->Hx.data[maxRegVar]);
          _mm_storeu_pd(&workspace_data[maxRegVar], _mm_add_pd(_mm_mul_pd
            (_mm_set1_pd(0.5), tmp_0), _mm_loadu_pd(&f_data[maxRegVar])));
        }

        for (int32_T maxRegVar{scalarLB}; maxRegVar < ixlast; maxRegVar++) {
          workspace_data[maxRegVar] = 0.5 * obj->Hx.data[maxRegVar] +
            f_data[maxRegVar];
        }

        if (obj->nvar >= 1) {
          int32_T c_ixlast;
          c_ixlast = obj->nvar;
          for (ixlast = 0; ixlast < c_ixlast; ixlast++) {
            val += workspace_data[ixlast] * x_data[ixlast];
          }
        }
      } else {
        if (obj->nvar >= 1) {
          int32_T ixlast;
          ixlast = obj->nvar;
          for (int32_T maxRegVar{0}; maxRegVar < ixlast; maxRegVar++) {
            val += x_data[maxRegVar] * obj->Hx.data[maxRegVar];
          }
        }

        val *= 0.5;
      }
    }
    break;

   case 4:
    {
      int32_T maxRegVar;
      maxRegVar = obj->maxVar;
      if (obj->hasLinear) {
        int32_T c_ixlast;
        int32_T ixlast;
        int32_T scalarLB;
        int32_T vectorUB;
        if (obj->nvar - 1 >= 0) {
          std::memcpy(&workspace_data[0], &f_data[0], static_cast<uint32_T>
                      (obj->nvar) * sizeof(real_T));
        }

        ixlast = obj->maxVar - obj->nvar;
        for (maxRegVar = 0; maxRegVar <= ixlast - 2; maxRegVar++) {
          workspace_data[obj->nvar + maxRegVar] = obj->rho;
        }

        c_ixlast = static_cast<uint16_T>(obj->maxVar - 1);
        scalarLB = (static_cast<uint16_T>(obj->maxVar - 1) / 2) << 1;
        vectorUB = scalarLB - 2;
        for (ixlast = 0; ixlast <= vectorUB; ixlast += 2) {
          __m128d tmp;
          __m128d tmp_0;
          tmp_0 = _mm_loadu_pd(&obj->Hx.data[ixlast]);
          tmp = _mm_loadu_pd(&workspace_data[ixlast]);
          _mm_storeu_pd(&workspace_data[ixlast], _mm_add_pd(tmp, _mm_mul_pd
            (_mm_set1_pd(0.5), tmp_0)));
        }

        for (ixlast = scalarLB; ixlast < c_ixlast; ixlast++) {
          workspace_data[ixlast] += 0.5 * obj->Hx.data[ixlast];
        }

        if (obj->maxVar - 1 >= 1) {
          c_ixlast = obj->maxVar;
          for (ixlast = 0; ixlast <= c_ixlast - 2; ixlast++) {
            val += workspace_data[ixlast] * x_data[ixlast];
          }
        }
      } else {
        int32_T c_ixlast;
        if (obj->maxVar - 1 >= 1) {
          c_ixlast = obj->maxVar;
          for (int32_T ixlast{0}; ixlast <= c_ixlast - 2; ixlast++) {
            val += x_data[ixlast] * obj->Hx.data[ixlast];
          }
        }

        val *= 0.5;
        c_ixlast = obj->nvar + 1;
        for (int32_T ixlast{c_ixlast}; ixlast < maxRegVar; ixlast++) {
          val += x_data[ixlast - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgeqrf(real_T A_data[], const int32_T A_size[2], int32_T
  m, int32_T n, real_T tau_data[], int32_T tau_size[1])
{
  int32_T minmana;
  int32_T minmn;
  if (A_size[0] <= A_size[1]) {
    minmana = A_size[0];
  } else {
    minmana = A_size[1];
  }

  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  tau_size[0] = minmana;
  if (minmana - 1 >= 0) {
    std::memset(&tau_data[0], 0, static_cast<uint32_T>(minmana) * sizeof(real_T));
  }

  if (minmn >= 1) {
    root_model_qrf(A_data, A_size, m, n, minmn, tau_data);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factorQR(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj, const
  real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  static const int32_T offsets[4]{ 0, 1, 2, 3 };

  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = static_cast<uint16_T>(mrows);
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    k = (ncols / 4) << 2;
    ix0 = k - 4;
    for (idx = 0; idx <= ix0; idx += 4) {
      _mm_storeu_si128((__m128i *)&obj->jpvt.data[idx], _mm_add_epi32
                       (_mm_add_epi32(_mm_set1_epi32(idx), _mm_loadu_si128((
        const __m128i *)&offsets[0])), _mm_set1_epi32(1)));
    }

    for (idx = k; idx < ncols; idx++) {
      obj->jpvt.data[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    root_model_xgeqrf(obj->QR.data, obj->QR.size, mrows, ncols, obj->tau.data,
                      obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_squareQ_appendCol(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj,
  const real_T vec_data[], int32_T iv0)
{
  real_T b_c;
  real_T s;
  real_T temp;
  int32_T b_iy;
  int32_T ia;
  int32_T idx;
  int32_T idxRotGCol;
  int32_T iy;
  int32_T iyend;
  int32_T temp_tmp;
  if (obj->mrows <= obj->ncols + 1) {
    obj->minRowCol = obj->mrows;
  } else {
    obj->minRowCol = obj->ncols + 1;
  }

  b_iy = obj->ldq * obj->ncols;
  idx = obj->ldq;
  if (obj->mrows != 0) {
    iyend = b_iy + obj->mrows;
    if (b_iy + 1 <= iyend) {
      std::memset(&obj->QR.data[b_iy], 0, static_cast<uint32_T>(iyend - b_iy) *
                  sizeof(real_T));
    }

    iy = (obj->mrows - 1) * obj->ldq + 1;
    for (iyend = 1; idx < 0 ? iyend >= iy : iyend <= iy; iyend += idx) {
      b_c = 0.0;
      idxRotGCol = iyend + obj->mrows;
      for (ia = iyend; ia < idxRotGCol; ia++) {
        b_c += vec_data[((iv0 + ia) - iyend) - 1] * obj->Q.data[ia - 1];
      }

      obj->QR.data[b_iy] += b_c;
      b_iy++;
    }
  }

  obj->ncols++;
  obj->jpvt.data[obj->ncols - 1] = obj->ncols;
  for (idx = obj->mrows - 2; idx + 2 > obj->ncols; idx--) {
    iyend = (obj->ncols - 1) * obj->ldq + idx;
    temp = obj->QR.data[iyend + 1];
    xrotg_FR4eOElQ(&obj->QR.data[iyend], &temp, &b_c, &s);
    obj->QR.data[iyend + 1] = temp;
    idxRotGCol = obj->ldq * idx;
    b_iy = obj->mrows;
    if (obj->mrows >= 1) {
      iy = obj->ldq + idxRotGCol;
      for (ia = 0; ia < b_iy; ia++) {
        iyend = iy + ia;
        temp_tmp = idxRotGCol + ia;
        temp = obj->Q.data[temp_tmp] * b_c + obj->Q.data[iyend] * s;
        obj->Q.data[iyend] = obj->Q.data[iyend] * b_c - obj->Q.data[temp_tmp] *
          s;
        obj->Q.data[temp_tmp] = temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_deleteColMoveEnd(s_Pct27jAhkqchx6iRs2zKpD_root_T *obj,
  int32_T idx)
{
  real_T b_s;
  real_T b_temp;
  real_T c_c;
  int32_T QRk0;
  int32_T b_ix;
  int32_T b_temp_tmp;
  int32_T b_temp_tmp_0;
  int32_T c_ix;
  int32_T e;
  int32_T endIdx;
  int32_T i;
  int32_T idxRotGCol;
  int32_T ix;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt.data[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt.data[idx - 1] = obj->jpvt.data[obj->ncols - 1];
    e = obj->minRowCol;
    for (endIdx = 0; endIdx < e; endIdx++) {
      obj->QR.data[endIdx + obj->ldq * (idx - 1)] = obj->QR.data[(obj->ncols - 1)
        * obj->ldq + endIdx];
    }

    obj->ncols--;
    if (obj->mrows <= obj->ncols) {
      obj->minRowCol = obj->mrows;
    } else {
      obj->minRowCol = obj->ncols;
    }

    if (idx < obj->mrows) {
      if (obj->mrows - 1 <= obj->ncols) {
        endIdx = obj->mrows - 1;
      } else {
        endIdx = obj->ncols;
      }

      e = endIdx;
      idxRotGCol = (idx - 1) * obj->ldq;
      while (e >= idx) {
        b_temp_tmp = e + idxRotGCol;
        b_temp = obj->QR.data[b_temp_tmp];
        xrotg_FR4eOElQ(&obj->QR.data[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[b_temp_tmp] = b_temp;
        obj->QR.data[e + obj->ldq * (e - 1)] = 0.0;
        QRk0 = obj->ldq * idx + e;
        b_ix = obj->ncols - idx;
        if (b_ix >= 1) {
          ix = QRk0 - 1;
          for (c_ix = 0; c_ix < b_ix; c_ix++) {
            b_temp = obj->QR.data[ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[ix] *
              b_s;
            obj->QR.data[ix] = b_temp;
            QRk0 += obj->ldq;
            ix += obj->ldq;
          }
        }

        i = (e - 1) * obj->ldq;
        QRk0 = obj->mrows;
        if (obj->mrows >= 1) {
          ix = obj->ldq + i;
          for (b_ix = 0; b_ix < QRk0; b_ix++) {
            b_temp_tmp = ix + b_ix;
            b_temp_tmp_0 = i + b_ix;
            b_temp = obj->Q.data[b_temp_tmp_0] * c_c + obj->Q.data[b_temp_tmp] *
              b_s;
            obj->Q.data[b_temp_tmp] = obj->Q.data[b_temp_tmp] * c_c -
              obj->Q.data[b_temp_tmp_0] * b_s;
            obj->Q.data[b_temp_tmp_0] = b_temp;
          }
        }

        e--;
      }

      for (e = idx + 1; e <= endIdx; e++) {
        b_temp_tmp = (e - 1) * obj->ldq + e;
        b_temp = obj->QR.data[b_temp_tmp];
        xrotg_FR4eOElQ(&obj->QR.data[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[b_temp_tmp] = b_temp;
        QRk0 = (obj->ldq + 1) * e;
        c_ix = obj->ncols - e;
        if (c_ix >= 1) {
          b_ix = QRk0 - 1;
          for (idxRotGCol = 0; idxRotGCol < c_ix; idxRotGCol++) {
            b_temp = obj->QR.data[b_ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[b_ix] *
              b_s;
            obj->QR.data[b_ix] = b_temp;
            QRk0 += obj->ldq;
            b_ix += obj->ldq;
          }
        }

        i = (e - 1) * obj->ldq;
        idxRotGCol = obj->mrows;
        if (obj->mrows >= 1) {
          b_ix = obj->ldq + i;
          for (c_ix = 0; c_ix < idxRotGCol; c_ix++) {
            b_temp_tmp = b_ix + c_ix;
            b_temp_tmp_0 = i + c_ix;
            b_temp = obj->Q.data[b_temp_tmp_0] * c_c + obj->Q.data[b_temp_tmp] *
              b_s;
            obj->Q.data[b_temp_tmp] = obj->Q.data[b_temp_tmp] * c_c -
              obj->Q.data[b_temp_tmp_0] * b_s;
            obj->Q.data[b_temp_tmp_0] = b_temp;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static boolean_T root_model_strcmp(const char_T a[7])
{
  int32_T ret;
  static const char_T b_b[7]{ 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  ret = memcmp(&a[0], &b_b[0], 7);
  return ret == 0;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemm(int32_T m, int32_T n, int32_T k, const real_T A
  [31329], int32_T lda, const real_T B_data[], int32_T ib0, int32_T ldb, real_T
  C_data[], int32_T ldc)
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T br;
    int32_T lastColC;
    br = ib0;
    lastColC = (n - 1) * ldc;
    for (int32_T cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      b = cr + m;
      for (int32_T ic{cr + 1}; ic <= b; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    for (int32_T cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = -1;
      c = br + k;
      for (int32_T ic{br}; ic < c; ic++) {
        int32_T d;
        int32_T scalarLB;
        int32_T vectorUB;
        d = cr + m;
        scalarLB = ((((d - cr) / 2) << 1) + cr) + 1;
        vectorUB = scalarLB - 2;
        for (b = cr + 1; b <= vectorUB; b += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&C_data[b - 1]);
          _mm_storeu_pd(&C_data[b - 1], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
            (B_data[ic - 1]), _mm_loadu_pd(&A[(ar + b) - cr])), tmp));
        }

        for (b = scalarLB; b <= d; b++) {
          C_data[b - 1] += A[(ar + b) - cr] * B_data[ic - 1];
        }

        ar += lda;
      }

      br += ldb;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemm_h(int32_T m, int32_T n, int32_T k, const real_T
  A_data[], int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb, real_T
  C_data[], int32_T ldc)
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T br;
    int32_T lastColC;
    lastColC = (n - 1) * ldc;
    for (int32_T cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      b = cr + m;
      for (int32_T ic{cr + 1}; ic <= b; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    br = -1;
    for (int32_T cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = ia0;
      c = cr + m;
      for (int32_T ic{cr + 1}; ic <= c; ic++) {
        real_T temp;
        temp = 0.0;
        for (b = 0; b < k; b++) {
          temp += A_data[(b + ar) - 1] * B_data[(b + br) + 1];
        }

        C_data[ic - 1] += temp;
        ar += lda;
      }

      br += ldb;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_fullColLDL2_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  int32_T LD_offset, int32_T NColsRemain)
{
  int32_T LDimSizeP1;
  LDimSizeP1 = obj->ldm;
  for (int32_T k{0}; k < NColsRemain; k++) {
    __m128d tmp;
    real_T alpha1;
    real_T y;
    int32_T LD_diagOffset;
    int32_T b_k;
    int32_T scalarLB;
    int32_T subMatrixDim;
    int32_T vectorUB;
    LD_diagOffset = (LDimSizeP1 + 1) * k + LD_offset;
    alpha1 = -1.0 / obj->FMat.data[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    for (b_k = 0; b_k <= subMatrixDim; b_k++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + b_k];
    }

    y = obj->workspace_;
    if (!(alpha1 == 0.0)) {
      int32_T jA;
      jA = LD_diagOffset + LDimSizeP1;
      for (b_k = 0; b_k <= subMatrixDim; b_k++) {
        if (y != 0.0) {
          real_T temp;
          int32_T b;
          temp = y * alpha1;
          b = (subMatrixDim + jA) + 1;
          scalarLB = ((((b - jA) / 2) << 1) + jA) + 1;
          vectorUB = scalarLB - 2;
          for (int32_T alpha1_tmp{jA + 1}; alpha1_tmp <= vectorUB; alpha1_tmp +=
               2) {
            tmp = _mm_loadu_pd(&obj->FMat.data[alpha1_tmp - 1]);
            _mm_storeu_pd(&obj->FMat.data[alpha1_tmp - 1], _mm_add_pd(tmp,
              _mm_set1_pd(obj->workspace_ * temp)));
          }

          for (int32_T alpha1_tmp{scalarLB}; alpha1_tmp <= b; alpha1_tmp++) {
            obj->FMat.data[alpha1_tmp - 1] += obj->workspace_ * temp;
          }
        }

        jA += obj->ldm;
      }
    }

    alpha1 = 1.0 / obj->FMat.data[LD_diagOffset - 1];
    b_k = (LD_diagOffset + subMatrixDim) + 1;
    scalarLB = ((((b_k - LD_diagOffset) / 2) << 1) + LD_diagOffset) + 1;
    vectorUB = scalarLB - 2;
    for (subMatrixDim = LD_diagOffset + 1; subMatrixDim <= vectorUB;
         subMatrixDim += 2) {
      tmp = _mm_loadu_pd(&obj->FMat.data[subMatrixDim - 1]);
      _mm_storeu_pd(&obj->FMat.data[subMatrixDim - 1], _mm_mul_pd(tmp,
        _mm_set1_pd(alpha1)));
    }

    for (subMatrixDim = scalarLB; subMatrixDim <= b_k; subMatrixDim++) {
      obj->FMat.data[subMatrixDim - 1] *= alpha1;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_partialColLDL3_(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  int32_T LD_offset, int32_T NColsRemain)
{
  __m128d tmp;
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T b_idx;
  int32_T c;
  int32_T c_idx;
  int32_T idx;
  int32_T ix;
  int32_T lastColC;
  int32_T lda;
  int32_T subRows;
  LDimSizeP1 = obj->ldm + 1;
  for (int32_T k{0}; k < 48; k++) {
    real_T y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LDimSizeP1 * k + LD_offset) - 1;
    for (b_idx = 0; b_idx <= subRows; b_idx++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + b_idx];
    }

    for (c_idx = 0; c_idx < NColsRemain; c_idx++) {
      obj->workspace2_ = obj->workspace_;
    }

    lda = obj->ldm;
    y = obj->workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      c = (k - 1) * obj->ldm + 1;
      for (b_idx = 1; lda < 0 ? b_idx >= c : b_idx <= c; b_idx += lda) {
        lastColC = b_idx + NColsRemain;
        for (idx = b_idx; idx < lastColC; idx++) {
          y += -obj->FMat.data[ix - 1] * obj->workspace_;
        }

        ix += obj->ldm;
      }
    }

    obj->workspace2_ = y;
    for (c_idx = 0; c_idx < NColsRemain; c_idx++) {
      obj->workspace_ = y;
    }

    for (c_idx = 0; c_idx <= subRows; c_idx++) {
      obj->FMat.data[LD_diagOffset + c_idx] = obj->workspace_;
    }

    c_idx = (subRows / 2) << 1;
    lda = c_idx - 2;
    for (b_idx = 0; b_idx <= lda; b_idx += 2) {
      ix = (b_idx + LD_diagOffset) + 1;
      tmp = _mm_loadu_pd(&obj->FMat.data[ix]);
      _mm_storeu_pd(&obj->FMat.data[ix], _mm_div_pd(tmp, _mm_set1_pd
        (obj->FMat.data[LD_diagOffset])));
    }

    for (b_idx = c_idx; b_idx < subRows; b_idx++) {
      ix = (b_idx + LD_diagOffset) + 1;
      obj->FMat.data[ix] /= obj->FMat.data[LD_diagOffset];
    }
  }

  for (int32_T k{48}; k <= NColsRemain - 1; k += 48) {
    int32_T br;
    br = NColsRemain - k;
    if (br >= 48) {
      subRows = 48;
    } else {
      subRows = br;
    }

    b_idx = k + subRows;
    for (LD_diagOffset = k; LD_diagOffset < b_idx; LD_diagOffset++) {
      idx = b_idx - LD_diagOffset;
      for (lda = 0; lda < 48; lda++) {
        obj->workspace2_ = obj->FMat.data[((LD_offset + LD_diagOffset) + lda *
          obj->ldm) - 1];
      }

      ix = obj->ldm;
      if (idx != 0) {
        c = (obj->ldm * 47 + LD_diagOffset) + 1;
        for (c_idx = LD_diagOffset + 1; ix < 0 ? c_idx >= c : c_idx <= c; c_idx +=
             ix) {
          lastColC = c_idx + idx;
          for (lda = c_idx; lda < lastColC; lda++) {
            /* Check node always fails. would cause program termination and was eliminated */
          }
        }
      }
    }

    if (b_idx < NColsRemain) {
      LD_diagOffset = br - subRows;
      b_idx = ((LD_offset + subRows) + LDimSizeP1 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        ix = (LD_offset + k) + idx * obj->ldm;
        for (lda = 0; lda < subRows; lda++) {
          obj->workspace2_ = obj->FMat.data[(ix + lda) - 1];
        }
      }

      idx = obj->ldm;
      if ((LD_diagOffset != 0) && (subRows != 0)) {
        lastColC = (subRows - 1) * obj->ldm + b_idx;
        br = 0;
        for (subRows = b_idx; idx < 0 ? subRows >= lastColC : subRows <=
             lastColC; subRows += idx) {
          int32_T g;
          br++;
          g = idx * 47 + br;
          for (ix = br; idx < 0 ? ix >= g : ix <= g; ix += idx) {
            int32_T h;
            h = subRows + LD_diagOffset;
            c_idx = ((((h - subRows) / 2) << 1) + subRows) + 1;
            lda = c_idx - 2;
            for (c = subRows + 1; c <= lda; c += 2) {
              tmp = _mm_loadu_pd(&obj->FMat.data[c - 1]);
              _mm_storeu_pd(&obj->FMat.data[c - 1], _mm_add_pd(tmp, _mm_set1_pd(
                -obj->workspace2_ * obj->workspace_)));
            }

            for (c = c_idx; c <= h; c++) {
              obj->FMat.data[c - 1] += -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static int32_T root_model_xpotrf(int32_T n, real_T A_data[], int32_T lda)
{
  int32_T b_j;
  int32_T info;
  boolean_T exitg1;
  info = 0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j <= n - 1)) {
    real_T c;
    real_T ssq;
    int32_T idxA1j;
    int32_T idxAjj;
    int32_T nmj;
    idxA1j = b_j * lda;
    idxAjj = idxA1j + b_j;
    ssq = 0.0;
    if (b_j >= 1) {
      for (nmj = 0; nmj < b_j; nmj++) {
        c = A_data[idxA1j + nmj];
        ssq += c * c;
      }
    }

    ssq = A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A_data[idxAjj] = ssq;
      if (b_j + 1 < n) {
        int32_T b_ix;
        nmj = (n - b_j) - 2;
        b_ix = (idxA1j + lda) + 1;
        idxAjj += lda;
        if ((b_j != 0) && (nmj + 1 != 0)) {
          int32_T b;
          int32_T iy;
          iy = idxAjj;
          b = lda * nmj + b_ix;
          for (int32_T b_iy{b_ix}; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda)
          {
            int32_T d;
            c = 0.0;
            d = b_iy + b_j;
            for (int32_T ia{b_iy}; ia < d; ia++) {
              c += A_data[(idxA1j + ia) - b_iy] * A_data[ia - 1];
            }

            A_data[iy] -= c;
            iy += lda;
          }
        }

        ssq = 1.0 / ssq;
        idxA1j = (lda * nmj + idxAjj) + 1;
        for (nmj = idxAjj + 1; lda < 0 ? nmj >= idxA1j : nmj <= idxA1j; nmj +=
             lda) {
          A_data[nmj - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      A_data[idxAjj] = ssq;
      info = b_j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_it3ax(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (m != 0) {
    int32_T b;
    int32_T ix;
    std::memset(&y_data[0], 0, static_cast<uint32_T>(m) * sizeof(real_T));
    ix = 0;
    b = (n - 1) * lda + ia0;
    for (int32_T b_iy{ia0}; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = b_iy + m;
      for (int32_T ia{b_iy}; ia < c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A_data[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factor_j(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const
  real_T A[31329], int32_T ndims, int32_T ldA)
{
  real_T s;
  real_T smax;
  int32_T A_maxDiag_idx;
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T exitg2;
  int32_T ix;
  int32_T iy0;
  boolean_T exitg1;
  LDimSizeP1 = obj->ldm + 1;
  obj->ndims = ndims;
  for (A_maxDiag_idx = 0; A_maxDiag_idx < ndims; A_maxDiag_idx++) {
    ix = ldA * A_maxDiag_idx;
    iy0 = obj->ldm * A_maxDiag_idx;
    for (LD_diagOffset = 0; LD_diagOffset < ndims; LD_diagOffset++) {
      obj->FMat.data[iy0 + LD_diagOffset] = A[LD_diagOffset + ix];
    }
  }

  if (ndims < 1) {
    A_maxDiag_idx = -1;
  } else {
    A_maxDiag_idx = 0;
    if (ndims > 1) {
      smax = std::abs(obj->FMat.data[0]);
      for (LD_diagOffset = 2; LD_diagOffset <= ndims; LD_diagOffset++) {
        s = std::abs(obj->FMat.data[(LD_diagOffset - 1) * LDimSizeP1]);
        if (s > smax) {
          A_maxDiag_idx = LD_diagOffset - 1;
          smax = s;
        }
      }
    }
  }

  obj->regTol_ = std::fmax(std::abs(obj->FMat.data[obj->ldm * A_maxDiag_idx +
    A_maxDiag_idx]) * 2.2204460492503131E-16, 0.0);
  if (ndims > 128) {
    A_maxDiag_idx = 0;
    exitg1 = false;
    while ((!exitg1) && (A_maxDiag_idx < ndims)) {
      LD_diagOffset = LDimSizeP1 * A_maxDiag_idx + 1;
      ix = ndims - A_maxDiag_idx;
      if (A_maxDiag_idx + 48 <= ndims) {
        root_model_partialColLDL3_(obj, LD_diagOffset, ix);
        A_maxDiag_idx += 48;
      } else {
        root_model_fullColLDL2_(obj, LD_diagOffset, ix);
        exitg1 = true;
      }
    }
  } else {
    root_model_fullColLDL2_(obj, 1, ndims);
  }

  if (obj->ConvexCheck) {
    LDimSizeP1 = 0;
    do {
      exitg2 = 0;
      if (LDimSizeP1 <= ndims - 1) {
        if (obj->FMat.data[obj->ldm * LDimSizeP1 + LDimSizeP1] <= 0.0) {
          obj->info = -LDimSizeP1 - 1;
          exitg2 = 1;
        } else {
          LDimSizeP1++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_factor(s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, const real_T
  A[31329], int32_T ndims, int32_T ldA)
{
  int32_T b_k;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  obj->ndims = ndims;
  for (idx = 0; idx < ndims; idx++) {
    ix0 = ldA * idx;
    iy0 = obj->ldm * idx;
    for (b_k = 0; b_k < ndims; b_k++) {
      obj->FMat.data[iy0 + b_k] = A[b_k + ix0];
    }
  }

  obj->info = root_model_xpotrf(ndims, obj->FMat.data, obj->ldm);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_solve_o(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj,
  real_T rhs_data[])
{
  int32_T jjA;
  int32_T n;
  n = obj->ndims;
  if (obj->ndims != 0) {
    for (int32_T b_j{0}; b_j < n; b_j++) {
      int32_T c;
      jjA = b_j * obj->ldm + b_j;
      c = (n - b_j) - 2;
      for (int32_T b_i{0}; b_i <= c; b_i++) {
        int32_T ix;
        ix = (b_i + b_j) + 1;
        rhs_data[ix] -= obj->FMat.data[(b_i + jjA) + 1] * rhs_data[b_j];
      }
    }
  }

  for (int32_T b_j{0}; b_j < n; b_j++) {
    rhs_data[b_j] /= obj->FMat.data[obj->ldm * b_j + b_j];
  }

  n = obj->ndims;
  if (obj->ndims != 0) {
    for (int32_T b_j{n}; b_j >= 1; b_j--) {
      real_T temp;
      jjA = (b_j - 1) * obj->ldm;
      temp = rhs_data[b_j - 1];
      for (int32_T b_i{n}; b_i >= b_j + 1; b_i--) {
        temp -= obj->FMat.data[(jjA + b_i) - 1] * rhs_data[b_i - 1];
      }

      rhs_data[b_j - 1] = temp;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_solve(const s_mdyENYu4oaiCttMAPwg9iE_root_T *obj, real_T
  rhs_data[])
{
  int32_T jA;
  int32_T n;
  n = obj->ndims;
  if (obj->ndims != 0) {
    for (int32_T j{0}; j < n; j++) {
      real_T temp;
      jA = j * obj->ldm;
      temp = rhs_data[j];
      for (int32_T i{0}; i < j; i++) {
        temp -= obj->FMat.data[jA + i] * rhs_data[i];
      }

      rhs_data[j] = temp / obj->FMat.data[jA + j];
    }
  }

  n = obj->ndims;
  if (obj->ndims != 0) {
    for (int32_T j{n}; j >= 1; j--) {
      jA = ((j - 1) * obj->ldm + j) - 2;
      rhs_data[j - 1] /= obj->FMat.data[jA + 1];
      for (int32_T i{0}; i <= j - 2; i++) {
        int32_T ix;
        ix = (j - i) - 2;
        rhs_data[ix] -= obj->FMat.data[jA - i] * rhs_data[j - 1];
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_compute_deltax(const real_T H[31329],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, const s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *cholmanager, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, boolean_T alwaysPositiveDef)
{
  __m128d tmp;
  real_T s;
  real_T smax;
  int32_T b_jjA;
  int32_T b_mNull;
  int32_T c_mNull;
  int32_T d_ix;
  int32_T exitg2;
  int32_T mNull;
  int32_T nVar;
  int32_T nVars;
  int32_T nullStart;
  int32_T nullStartIdx;
  boolean_T exitg1;
  nVar = qrmanager->mrows - 1;
  mNull = qrmanager->mrows - qrmanager->ncols;
  if (mNull <= 0) {
    if (qrmanager->mrows - 1 >= 0) {
      std::memset(&solution->searchDir.data[0], 0, static_cast<uint32_T>
                  ((qrmanager->mrows - 1) + 1) * sizeof(real_T));
    }
  } else {
    c_mNull = (qrmanager->mrows / 2) << 1;
    d_ix = c_mNull - 2;
    for (nullStartIdx = 0; nullStartIdx <= d_ix; nullStartIdx += 2) {
      tmp = _mm_loadu_pd(&objective->grad.data[nullStartIdx]);
      _mm_storeu_pd(&solution->searchDir.data[nullStartIdx], _mm_mul_pd(tmp,
        _mm_set1_pd(-1.0)));
    }

    for (nullStartIdx = c_mNull; nullStartIdx <= nVar; nullStartIdx++) {
      solution->searchDir.data[nullStartIdx] = -objective->
        grad.data[nullStartIdx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        if (alwaysPositiveDef) {
          root_model_factor(cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        } else {
          root_model_factor_j(cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          root_model_solve(cholmanager, solution->searchDir.data);
        } else {
          root_model_solve_o(cholmanager, solution->searchDir.data);
        }
        break;

       case 4:
        if (alwaysPositiveDef) {
          root_model_factor(cholmanager, H, objective->nvar, objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            root_model_solve(cholmanager, solution->searchDir.data);
            smax = 1.0 / objective->beta;
            nullStartIdx = objective->nvar + 1;
            b_mNull = qrmanager->mrows;
            c_mNull = ((((qrmanager->mrows - objective->nvar) / 2) << 1) +
                       objective->nvar) + 1;
            d_ix = c_mNull - 2;
            for (mNull = nullStartIdx; mNull <= d_ix; mNull += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[mNull - 1]);
              _mm_storeu_pd(&solution->searchDir.data[mNull - 1], _mm_mul_pd(tmp,
                _mm_set1_pd(smax)));
            }

            for (mNull = c_mNull; mNull <= b_mNull; mNull++) {
              solution->searchDir.data[mNull - 1] *= smax;
            }
          }
        }
        break;
      }
    } else {
      nullStartIdx = qrmanager->ldq * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (nullStart = 0; nullStart < mNull; nullStart++) {
          memspace->workspace_float.data[nullStart] = -qrmanager->Q.data
            [(qrmanager->ncols + nullStart) * qrmanager->ldq + nVar];
        }

        root_model_xgemv_it3ax(qrmanager->mrows, mNull, qrmanager->Q.data,
          nullStartIdx, qrmanager->ldq, memspace->workspace_float.data,
          solution->searchDir.data);
      } else {
        if (objective->objtype == 3) {
          root_model_xgemm(qrmanager->mrows, mNull, qrmanager->mrows, H,
                           qrmanager->mrows, qrmanager->Q.data, nullStartIdx,
                           qrmanager->ldq, memspace->workspace_float.data,
                           memspace->workspace_float.size[0]);
          root_model_xgemm_h(mNull, mNull, qrmanager->mrows, qrmanager->Q.data,
                             nullStartIdx, qrmanager->ldq,
                             memspace->workspace_float.data,
                             memspace->workspace_float.size[0],
                             cholmanager->FMat.data, cholmanager->ldm);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          root_model_xgemm(objective->nvar, mNull, objective->nvar, H,
                           objective->nvar, qrmanager->Q.data, nullStartIdx,
                           qrmanager->ldq, memspace->workspace_float.data,
                           memspace->workspace_float.size[0]);
          for (b_mNull = 0; b_mNull < mNull; b_mNull++) {
            b_jjA = objective->nvar + 1;
            c_mNull = ((((nVars - objective->nvar) / 2) << 1) + objective->nvar)
              + 1;
            d_ix = c_mNull - 2;
            for (nullStart = b_jjA; nullStart <= d_ix; nullStart += 2) {
              tmp = _mm_loadu_pd(&qrmanager->Q.data[((b_mNull + qrmanager->ncols)
                * qrmanager->Q.size[0] + nullStart) - 1]);
              _mm_storeu_pd(&memspace->workspace_float.data[(nullStart +
                memspace->workspace_float.size[0] * b_mNull) - 1], _mm_mul_pd
                            (tmp, _mm_set1_pd(objective->beta)));
            }

            for (nullStart = c_mNull; nullStart <= nVars; nullStart++) {
              memspace->workspace_float.data[(nullStart +
                memspace->workspace_float.size[0] * b_mNull) - 1] =
                qrmanager->Q.data[((b_mNull + qrmanager->ncols) *
                                   qrmanager->Q.size[0] + nullStart) - 1] *
                objective->beta;
            }
          }

          root_model_xgemm_h(mNull, mNull, qrmanager->mrows, qrmanager->Q.data,
                             nullStartIdx, qrmanager->ldq,
                             memspace->workspace_float.data,
                             memspace->workspace_float.size[0],
                             cholmanager->FMat.data, cholmanager->ldm);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull;
          cholmanager->info = root_model_xpotrf(mNull, cholmanager->FMat.data,
            cholmanager->ldm);
        } else {
          nullStart = cholmanager->ldm + 1;
          cholmanager->ndims = mNull;
          nVars = 0;
          if (mNull > 1) {
            smax = std::abs(cholmanager->FMat.data[0]);
            for (b_mNull = 2; b_mNull <= mNull; b_mNull++) {
              s = std::abs(cholmanager->FMat.data[(b_mNull - 1) * nullStart]);
              if (s > smax) {
                nVars = b_mNull - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = std::fmax(std::abs(cholmanager->
            FMat.data[cholmanager->ldm * nVars + nVars]) *
            2.2204460492503131E-16, 0.0);
          if (mNull > 128) {
            b_mNull = 0;
            exitg1 = false;
            while ((!exitg1) && (b_mNull < mNull)) {
              nVars = nullStart * b_mNull + 1;
              c_mNull = mNull - b_mNull;
              if (b_mNull + 48 <= mNull) {
                root_model_partialColLDL3_(cholmanager, nVars, c_mNull);
                b_mNull += 48;
              } else {
                root_model_fullColLDL2_(cholmanager, nVars, c_mNull);
                exitg1 = true;
              }
            }
          } else {
            root_model_fullColLDL2_(cholmanager, 1, mNull);
          }

          if (cholmanager->ConvexCheck) {
            b_mNull = 0;
            do {
              exitg2 = 0;
              if (b_mNull <= mNull - 1) {
                if (cholmanager->FMat.data[cholmanager->ldm * b_mNull + b_mNull]
                    <= 0.0) {
                  cholmanager->info = -b_mNull - 1;
                  exitg2 = 1;
                } else {
                  b_mNull++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          c_mNull = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            std::memset(&memspace->workspace_float.data[0], 0,
                        static_cast<uint32_T>(mNull) * sizeof(real_T));
            b_mNull = 0;
            b_jjA = (mNull - 1) * qrmanager->ldq + nullStartIdx;
            for (nullStart = nullStartIdx; c_mNull < 0 ? nullStart >= b_jjA :
                 nullStart <= b_jjA; nullStart += c_mNull) {
              smax = 0.0;
              d_ix = nullStart + nVar;
              for (nVars = nullStart; nVars <= d_ix; nVars++) {
                smax += qrmanager->Q.data[nVars - 1] * objective->
                  grad.data[nVars - nullStart];
              }

              memspace->workspace_float.data[b_mNull] -= smax;
              b_mNull++;
            }
          }

          if (alwaysPositiveDef) {
            c_mNull = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nullStart = 0; nullStart < c_mNull; nullStart++) {
                b_mNull = nullStart * cholmanager->ldm;
                smax = memspace->workspace_float.data[nullStart];
                for (nVars = 0; nVars < nullStart; nVars++) {
                  smax -= cholmanager->FMat.data[b_mNull + nVars] *
                    memspace->workspace_float.data[nVars];
                }

                memspace->workspace_float.data[nullStart] = smax /
                  cholmanager->FMat.data[b_mNull + nullStart];
              }
            }

            b_mNull = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nVars = b_mNull; nVars >= 1; nVars--) {
                b_jjA = ((nVars - 1) * cholmanager->ldm + nVars) - 2;
                memspace->workspace_float.data[nVars - 1] /=
                  cholmanager->FMat.data[b_jjA + 1];
                for (c_mNull = 0; c_mNull <= nVars - 2; c_mNull++) {
                  d_ix = (nVars - c_mNull) - 2;
                  memspace->workspace_float.data[d_ix] -=
                    memspace->workspace_float.data[nVars - 1] *
                    cholmanager->FMat.data[b_jjA - c_mNull];
                }
              }
            }
          } else {
            b_mNull = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nVars = 0; nVars < b_mNull; nVars++) {
                nullStart = nVars * cholmanager->ldm + nVars;
                d_ix = (b_mNull - nVars) - 2;
                for (c_mNull = 0; c_mNull <= d_ix; c_mNull++) {
                  nVar = (c_mNull + nVars) + 1;
                  memspace->workspace_float.data[nVar] -= cholmanager->
                    FMat.data[(c_mNull + nullStart) + 1] *
                    memspace->workspace_float.data[nVars];
                }
              }
            }

            for (nullStart = 0; nullStart < b_mNull; nullStart++) {
              memspace->workspace_float.data[nullStart] /=
                cholmanager->FMat.data[cholmanager->ldm * nullStart + nullStart];
            }

            c_mNull = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nullStart = c_mNull; nullStart >= 1; nullStart--) {
                b_mNull = (nullStart - 1) * cholmanager->ldm;
                smax = memspace->workspace_float.data[nullStart - 1];
                for (nVars = c_mNull; nVars >= nullStart + 1; nVars--) {
                  smax -= cholmanager->FMat.data[(b_mNull + nVars) - 1] *
                    memspace->workspace_float.data[nVars - 1];
                }

                memspace->workspace_float.data[nullStart - 1] = smax;
              }
            }
          }

          root_model_xgemv_it3ax(qrmanager->mrows, mNull, qrmanager->Q.data,
            nullStartIdx, qrmanager->ldq, memspace->workspace_float.data,
            solution->searchDir.data);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_xnrm2_ob(int32_T n, const real_T x_data[])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[0]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      for (kend = 0; kend < n; kend++) {
        real_T absxk;
        absxk = std::abs(x_data[kend]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
      if (std::isnan(y)) {
        kend = 0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (kend <= n - 1) {
            if (std::isnan(x_data[kend])) {
              exitg1 = 1;
            } else {
              kend++;
            }
          } else {
            y = (rtInf);
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_xgemv_it3axg(int32_T m, int32_T n, const real_T A_data[],
  int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T scalarLB;
    int32_T vectorUB;
    b = static_cast<uint16_T>(n);
    scalarLB = (static_cast<uint16_T>(n) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T b_iy{0}; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (int32_T b_iy{scalarLB}; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    b = 0;
    scalarLB = (n - 1) * lda + 1;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= scalarLB : b_iy <= scalarLB; b_iy +=
         lda) {
      real_T c;
      c = 0.0;
      vectorUB = b_iy + m;
      for (int32_T ia{b_iy}; ia < vectorUB; ia++) {
        c += A_data[ia - 1] * x_data[ia - b_iy];
      }

      y_data[b] += c;
      b++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_feasibleratiotest(const real_T solution_xstar_data[],
  const real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
  workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
  real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
  real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
  int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
  boolean_T workingset_isActiveConstr_data[], const int32_T workingset_nWConstr
  [5], boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
  *constrType, int32_T *constrIdx)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T b_c;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T ratio;
  int32_T d;
  int32_T d_tmp;
  int32_T e;
  int32_T f;
  int32_T ia;
  int32_T iyend;
  int32_T ldw;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * root_model_xnrm2_ob(workingset_nVar,
    solution_searchDir_data);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    d_tmp = static_cast<uint16_T>(workingset_sizes[2]);
    if (d_tmp - 1 >= 0) {
      std::memcpy(&workspace_data[0], &workingset_bineq_data[0],
                  static_cast<uint32_T>(d_tmp) * sizeof(real_T));
    }

    root_model_xgemv_it3axg(workingset_nVar, workingset_sizes[2],
      workingset_Aineq_data, workingset_ldA, solution_xstar_data, workspace_data);
    ldw = workspace_size[0];
    if (workingset_sizes[2] != 0) {
      iyend = workspace_size[0] + workingset_sizes[2];
      if (ldw + 1 <= iyend) {
        std::memset(&workspace_data[ldw], 0, static_cast<uint32_T>(iyend - ldw) *
                    sizeof(real_T));
      }

      iyend = workspace_size[0];
      e = (workingset_sizes[2] - 1) * workingset_ldA + 1;
      for (d = 1; workingset_ldA < 0 ? d >= e : d <= e; d += workingset_ldA) {
        b_c = 0.0;
        f = d + workingset_nVar;
        for (ia = d; ia < f; ia++) {
          b_c += workingset_Aineq_data[ia - 1] * solution_searchDir_data[ia - d];
        }

        workspace_data[iyend] += b_c;
        iyend++;
      }
    }

    for (d = 0; d < d_tmp; d++) {
      b_c = workspace_data[ldw + d];
      if ((b_c > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[2] + d) - 1])) {
        alphaTemp = std::fmin(std::abs(workspace_data[d]), 1.0E-6 -
                              workspace_data[d]) / b_c;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = d + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir_data[workingset_nVar - 1],
       solution_xstar_data[workingset_nVar - 1]), _mm_set1_pd(static_cast<real_T>
      (isPhaseOne))));
    b_c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    d = workingset_sizes[3];
    for (ldw = 0; ldw <= d - 2; ldw++) {
      d_tmp = workingset_indexLB_data[ldw];
      alphaTemp = -solution_searchDir_data[d_tmp - 1] - phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[3] + ldw) - 1])) {
        ratio = (-solution_xstar_data[d_tmp - 1] - workingset_lb_data[d_tmp - 1])
          - b_c;
        alphaTemp = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = ldw + 1;
          *newBlocking = true;
        }
      }
    }

    ia = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    b_c = -solution_searchDir_data[ia];
    if ((b_c > denomTol) && (!workingset_isActiveConstr_data
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar_data[ia] - workingset_lb_data[ia];
      alphaTemp = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / b_c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (!isPhaseOne) {
    *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
    *alpha = std::fmin(*alpha, 1.0);
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_m_checkUnboundedOrIllPosed(s_7iu1rzNoAoHigERqeOFm2G_root_T
  *solution, const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective)
{
  if (objective->objtype == 5) {
    if (root_model_xnrm2_ob(objective->nvar, solution->searchDir.data) > 100.0 *
        static_cast<real_T>(objective->nvar) * 1.4901161193847656E-8) {
      solution->state = 3;
    } else {
      solution->state = 4;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_addBoundToActiveSetMatrix_(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T TYPE, int32_T idx_local)
{
  int32_T colOffset;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  colOffset = (obj->nActiveConstr - 1) * obj->ldA - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->ub.data[obj->
      indexUB.data[idx_local - 1] - 1];
  } else {
    idx_bnd_local = obj->indexLB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->lb.data[obj->
      indexLB.data[idx_local - 1] - 1];
  }

  if (static_cast<uint16_T>(idx_bnd_local - 1) - 1 >= 0) {
    std::memset(&obj->ATwset.data[colOffset + 1], 0, static_cast<uint16_T>
                (idx_bnd_local - 1) * sizeof(real_T));
  }

  obj->ATwset.data[idx_bnd_local + colOffset] = static_cast<real_T>(TYPE == 5) *
    2.0 - 1.0;
  if (idx_bnd_local + 1 <= obj->nVar) {
    std::memset(&obj->ATwset.data[(idx_bnd_local + colOffset) + 1], 0,
                static_cast<uint32_T>(((obj->nVar + colOffset) - idx_bnd_local)
      - colOffset) * sizeof(real_T));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset.data[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_addAineqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_local)
{
  int32_T b;
  int32_T iAineq0;
  int32_T iAw0;
  obj->nWConstr[2]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = 3;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = (idx_local - 1) * obj->ldA;
  iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
  b = obj->nVar;
  for (int32_T idx{0}; idx < b; idx++) {
    obj->ATwset.data[iAw0 + idx] = obj->Aineq.data[iAineq0 + idx];
  }

  obj->bwset.data[obj->nActiveConstr - 1] = obj->bineq.data[idx_local - 1];
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_compute_lambda(real_T workspace_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager)
{
  int32_T nActiveConstr;
  nActiveConstr = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    real_T c;
    int32_T b_idx;
    int32_T idxQR;
    boolean_T guard1;
    guard1 = false;
    if (objective->objtype != 4) {
      boolean_T nonDegenerate;
      if (qrmanager->mrows >= qrmanager->ncols) {
        b_idx = qrmanager->mrows;
      } else {
        b_idx = qrmanager->ncols;
      }

      c = std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * static_cast<
                    real_T>(b_idx));
      nonDegenerate = ((qrmanager->mrows > 0) && (qrmanager->ncols > 0));
      if (nonDegenerate) {
        boolean_T guard2;
        b_idx = qrmanager->ncols;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = (qrmanager->ncols - 1) * qrmanager->ldq + qrmanager->mrows;
          while ((b_idx > qrmanager->mrows) && (std::abs(qrmanager->
                   QR.data[idxQR - 1]) >= c)) {
            b_idx--;
            idxQR -= qrmanager->ldq;
          }

          nonDegenerate = (b_idx == qrmanager->mrows);
          if (!nonDegenerate) {
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          idxQR = (b_idx - 1) * qrmanager->ldq + b_idx;
          while ((b_idx >= 1) && (std::abs(qrmanager->QR.data[idxQR - 1]) >= c))
          {
            b_idx--;
            idxQR = (idxQR - qrmanager->ldq) - 1;
          }

          nonDegenerate = (b_idx == 0);
        }
      }

      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      int32_T b_ix;
      int32_T ia;
      int32_T jjA;
      b_idx = qrmanager->ldq;
      if (qrmanager->mrows != 0) {
        int32_T b;
        std::memset(&workspace_data[0], 0, static_cast<uint32_T>
                    (qrmanager->ncols) * sizeof(real_T));
        jjA = 0;
        b = (qrmanager->ncols - 1) * qrmanager->ldq + 1;
        for (idxQR = 1; b_idx < 0 ? idxQR >= b : idxQR <= b; idxQR += b_idx) {
          c = 0.0;
          b_ix = idxQR + qrmanager->mrows;
          for (ia = idxQR; ia < b_ix; ia++) {
            c += qrmanager->Q.data[ia - 1] * objective->grad.data[ia - idxQR];
          }

          workspace_data[jjA] += c;
          jjA++;
        }
      }

      if (qrmanager->ncols != 0) {
        for (idxQR = nActiveConstr; idxQR >= 1; idxQR--) {
          jjA = ((idxQR - 1) * b_idx + idxQR) - 2;
          workspace_data[idxQR - 1] /= qrmanager->QR.data[jjA + 1];
          for (ia = 0; ia <= idxQR - 2; ia++) {
            b_ix = (idxQR - ia) - 2;
            workspace_data[b_ix] -= workspace_data[idxQR - 1] *
              qrmanager->QR.data[jjA - ia];
          }
        }
      }

      idxQR = (qrmanager->ncols / 2) << 1;
      ia = idxQR - 2;
      for (b_idx = 0; b_idx <= ia; b_idx += 2) {
        __m128d tmp;
        tmp = _mm_loadu_pd(&workspace_data[b_idx]);
        _mm_storeu_pd(&solution->lambda.data[b_idx], _mm_mul_pd(tmp, _mm_set1_pd
          (-1.0)));
      }

      for (b_idx = idxQR; b_idx < nActiveConstr; b_idx++) {
        solution->lambda.data[b_idx] = -workspace_data[b_idx];
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_checkStoppingAndUpdateFval(int32_T *activeSetChangeID, const
  real_T f_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *solution,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, const
  s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, int32_T
  runTimeOptions_MaxIterations, const boolean_T *updateFval)
{
  real_T tempMaxConstr;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    tempMaxConstr = root_maxConstraintViolation_i0u(workingset,
      solution->xstar.data);
    solution->maxConstr = tempMaxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      if (objective->nvar - 1 >= 0) {
        std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                    static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
      }

      nonDegenerateWset = root_mo_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = root_maxConstraintViolation_i0u(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        if (objective->nvar - 1 >= 0) {
          std::memcpy(&solution->xstar.data[0], &solution->searchDir.data[0],
                      static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    solution->fstar = root_model_computeFval_ReuseHx(objective,
      memspace->workspace_float.data, f_data, solution->xstar.data);
    if ((solution->fstar < 1.0E-6) && ((solution->state != 0) ||
         (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_iterate(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  root_model_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = root_model_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  solution->state = -5;
  idxMinLambda = workingset->mConstrMax;
  if (workingset->mConstrMax - 1 >= 0) {
    std::memset(&solution->lambda.data[0], 0, static_cast<uint32_T>
                (workingset->mConstrMax) * sizeof(real_T));
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          root_model_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          root_model_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          root_model_factorQR(qrmanager, workingset->ATwset.data, nVar,
                              workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            idxMinLambda = qrmanager->mrows - k;
            if (idxMinLambda - 2 >= 0) {
              std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0
                          + 1], static_cast<uint32_T>(idxMinLambda - 1) * sizeof
                          (real_T));
            }
          }

          root_model_xorgqr(qrmanager->mrows, qrmanager->mrows,
                            qrmanager->minRowCol, qrmanager->Q.data,
                            qrmanager->Q.size, qrmanager->ldq,
                            qrmanager->tau.data);
          break;
        }

        root_model_compute_deltax(H, solution, memspace, qrmanager, cholmanager,
          objective, root_model_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = root_model_xnrm2_ob(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        if (nVar - 1 >= 0) {
          std::memset(&solution->searchDir.data[0], 0, static_cast<uint32_T>
                      (nVar) * sizeof(real_T));
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.4901161193847657E-10) ||
            (workingset->nActiveConstr >= nVar)) {
          root_model_compute_lambda(memspace->workspace_float.data, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              root_model_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            root_model_removeConstr(workingset, workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          root_model_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr, true,
            &normDelta, &updateFval, &k, &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              root_model_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              root_addBoundToActiveSetMatrix_(workingset, 4, idxMinLambda);
              break;

             default:
              root_addBoundToActiveSetMatrix_(workingset, 5, idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            root_m_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            idxMinLambda = (nVar / 2) << 1;
            g = idxMinLambda - 2;
            for (k = 0; k <= g; k += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[k]);
              tmp_0 = _mm_loadu_pd(&solution->xstar.data[k]);
              _mm_storeu_pd(&solution->xstar.data[k], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (k = idxMinLambda; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          root_model_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        root_checkStoppingAndUpdateFval(&activeSetChangeID, f_data, solution,
          memspace, objective, workingset, qrmanager,
          runTimeOptions_MaxIterations, &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = root_model_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_phaseone(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], const somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions)
{
  int32_T PROBTYPE_ORIG;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T mConstr;
  int32_T nVar;
  int32_T nVarP1;
  boolean_T exitg1;
  PROBTYPE_ORIG = workingset->probType;
  nVar = workingset->nVar;
  nVarP1 = workingset->nVar;
  solution->xstar.data[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }

  root_model_setProblemType(workingset, mConstr);
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (mConstr = idxStartIneq; mConstr <= idxEndIneq; mConstr++) {
    workingset->isActiveConstr.data[(workingset->isActiveIdx
      [workingset->Wid.data[mConstr - 1] - 1] + workingset->
      Wlocalidx.data[mConstr - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVarP1 + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  solution->fstar = solution->xstar.data[nVarP1];
  solution->state = 5;
  root_model_iterate(H, f_data, solution, memspace, workingset, qrmanager,
                     cholmanager, objective, options_SolverName,
                     runTimeOptions->MaxIterations);
  if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] +
       workingset->sizes[3]) - 2]) {
    mConstr = workingset->sizes[0] + 171;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      if ((workingset->Wid.data[mConstr - 1] == 4) &&
          (workingset->Wlocalidx.data[mConstr - 1] == workingset->sizes[3])) {
        root_model_removeConstr(workingset, mConstr);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }

  mConstr = workingset->nActiveConstr;
  while ((mConstr > workingset->sizes[0] + 170) && (mConstr > nVar)) {
    root_model_removeConstr(workingset, mConstr);
    mConstr--;
  }

  solution->maxConstr = solution->xstar.data[nVarP1];
  root_model_setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void ro_checkStoppingAndUpdateFval_j(int32_T *activeSetChangeID,
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, const s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective,
  s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset, s_Pct27jAhkqchx6iRs2zKpD_root_T
  *qrmanager, int32_T runTimeOptions_MaxIterations, boolean_T *updateFval)
{
  real_T tempMaxConstr;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    tempMaxConstr = root_maxConstraintViolation_i0u(workingset,
      solution->xstar.data);
    solution->maxConstr = tempMaxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      if (objective->nvar - 1 >= 0) {
        std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                    static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
      }

      nonDegenerateWset = root_mo_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = root_maxConstraintViolation_i0u(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        if (objective->nvar - 1 >= 0) {
          std::memcpy(&solution->xstar.data[0], &solution->searchDir.data[0],
                      static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    *updateFval = false;
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_iterate_j(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  root_model_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = root_model_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  idxMinLambda = workingset->mConstrMax;
  if (workingset->mConstrMax - 1 >= 0) {
    std::memset(&solution->lambda.data[0], 0, static_cast<uint32_T>
                (workingset->mConstrMax) * sizeof(real_T));
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          root_model_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          root_model_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          root_model_factorQR(qrmanager, workingset->ATwset.data, nVar,
                              workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            idxMinLambda = qrmanager->mrows - k;
            if (idxMinLambda - 2 >= 0) {
              std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0
                          + 1], static_cast<uint32_T>(idxMinLambda - 1) * sizeof
                          (real_T));
            }
          }

          root_model_xorgqr(qrmanager->mrows, qrmanager->mrows,
                            qrmanager->minRowCol, qrmanager->Q.data,
                            qrmanager->Q.size, qrmanager->ldq,
                            qrmanager->tau.data);
          break;
        }

        root_model_compute_deltax(H, solution, memspace, qrmanager, cholmanager,
          objective, root_model_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = root_model_xnrm2_ob(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        if (nVar - 1 >= 0) {
          std::memset(&solution->searchDir.data[0], 0, static_cast<uint32_T>
                      (nVar) * sizeof(real_T));
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.0E-6) ||
            (workingset->nActiveConstr >= nVar)) {
          root_model_compute_lambda(memspace->workspace_float.data, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              root_model_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            root_model_removeConstr(workingset, workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          root_model_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr,
            (objective->objtype == 5), &normDelta, &updateFval, &k,
            &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              root_model_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              root_addBoundToActiveSetMatrix_(workingset, 4, idxMinLambda);
              break;

             default:
              root_addBoundToActiveSetMatrix_(workingset, 5, idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            root_m_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            idxMinLambda = (nVar / 2) << 1;
            g = idxMinLambda - 2;
            for (k = 0; k <= g; k += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[k]);
              tmp_0 = _mm_loadu_pd(&solution->xstar.data[k]);
              _mm_storeu_pd(&solution->xstar.data[k], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (k = idxMinLambda; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          root_model_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        ro_checkStoppingAndUpdateFval_j(&activeSetChangeID, solution, memspace,
          objective, workingset, qrmanager, runTimeOptions_MaxIterations,
          &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = root_model_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
  real_T workspace_data[], const real_T H[31329], const real_T f_data[], const
  real_T x_data[])
{
  int32_T beta1;
  beta1 = 0;
  if (obj_hasLinear) {
    if (obj_nvar - 1 >= 0) {
      std::memcpy(&workspace_data[0], &f_data[0], static_cast<uint32_T>(obj_nvar)
                  * sizeof(real_T));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if ((beta1 != 1) && (obj_nvar - 1 >= 0)) {
      std::memset(&workspace_data[0], 0, static_cast<uint32_T>(obj_nvar) *
                  sizeof(real_T));
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar + 1;
    for (beta1 = 1; obj_nvar < 0 ? beta1 >= d : beta1 <= d; beta1 += obj_nvar) {
      real_T c;
      int32_T e;
      c = 0.5 * x_data[ix];
      e = beta1 + obj_nvar;
      for (int32_T ia{beta1}; ia < e; ia++) {
        int32_T tmp;
        tmp = ia - beta1;
        workspace_data[tmp] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_driver_i(const real_T H[31329], const real_T f_data[],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *solution, s_3DDm1jFKx2gyv5YUdHGqJC_root_T
  *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T *workingset,
  s_Pct27jAhkqchx6iRs2zKpD_root_T *qrmanager, s_mdyENYu4oaiCttMAPwg9iE_root_T
  *cholmanager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *objective, const
  somzaGboVhDG7PNQS6E98jD_root__T *options, const
  somzaGboVhDG7PNQS6E98jD_root__T *runTimeOptions)
{
  __m128d tmp;
  real_T maxConstr_new;
  int32_T idx;
  int32_T ixlast;
  int32_T nVar;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T guard1;
  solution->iterations = 0;
  nVar = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    ixlast = static_cast<uint16_T>(workingset->sizes[0]);
    for (idx = 0; idx < ixlast; idx++) {
      solution->xstar.data[workingset->indexFixed.data[idx] - 1] =
        workingset->ub.data[workingset->indexFixed.data[idx] - 1];
    }

    ixlast = static_cast<uint16_T>(workingset->sizes[3]);
    for (idx = 0; idx < ixlast; idx++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] + idx) - 1])
      {
        solution->xstar.data[workingset->indexLB.data[idx] - 1] =
          -workingset->lb.data[workingset->indexLB.data[idx] - 1];
      }
    }

    ixlast = static_cast<uint16_T>(workingset->sizes[4]);
    for (idx = 0; idx < ixlast; idx++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[4] + idx) - 1])
      {
        solution->xstar.data[workingset->indexUB.data[idx] - 1] =
          workingset->ub.data[workingset->indexUB.data[idx] - 1];
      }
    }

    root_model_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state < 0) {
    } else {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = root_maxConstraintViolation_i0u(workingset,
      solution->xstar.data);
    if (solution->maxConstr > 1.0E-6) {
      root_model_phaseone(H, f_data, solution, memspace, workingset, qrmanager,
                          cholmanager, objective, options->SolverName,
                          runTimeOptions);
      if (solution->state == 0) {
      } else {
        solution->maxConstr = root_maxConstraintViolation_i0u(workingset,
          solution->xstar.data);
        if (solution->maxConstr > 1.0E-6) {
          if (workingset->mConstrMax - 1 >= 0) {
            std::memset(&solution->lambda.data[0], 0, static_cast<uint32_T>
                        (workingset->mConstrMax) * sizeof(real_T));
          }

          maxConstr_new = 0.0;
          switch (objective->objtype) {
           case 5:
            maxConstr_new = solution->xstar.data[objective->nvar - 1] *
              objective->gammaScalar;
            break;

           case 3:
            root_model_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            if (objective->nvar >= 1) {
              ixlast = objective->nvar;
              for (nVar = 0; nVar < ixlast; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;

           case 4:
            root_model_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            idx = objective->nvar + 1;
            ixlast = objective->maxVar;
            scalarLB = (((((objective->maxVar - objective->nvar) - 1) / 2) << 1)
                        + objective->nvar) + 1;
            vectorUB = scalarLB - 2;
            for (nVar = idx; nVar <= vectorUB; nVar += 2) {
              tmp = _mm_loadu_pd(&solution->xstar.data[nVar - 1]);
              _mm_storeu_pd(&memspace->workspace_float.data[nVar - 1],
                            _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5 *
                objective->beta), tmp), _mm_set1_pd(objective->rho)));
            }

            for (nVar = scalarLB; nVar < ixlast; nVar++) {
              memspace->workspace_float.data[nVar - 1] = 0.5 * objective->beta *
                solution->xstar.data[nVar - 1] + objective->rho;
            }

            if (objective->maxVar - 1 >= 1) {
              ixlast = objective->maxVar;
              for (nVar = 0; nVar <= ixlast - 2; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;
          }

          solution->fstar = maxConstr_new;
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            if (nVar - 1 >= 0) {
              std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                          static_cast<uint32_T>(nVar) * sizeof(real_T));
            }

            root_model_PresolveWorkingSet(solution, memspace, workingset,
              qrmanager);
            maxConstr_new = root_maxConstraintViolation_i0u(workingset,
              solution->xstar.data);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (nVar - 1 >= 0) {
                std::memcpy(&solution->xstar.data[0], &solution->searchDir.data
                            [0], static_cast<uint32_T>(nVar) * sizeof(real_T));
              }
            }
          }

          root_model_iterate_j(H, f_data, solution, memspace, workingset,
                               qrmanager, cholmanager, objective,
                               options->SolverName,
                               runTimeOptions->MaxIterations);
        }
      }
    } else {
      root_model_iterate_j(H, f_data, solution, memspace, workingset, qrmanager,
                           cholmanager, objective, options->SolverName,
                           runTimeOptions->MaxIterations);
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_addAeqConstr(s_Jrk1gF3jitJmkPBgphEknE_root_T *obj,
  int32_T idx_local)
{
  int32_T totalEq;
  totalEq = obj->nWConstr[0] + obj->nWConstr[1];
  if ((obj->nActiveConstr == totalEq) && (idx_local > obj->nWConstr[1])) {
    int32_T b_idx;
    int32_T iAeq0;
    int32_T iAw0;
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = 2;
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
    b_idx = static_cast<uint16_T>(obj->nVar);
    for (totalEq = 0; totalEq < b_idx; totalEq++) {
      obj->ATwset.data[iAw0 + totalEq] = obj->Aeq.data[iAeq0 + totalEq];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->beq[idx_local - 1];
  } else {
    int32_T c;
    int32_T iAeq0;
    int32_T iAw0;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = obj->Wid.data[totalEq];
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = obj->Wlocalidx.data[totalEq];
    iAw0 = static_cast<uint16_T>(obj->nVar);
    for (iAeq0 = 0; iAeq0 < iAw0; iAeq0++) {
      obj->ATwset.data[iAeq0 + obj->ldA * (obj->nActiveConstr - 1)] =
        obj->ATwset.data[obj->ldA * totalEq + iAeq0];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->bwset.data[totalEq];
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->Wid.data[totalEq] = 2;
    obj->Wlocalidx.data[totalEq] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = obj->ldA * totalEq;
    c = static_cast<uint16_T>(obj->nVar);
    for (int32_T b_idx{0}; b_idx < c; b_idx++) {
      obj->ATwset.data[iAw0 + b_idx] = obj->Aeq.data[iAeq0 + b_idx];
    }

    obj->bwset.data[totalEq] = obj->beq[idx_local - 1];
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static boolean_T root_model_soc(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_root__T *qpoptions)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T b_c;
  int32_T idxIneqOffset;
  int32_T idxStartIneq;
  int32_T idx_Aineq;
  int32_T idx_lower;
  int32_T ix;
  int32_T iy;
  int32_T l;
  int32_T nVar;
  int32_T nWIneq_old;
  int32_T nWLower_old;
  int32_T nWUpper_old;
  boolean_T success;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  std::memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
              static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  std::memcpy(&TrialState->socDirection.data[0], &TrialState->xstar.data[0],
              static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  if (WorkingSet->mConstrMax - 1 >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->lambda.data[0],
                static_cast<uint32_T>(WorkingSet->mConstrMax) * sizeof(real_T));
  }

  idxIneqOffset = WorkingSet->isActiveIdx[2];
  for (idxStartIneq = 0; idxStartIneq <= 168; idxStartIneq += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->cEq[idxStartIneq]);
    _mm_storeu_pd(&WorkingSet->beq[idxStartIneq], _mm_mul_pd(tmp_0, _mm_set1_pd(
      -1.0)));
  }

  idx_lower = WorkingSet->ldA;
  iy = 0;
  l = WorkingSet->ldA * 169 + 1;
  for (idxStartIneq = 1; idx_lower < 0 ? idxStartIneq >= l : idxStartIneq <= l;
       idxStartIneq += idx_lower) {
    b_c = 0.0;
    ix = idxStartIneq + WorkingSet->nVar;
    for (idx_Aineq = idxStartIneq; idx_Aineq < ix; idx_Aineq++) {
      b_c += WorkingSet->Aeq.data[idx_Aineq - 1] * TrialState->
        searchDir.data[idx_Aineq - idxStartIneq];
    }

    WorkingSet->beq[iy] += b_c;
    iy++;
  }

  for (idxStartIneq = 0; idxStartIneq < 170; idxStartIneq++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + idxStartIneq] =
      WorkingSet->beq[idxStartIneq];
  }

  if (WorkingSet->sizes[2] > 0) {
    idx_Aineq = static_cast<uint16_T>(WorkingSet->sizes[2]);
    idx_lower = (static_cast<uint16_T>(WorkingSet->sizes[2]) / 2) << 1;
    iy = idx_lower - 2;
    for (idxStartIneq = 0; idxStartIneq <= iy; idxStartIneq += 2) {
      tmp_0 = _mm_loadu_pd(&TrialState->cIneq.data[idxStartIneq]);
      _mm_storeu_pd(&WorkingSet->bineq.data[idxStartIneq], _mm_mul_pd(tmp_0,
        _mm_set1_pd(-1.0)));
    }

    for (idxStartIneq = idx_lower; idxStartIneq < idx_Aineq; idxStartIneq++) {
      WorkingSet->bineq.data[idxStartIneq] = -TrialState->
        cIneq.data[idxStartIneq];
    }

    idx_lower = WorkingSet->ldA;
    iy = 0;
    l = (WorkingSet->sizes[2] - 1) * WorkingSet->ldA + 1;
    for (idxStartIneq = 1; idx_lower < 0 ? idxStartIneq >= l : idxStartIneq <= l;
         idxStartIneq += idx_lower) {
      b_c = 0.0;
      ix = idxStartIneq + WorkingSet->nVar;
      for (idx_Aineq = idxStartIneq; idx_Aineq < ix; idx_Aineq++) {
        b_c += WorkingSet->Aineq.data[idx_Aineq - 1] *
          TrialState->searchDir.data[idx_Aineq - idxStartIneq];
      }

      WorkingSet->bineq.data[iy] += b_c;
      iy++;
    }

    idx_Aineq = 1;
    idx_lower = WorkingSet->sizes[2] + 1;
    iy = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
    l = WorkingSet->nActiveConstr;
    for (idxStartIneq = idxIneqOffset; idxStartIneq <= l; idxStartIneq++) {
      switch (WorkingSet->Wid.data[idxStartIneq - 1]) {
       case 3:
        ix = idx_Aineq;
        idx_Aineq++;
        WorkingSet->bwset.data[idxStartIneq - 1] = WorkingSet->
          bineq.data[WorkingSet->Wlocalidx.data[idxStartIneq - 1] - 1];
        break;

       case 4:
        ix = idx_lower;
        idx_lower++;
        break;

       default:
        ix = iy;
        iy++;
        break;
      }

      TrialState->workingset_old.data[ix - 1] = WorkingSet->
        Wlocalidx.data[idxStartIneq - 1];
    }
  }

  std::memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0], static_cast<
              uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  root_model_driver_i(Hessian, grad_data, TrialState, memspace, WorkingSet,
                      QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    root_model_addAeqConstr(WorkingSet, WorkingSet->indexEqRemoved
      [WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  idxStartIneq = static_cast<uint16_T>(nVar);
  idx_lower = (static_cast<uint16_T>(nVar) / 2) << 1;
  iy = idx_lower - 2;
  for (idxIneqOffset = 0; idxIneqOffset <= iy; idxIneqOffset += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[idxIneqOffset]);
    tmp = _mm_loadu_pd(&TrialState->xstar.data[idxIneqOffset]);
    _mm_storeu_pd(&TrialState->socDirection.data[idxIneqOffset], _mm_sub_pd(tmp,
      tmp_0));
    _mm_storeu_pd(&TrialState->xstar.data[idxIneqOffset], tmp_0);
  }

  for (idxIneqOffset = idx_lower; idxIneqOffset < idxStartIneq; idxIneqOffset++)
  {
    b_c = TrialState->socDirection.data[idxIneqOffset];
    TrialState->socDirection.data[idxIneqOffset] = TrialState->
      xstar.data[idxIneqOffset] - b_c;
    TrialState->xstar.data[idxIneqOffset] = b_c;
  }

  success = (root_model_xnrm2_ob(nVar, TrialState->socDirection.data) <= 2.0 *
             root_model_xnrm2_ob(nVar, TrialState->xstar.data));
  idxIneqOffset = WorkingSet->sizes[2];
  for (nVar = 0; nVar <= 168; nVar += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->cEq[nVar]);
    _mm_storeu_pd(&WorkingSet->beq[nVar], _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
  }

  for (nVar = 0; nVar < 170; nVar++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + nVar] = WorkingSet->beq[nVar];
  }

  if (WorkingSet->sizes[2] > 0) {
    idxStartIneq = static_cast<uint16_T>(WorkingSet->sizes[2]);
    idx_lower = (static_cast<uint16_T>(WorkingSet->sizes[2]) / 2) << 1;
    iy = idx_lower - 2;
    for (nVar = 0; nVar <= iy; nVar += 2) {
      tmp_0 = _mm_loadu_pd(&TrialState->cIneq.data[nVar]);
      _mm_storeu_pd(&WorkingSet->bineq.data[nVar], _mm_mul_pd(tmp_0, _mm_set1_pd
        (-1.0)));
    }

    for (nVar = idx_lower; nVar < idxStartIneq; nVar++) {
      WorkingSet->bineq.data[nVar] = -TrialState->cIneq.data[nVar];
    }

    if (!success) {
      idxStartIneq = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
      idx_Aineq = WorkingSet->nActiveConstr;
      for (nVar = idxStartIneq; nVar <= idx_Aineq; nVar++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[nVar - 1] - 1] + WorkingSet->Wlocalidx.data[nVar
          - 1]) - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] + WorkingSet->
        nWConstr[1];
      for (nVar = 0; nVar < nWIneq_old; nVar++) {
        root_model_addAineqConstr(WorkingSet, TrialState->
          workingset_old.data[nVar]);
      }

      for (nWIneq_old = 0; nWIneq_old < nWLower_old; nWIneq_old++) {
        root_addBoundToActiveSetMatrix_(WorkingSet, 4,
          TrialState->workingset_old.data[nWIneq_old + idxIneqOffset]);
      }

      for (nWLower_old = 0; nWLower_old < nWUpper_old; nWLower_old++) {
        root_addBoundToActiveSetMatrix_(WorkingSet, 5,
          TrialState->workingset_old.data[(nWLower_old + idxIneqOffset) +
          WorkingSet->sizes[3]]);
      }
    }
  }

  if (!success) {
    if (WorkingSet->mConstrMax - 1 >= 0) {
      std::memcpy(&TrialState->lambda.data[0], &TrialState->lambdaStopTest.data
                  [0], static_cast<uint32_T>(WorkingSet->mConstrMax) * sizeof
                  (real_T));
    }
  } else {
    root_model_sortLambdaQP(TrialState->lambda.data, WorkingSet->nActiveConstr,
      WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid.data,
      WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
  }

  return success;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_mod_maxConstraintViolation(const
  s_Jrk1gF3jitJmkPBgphEknE_root_T *obj, const real_T x_data[])
{
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    root_model_B.b_obj = *obj;
    v = maxConstraintViolation_AMats_re(&root_model_B.b_obj, x_data);
  } else {
    root_model_B.b_obj = *obj;
    v = maxConstraintViolation_AMats_no(&root_model_B.b_obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = static_cast<uint16_T>(obj->sizes[3]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, -x_data[root_model_B.b_obj.indexLB.data[idx] - 1] -
                    root_model_B.b_obj.lb.data[root_model_B.b_obj.indexLB.data[idx]
                    - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    b = static_cast<uint16_T>(obj->sizes[4]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, x_data[root_model_B.b_obj.indexUB.data[idx] - 1] -
                    root_model_B.b_obj.ub.data[root_model_B.b_obj.indexUB.data[idx]
                    - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    b = static_cast<uint16_T>(obj->sizes[0]);
    for (idx = 0; idx < b; idx++) {
      v = std::fmax(v, std::abs(x_data[root_model_B.b_obj.indexFixed.data[idx] -
        1] - root_model_B.b_obj.ub.data[root_model_B.b_obj.indexFixed.data[idx]
        - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_normal(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_root__T *qpoptions,
  s7RdrPWkr8UPAUyTdDJkLaG_root__T *stepFlags)
{
  real_T TrialState_cIneq;
  real_T constrViolationEq;
  real_T constrViolationIneq;
  real_T penaltyParamTrial;
  int32_T b;
  int32_T k;
  boolean_T isEqAndIneqFeasible;
  root_model_driver_i(Hessian, grad_data, TrialState, memspace, WorkingSet,
                      QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  isEqAndIneqFeasible = (root_mod_maxConstraintViolation(WorkingSet,
    TrialState->xstar.data) <= 1.0E-6);
  if ((TrialState->state > 0) || ((TrialState->state == 0) &&
       isEqAndIneqFeasible)) {
    penaltyParamTrial = MeritFunction->penaltyParam;
    constrViolationEq = 0.0;
    for (k = 0; k < 170; k++) {
      constrViolationEq += std::abs(TrialState->cEq[k]);
    }

    constrViolationIneq = 0.0;
    b = static_cast<uint16_T>(WorkingSet->sizes[2]);
    for (k = 0; k < b; k++) {
      TrialState_cIneq = TrialState->cIneq.data[k];
      if (TrialState_cIneq > 0.0) {
        constrViolationIneq += TrialState_cIneq;
      }
    }

    constrViolationEq += constrViolationIneq;
    constrViolationIneq = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    constrViolationIneq += constrViolationEq;
    if ((constrViolationIneq > 2.2204460492503131E-16) && (TrialState->fstar >
         0.0)) {
      if (TrialState->sqpFval == 0.0) {
        penaltyParamTrial = 1.0;
      } else {
        penaltyParamTrial = 1.5;
      }

      penaltyParamTrial = penaltyParamTrial * TrialState->fstar /
        constrViolationIneq;
    }

    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi = penaltyParamTrial * constrViolationEq +
        TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * penaltyParamTrial +
           MeritFunction->initFval) - MeritFunction->phi > static_cast<real_T>
          (MeritFunction->nPenaltyDecreases) * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        MeritFunction->penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
          TrialState->sqpFval;
      }
    } else {
      MeritFunction->penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
        TrialState->sqpFval;
    }

    MeritFunction->phiPrimePlus = std::fmin(TrialState->fstar -
      MeritFunction->penaltyParam * constrViolationEq, 0.0);
  } else if (TrialState->state != -6) {
    stepFlags->stepType = 2;
  }

  root_model_sortLambdaQP(TrialState->lambda.data, WorkingSet->nActiveConstr,
    WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid.data,
    WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
  isEqAndIneqFeasible = (WorkingSet->mEqRemoved > 0);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    root_model_addAeqConstr(WorkingSet, WorkingSet->indexEqRemoved
      [WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  if (isEqAndIneqFeasible) {
    for (k = 0; k < 170; k++) {
      WorkingSet->Wlocalidx.data[WorkingSet->sizes[0] + k] = k + 1;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_relaxed(const real_T Hessian[31329], const real_T
  grad_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_root__T *qpoptions)
{
  real_T TrialState_cIneq;
  real_T beta;
  real_T qpfvalQuadExcess;
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T ix;
  int32_T mFiniteLBOrig;
  int32_T mLBOrig;
  int32_T nVarOrig;
  int32_T tmp;
  boolean_T b_tf;
  boolean_T tf;
  nVarOrig = WorkingSet->nVar - 1;
  beta = 0.0;
  idx_max = static_cast<uint16_T>(WorkingSet->nVar);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
    beta += Hessian[177 * mFiniteLBOrig + mFiniteLBOrig];
  }

  beta /= static_cast<real_T>(WorkingSet->nVar);
  if (TrialState->sqpIterations <= 1) {
    mLBOrig = QPObjective->nvar;
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective->nvar > 1) {
        smax = std::abs(grad_data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
          s = std::abs(grad_data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    smax = std::fmax(1.0, std::abs(grad_data[idx_max - 1])) * 100.0;
  } else {
    mLBOrig = WorkingSet->mConstr;
    if (WorkingSet->mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet->mConstr > 1) {
        smax = std::abs(TrialState->lambdasqp.data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
          s = std::abs(TrialState->lambdasqp.data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    smax = std::abs(TrialState->lambdasqp.data[idx_max - 1]);
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = smax;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  root_model_B.b_WorkingSet = *WorkingSet;
  root_model_setProblemType(&root_model_B.b_WorkingSet, 2);
  idx_max = root_model_B.b_WorkingSet.sizes[2] + 1;
  mLBOrig = (root_model_B.b_WorkingSet.sizes[3] -
             root_model_B.b_WorkingSet.sizes[2]) - 340;
  ix = static_cast<uint16_T>(root_model_B.b_WorkingSet.sizes[2]);
  if (ix - 1 >= 0) {
    std::memcpy(&memspace->workspace_float.data[0],
                &root_model_B.b_WorkingSet.bineq.data[0], static_cast<uint32_T>
                (ix) * sizeof(real_T));
  }

  root_model_xgemv_it3axg(WorkingSet->nVar, root_model_B.b_WorkingSet.sizes[2],
    root_model_B.b_WorkingSet.Aineq.data, root_model_B.b_WorkingSet.ldA,
    TrialState->xstar.data, memspace->workspace_float.data);
  ix = static_cast<uint16_T>(root_model_B.b_WorkingSet.sizes[2]);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < ix; mFiniteLBOrig++) {
    TrialState->xstar.data[(nVarOrig + mFiniteLBOrig) + 1] = static_cast<real_T>
      (memspace->workspace_float.data[mFiniteLBOrig] > 0.0) *
      memspace->workspace_float.data[mFiniteLBOrig];
  }

  std::memcpy(&memspace->workspace_float.data[0],
              &root_model_B.b_WorkingSet.beq[0], 170U * sizeof(real_T));
  root_model_xgemv_it3axg(WorkingSet->nVar, 170,
    root_model_B.b_WorkingSet.Aeq.data, root_model_B.b_WorkingSet.ldA,
    TrialState->xstar.data, memspace->workspace_float.data);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 170; mFiniteLBOrig++) {
    ix = idx_max + mFiniteLBOrig;
    if (memspace->workspace_float.data[mFiniteLBOrig] <= 0.0) {
      tmp = nVarOrig + ix;
      TrialState->xstar.data[tmp] = 0.0;
      TrialState->xstar.data[tmp + 170] = -memspace->
        workspace_float.data[mFiniteLBOrig];
      tmp = mLBOrig + ix;
      root_addBoundToActiveSetMatrix_(&root_model_B.b_WorkingSet, 4, tmp);
      if (memspace->workspace_float.data[mFiniteLBOrig] >= -1.0E-6) {
        root_addBoundToActiveSetMatrix_(&root_model_B.b_WorkingSet, 4, tmp + 170);
      }
    } else {
      tmp = nVarOrig + ix;
      TrialState->xstar.data[tmp] = memspace->workspace_float.data[mFiniteLBOrig];
      TrialState->xstar.data[tmp + 170] = 0.0;
      tmp = mLBOrig + ix;
      root_addBoundToActiveSetMatrix_(&root_model_B.b_WorkingSet, 4, tmp + 170);
      if (memspace->workspace_float.data[mFiniteLBOrig] <= 1.0E-6) {
        root_addBoundToActiveSetMatrix_(&root_model_B.b_WorkingSet, 4, tmp);
      }
    }
  }

  nVarOrig = qpoptions->MaxIterations;
  qpoptions->MaxIterations = (qpoptions->MaxIterations +
    root_model_B.b_WorkingSet.nVar) - WorkingSet->nVar;
  root_model_driver_i(Hessian, grad_data, TrialState, memspace,
                      &root_model_B.b_WorkingSet, QRManager, CholManager,
                      QPObjective, qpoptions, qpoptions);
  qpoptions->MaxIterations = nVarOrig;
  idx_max = root_model_B.b_WorkingSet.sizes[3] - 341;
  nVarOrig = 0;
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 170; mFiniteLBOrig++) {
    mLBOrig = (root_model_B.b_WorkingSet.isActiveIdx[3] + idx_max) +
      mFiniteLBOrig;
    tf = root_model_B.b_WorkingSet.isActiveConstr.data[mLBOrig];
    b_tf = root_model_B.b_WorkingSet.isActiveConstr.data[mLBOrig + 170];
    memspace->workspace_int.data[mFiniteLBOrig] = tf;
    memspace->workspace_int.data[mFiniteLBOrig + 170] = b_tf;
    nVarOrig = (nVarOrig + tf) + b_tf;
  }

  mLBOrig = static_cast<uint16_T>(root_model_B.b_WorkingSet.sizes[2]);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < mLBOrig; mFiniteLBOrig++) {
    tmp = root_model_B.b_WorkingSet.isActiveConstr.data
      [((root_model_B.b_WorkingSet.isActiveIdx[3] + idx_max) -
        root_model_B.b_WorkingSet.sizes[2]) + mFiniteLBOrig];
    memspace->workspace_int.data[mFiniteLBOrig + 340] = tmp;
    nVarOrig += tmp;
  }

  if (TrialState->state != -6) {
    idx_max = (root_model_B.b_WorkingSet.nVarMax - WorkingSet->nVar) - 1;
    mLBOrig = WorkingSet->nVar + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (idx_max >= 1) {
      ix = WorkingSet->nVar + idx_max;
      for (mFiniteLBOrig = mLBOrig; mFiniteLBOrig <= ix; mFiniteLBOrig++) {
        s += std::abs(TrialState->xstar.data[mFiniteLBOrig - 1]);
      }

      idx_max = static_cast<uint16_T>(idx_max);
      for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
        qpfvalQuadExcess += TrialState->xstar.data[WorkingSet->nVar +
          mFiniteLBOrig] * TrialState->xstar.data[WorkingSet->nVar +
          mFiniteLBOrig];
      }
    }

    beta = (TrialState->fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    idx_max = WorkingSet->nVar + 1;
    mLBOrig = (WorkingSet->nVarMax - WorkingSet->nVar) - 1;
    smax = MeritFunction->penaltyParam;
    s = 0.0;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 170; mFiniteLBOrig++) {
      s += std::abs(TrialState->cEq[mFiniteLBOrig]);
    }

    qpfvalQuadExcess = 0.0;
    ix = static_cast<uint16_T>(WorkingSet->sizes[2]);
    for (mFiniteLBOrig = 0; mFiniteLBOrig < ix; mFiniteLBOrig++) {
      TrialState_cIneq = TrialState->cIneq.data[mFiniteLBOrig];
      if (TrialState_cIneq > 0.0) {
        qpfvalQuadExcess += TrialState_cIneq;
      }
    }

    s += qpfvalQuadExcess;
    qpfvalQuadExcess = MeritFunction->linearizedConstrViol;
    TrialState_cIneq = 0.0;
    if (mLBOrig >= 1) {
      mLBOrig += WorkingSet->nVar;
      for (mFiniteLBOrig = idx_max; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
        TrialState_cIneq += std::abs(TrialState->xstar.data[mFiniteLBOrig - 1]);
      }
    }

    MeritFunction->linearizedConstrViol = TrialState_cIneq;
    qpfvalQuadExcess = (s + qpfvalQuadExcess) - TrialState_cIneq;
    if ((qpfvalQuadExcess > 2.2204460492503131E-16) && (beta > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        smax = 1.0;
      } else {
        smax = 1.5;
      }

      smax = smax * beta / qpfvalQuadExcess;
    }

    if (smax < MeritFunction->penaltyParam) {
      MeritFunction->phi = smax * s + TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * smax +
           MeritFunction->initFval) - MeritFunction->phi > static_cast<real_T>
          (MeritFunction->nPenaltyDecreases) * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        MeritFunction->penaltyParam = std::fmax(smax, 1.0E-10);
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * s +
          TrialState->sqpFval;
      }
    } else {
      MeritFunction->penaltyParam = std::fmax(smax, 1.0E-10);
      MeritFunction->phi = MeritFunction->penaltyParam * s + TrialState->sqpFval;
    }

    MeritFunction->phiPrimePlus = std::fmin(beta - MeritFunction->penaltyParam *
      s, 0.0);
    idx_max = root_model_B.b_WorkingSet.isActiveIdx[1] - 1;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 170; mFiniteLBOrig++) {
      if ((memspace->workspace_int.data[mFiniteLBOrig] != 0) &&
          (memspace->workspace_int.data[mFiniteLBOrig + 170] != 0)) {
        tf = true;
      } else {
        tf = false;
      }

      tmp = idx_max + mFiniteLBOrig;
      TrialState->lambda.data[tmp] *= static_cast<real_T>(tf);
    }

    idx_max = root_model_B.b_WorkingSet.isActiveIdx[2];
    mLBOrig = root_model_B.b_WorkingSet.nActiveConstr;
    for (mFiniteLBOrig = idx_max; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
      if (root_model_B.b_WorkingSet.Wid.data[mFiniteLBOrig - 1] == 3) {
        TrialState->lambda.data[mFiniteLBOrig - 1] *= static_cast<real_T>
          (memspace->
           workspace_int.data[root_model_B.b_WorkingSet.Wlocalidx.data[mFiniteLBOrig
           - 1] + 339]);
      }
    }
  }

  mFiniteLBOrig = (root_model_B.b_WorkingSet.sizes[3] -
                   root_model_B.b_WorkingSet.sizes[2]) - 340;
  idx_max = root_model_B.b_WorkingSet.nActiveConstr;
  while ((idx_max > root_model_B.b_WorkingSet.sizes[0] + 170) && (nVarOrig > 0))
  {
    if ((root_model_B.b_WorkingSet.Wid.data[idx_max - 1] == 4) &&
        (root_model_B.b_WorkingSet.Wlocalidx.data[idx_max - 1] > mFiniteLBOrig))
    {
      beta = TrialState->lambda.data[root_model_B.b_WorkingSet.nActiveConstr - 1];
      TrialState->lambda.data[root_model_B.b_WorkingSet.nActiveConstr - 1] = 0.0;
      TrialState->lambda.data[idx_max - 1] = beta;
      root_model_removeConstr(&root_model_B.b_WorkingSet, idx_max);
      nVarOrig--;
    }

    idx_max--;
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  *WorkingSet = root_model_B.b_WorkingSet;
  root_model_setProblemType(WorkingSet, 3);
  root_model_sortLambdaQP(TrialState->lambda.data, WorkingSet->nActiveConstr,
    WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid.data,
    WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_step_k(s7RdrPWkr8UPAUyTdDJkLaG_root__T *stepFlags, real_T
  Hessian[31329], const real_T lb[177], s_7iu1rzNoAoHigERqeOFm2G_root_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T *MeritFunction,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_root__T *qpoptions)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T tmp_1[2];
  real_T nrmDirInf;
  real_T nrmGradInf;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T k;
  int32_T nVar;
  int32_T vectorUB;
  boolean_T checkBoundViolation;
  boolean_T guard1;
  stepFlags->stepAccepted = true;
  checkBoundViolation = true;
  nVar = WorkingSet->nVar - 1;
  if (stepFlags->stepType != 3) {
    std::memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0],
                static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  } else if (WorkingSet->nVar - 1 >= 0) {
    std::memcpy(&TrialState->searchDir.data[0], &TrialState->xstar.data[0],
                static_cast<uint32_T>((WorkingSet->nVar - 1) + 1) * sizeof
                (real_T));
  }

  do {
    exitg1 = 0;
    guard1 = false;
    switch (stepFlags->stepType) {
     case 1:
      root_model_normal(Hessian, TrialState->grad.data, TrialState,
                        MeritFunction, memspace, WorkingSet, QRManager,
                        CholManager, QPObjective, qpoptions, stepFlags);
      if (stepFlags->stepType == 2) {
      } else {
        if (nVar >= 0) {
          std::memcpy(&TrialState->delta_x.data[0], &TrialState->xstar.data[0],
                      static_cast<uint32_T>(nVar + 1) * sizeof(real_T));
        }

        guard1 = true;
      }
      break;

     case 2:
      idxStartIneq = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (k = idxStartIneq; k <= idxEndIneq; k++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[k - 1] - 1] + WorkingSet->Wlocalidx.data[k - 1])
          - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] + WorkingSet->
        nWConstr[1];
      vectorUB = TrialState->xstar.size[0];
      k = TrialState->xstar.size[0];
      if (k - 1 >= 0) {
        std::memcpy(&root_model_B.b_data[0], &TrialState->xstar.data[0],
                    static_cast<uint32_T>(k) * sizeof(real_T));
      }

      idxStartIneq = static_cast<uint16_T>(WorkingSet->sizes[3]);
      for (k = 0; k < idxStartIneq; k++) {
        if (-root_model_B.b_data[WorkingSet->indexLB.data[k] - 1] >
            WorkingSet->lb.data[WorkingSet->indexLB.data[k] - 1]) {
          root_model_B.b_data[WorkingSet->indexLB.data[k] - 1] =
            -WorkingSet->lb.data[WorkingSet->indexLB.data[k] - 1] + std::abs
            (WorkingSet->lb.data[WorkingSet->indexLB.data[k] - 1]);
        }
      }

      if (vectorUB - 1 >= 0) {
        std::memcpy(&TrialState->xstar.data[0], &root_model_B.b_data[0],
                    static_cast<uint32_T>(vectorUB) * sizeof(real_T));
      }

      root_model_relaxed(Hessian, TrialState->grad.data, TrialState,
                         MeritFunction, memspace, WorkingSet, QRManager,
                         CholManager, QPObjective, qpoptions);
      if (nVar >= 0) {
        std::memcpy(&TrialState->delta_x.data[0], &TrialState->xstar.data[0],
                    static_cast<uint32_T>(nVar + 1) * sizeof(real_T));
      }

      guard1 = true;
      break;

     default:
      checkBoundViolation = root_model_soc(Hessian, TrialState->grad.data,
        TrialState, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        qpoptions);
      stepFlags->stepAccepted = checkBoundViolation;
      if (stepFlags->stepAccepted && (TrialState->state != -6)) {
        idxStartIneq = static_cast<uint16_T>(nVar + 1);
        idxEndIneq = (static_cast<uint16_T>(nVar + 1) / 2) << 1;
        vectorUB = idxEndIneq - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          tmp = _mm_loadu_pd(&TrialState->xstar.data[k]);
          tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[k]);
          _mm_storeu_pd(&TrialState->delta_x.data[k], _mm_add_pd(tmp, tmp_0));
        }

        for (k = idxEndIneq; k < idxStartIneq; k++) {
          TrialState->delta_x.data[k] = TrialState->xstar.data[k] +
            TrialState->socDirection.data[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (k = 0; k < 177; k++) {
          nrmGradInf = std::fmax(nrmGradInf, std::abs(TrialState->grad.data[k]));
          nrmDirInf = std::fmax(nrmDirInf, std::abs(TrialState->xstar.data[k]));
        }

        nrmGradInf = std::fmax(2.2204460492503131E-16, nrmGradInf / nrmDirInf);
        for (k = 0; k < 177; k++) {
          idxEndIneq = 177 * k;
          for (idxStartIneq = 0; idxStartIneq < k; idxStartIneq++) {
            Hessian[idxEndIneq + idxStartIneq] = 0.0;
          }

          idxEndIneq = 177 * k + k;
          Hessian[idxEndIneq] = nrmGradInf;
          if (175 - k >= 0) {
            std::memset(&Hessian[idxEndIneq + 1], 0, static_cast<uint32_T>((175
              - k) + 1) * sizeof(real_T));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    vectorUB = TrialState->delta_x.size[0];
    k = TrialState->delta_x.size[0];
    if (k - 1 >= 0) {
      std::memcpy(&root_model_B.b_data[0], &TrialState->delta_x.data[0],
                  static_cast<uint32_T>(k) * sizeof(real_T));
    }

    k = static_cast<uint16_T>(WorkingSet->sizes[3]);
    for (nVar = 0; nVar < k; nVar++) {
      nrmGradInf = (TrialState->xstarsqp[WorkingSet->indexLB.data[nVar] - 1] +
                    root_model_B.b_data[WorkingSet->indexLB.data[nVar] - 1]) -
        lb[WorkingSet->indexLB.data[nVar] - 1];
      if (nrmGradInf < 0.0) {
        _mm_storeu_pd(&tmp_1[0], _mm_sub_pd(_mm_set_pd(TrialState->
          xstar.data[WorkingSet->indexLB.data[nVar] - 1],
          root_model_B.b_data[WorkingSet->indexLB.data[nVar] - 1]), _mm_set1_pd
          (nrmGradInf)));
        root_model_B.b_data[WorkingSet->indexLB.data[nVar] - 1] = tmp_1[0];
        TrialState->xstar.data[WorkingSet->indexLB.data[nVar] - 1] = tmp_1[1];
      }
    }

    if (vectorUB - 1 >= 0) {
      std::memcpy(&TrialState->delta_x.data[0], &root_model_B.b_data[0],
                  static_cast<uint32_T>(vectorUB) * sizeof(real_T));
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_stateEvolution_c(const real_T X[187], const real_T U[33],
  real_T parameters_f1, real_T parameters_f2, real_T parameters_f3, const real_T
  parameters_f4[9], real_T parameters_f5, real_T c[170])
{
  __m128d tmp_0;
  real_T b_X[187];
  real_T b_U[33];
  real_T fk[17];
  real_T fk1[17];
  real_T ic[17];
  real_T parameters_f4_0[9];
  real_T tmp[2];
  real_T ic_0;
  int32_T fk1_tmp;
  int32_T i;
  int32_T i_0;
  std::memset(&c[0], 0, 170U * sizeof(real_T));
  for (i_0 = 0; i_0 <= 14; i_0 += 2) {
    tmp[0] = i_0;
    tmp[1] = i_0 + 1;
    tmp_0 = _mm_loadu_pd(&tmp[0]);
    _mm_storeu_pd(&ic[i_0], _mm_add_pd(tmp_0, _mm_set1_pd(1.0)));
  }

  for (i_0 = 16; i_0 < 17; i_0++) {
    ic[i_0] = static_cast<real_T>(i_0) + 1.0;
  }

  for (i_0 = 0; i_0 < 11; i_0++) {
    b_U[3 * i_0] = U[i_0];
    b_U[3 * i_0 + 1] = U[i_0 + 11];
    b_U[3 * i_0 + 2] = U[i_0 + 22];
    for (i = 0; i < 17; i++) {
      b_X[i + 17 * i_0] = X[11 * i + i_0];
    }
  }

  for (i = 0; i < 10; i++) {
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(&b_X[17 * i], &b_U[3 * i], parameters_f1,
      parameters_f2, parameters_f3, parameters_f4_0, parameters_f5, fk);
    fk1_tmp = (i + 1) * 17;
    std::memcpy(&parameters_f4_0[0], &parameters_f4[0], 9U * sizeof(real_T));
    root_model_vehicle_state_func(&b_X[fk1_tmp], &b_U[3 * i], parameters_f1,
      parameters_f2, parameters_f3, parameters_f4_0, parameters_f5, fk1);
    for (i_0 = 0; i_0 < 17; i_0++) {
      ic_0 = ic[i_0];
      c[static_cast<int32_T>(ic_0) - 1] = (b_X[17 * i + i_0] + (fk[i_0] +
        fk1[i_0]) * 0.5) - b_X[fk1_tmp + i_0];
      ic[i_0] = ic_0 + 17.0;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_outputBounds_n(const real_T runtimedata_OutputMin[130],
  const real_T runtimedata_OutputMax[130], const real_T X[187], real_T e, real_T
  c_data[], int32_T c_size[2])
{
  __m128d tmp_1;
  real_T b_c[260];
  real_T ic[13];
  real_T tmp_0[2];
  real_T ic_0;
  real_T runtimedata_OutputMin_0;
  int32_T icf_tmp_1[26];
  int32_T icf_tmp[13];
  int32_T icf_tmp_0[13];
  int32_T i;
  int32_T icf_tmp_2;
  int32_T k;
  int16_T tmp_data[260];
  boolean_T icf[260];
  boolean_T tmp[130];
  boolean_T icf_0[26];
  boolean_T x[13];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T y;
  for (i = 0; i < 130; i++) {
    tmp[i] = std::isinf(runtimedata_OutputMin[i]);
  }

  all_Dt0A7QFv(tmp, x);
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 13)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  guard1 = false;
  if (y) {
    for (i = 0; i < 130; i++) {
      tmp[i] = std::isinf(runtimedata_OutputMax[i]);
    }

    all_Dt0A7QFv(tmp, x);
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 13)) {
      if (!x[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (y) {
      c_size[0] = 0;
      c_size[1] = 0;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    for (i = 0; i < 260; i++) {
      b_c[i] = 0.0;
      icf[i] = true;
    }

    for (i = 0; i <= 10; i += 2) {
      tmp_0[0] = i;
      tmp_0[1] = i + 1;
      tmp_1 = _mm_loadu_pd(&tmp_0[0]);
      _mm_storeu_pd(&ic[i], _mm_add_pd(tmp_1, _mm_set1_pd(1.0)));
    }

    for (i = 12; i < 13; i++) {
      ic[i] = static_cast<real_T>(i) + 1.0;
    }

    for (k = 0; k < 10; k++) {
      for (i = 0; i < 13; i++) {
        ic_0 = ic[i];
        icf_tmp[i] = static_cast<int32_T>(ic_0);
        runtimedata_OutputMin_0 = runtimedata_OutputMin[10 * i + k];
        icf[static_cast<int32_T>(ic_0) - 1] = ((!std::isinf
          (runtimedata_OutputMin_0)) && (!std::isnan(runtimedata_OutputMin_0)));
        icf_tmp_0[i] = static_cast<int32_T>(ic_0 + 13.0);
      }

      for (i = 0; i < 13; i++) {
        icf_tmp_2 = icf_tmp_0[i];
        ic_0 = runtimedata_OutputMax[10 * i + k];
        icf[icf_tmp_2 - 1] = ((!std::isinf(ic_0)) && (!std::isnan(ic_0)));
        icf_tmp_1[i] = icf_tmp[i] - 1;
        icf_tmp_1[i + 13] = icf_tmp_2 - 1;
      }

      for (i = 0; i < 26; i++) {
        icf_0[i] = icf[icf_tmp_1[i]];
      }

      if (root_model_any(icf_0)) {
        for (i = 0; i < 13; i++) {
          b_c[icf_tmp[i] - 1] = (runtimedata_OutputMin[10 * i + k] - e) - X[(11 *
            i + k) + 1];
        }

        for (i = 0; i < 13; i++) {
          b_c[static_cast<int32_T>(ic[i] + 13.0) - 1] = (X[(11 * i + k) + 1] -
            runtimedata_OutputMax[10 * i + k]) - e;
        }
      }

      for (i = 0; i <= 10; i += 2) {
        tmp_1 = _mm_loadu_pd(&ic[i]);
        _mm_storeu_pd(&ic[i], _mm_add_pd(tmp_1, _mm_set1_pd(26.0)));
      }

      for (i = 12; i < 13; i++) {
        ic[i] += 26.0;
      }
    }

    k = 0;
    for (i = 0; i < 260; i++) {
      if (icf[i]) {
        k++;
      }
    }

    icf_tmp_2 = k;
    k = 0;
    for (i = 0; i < 260; i++) {
      if (icf[i]) {
        tmp_data[k] = static_cast<int16_T>(i);
        k++;
      }
    }

    c_size[0] = icf_tmp_2;
    c_size[1] = 1;
    for (i = 0; i < icf_tmp_2; i++) {
      c_data[i] = b_c[tmp_data[i]];
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_c4_mpclib_anonFcn2_n(const real_T runtimedata_x[17],
  const real_T runtimedata_OutputMin[130], const real_T runtimedata_OutputMax
  [130], real_T runtimedata_Parameters_f1, real_T runtimedata_Parameters_f2,
  real_T runtimedata_Parameters_f3, const real_T runtimedata_Parameters_f4[9],
  real_T runtimedata_Parameters_f5, const real_T z[177], real_T
  varargout_1_data[], int32_T varargout_1_size[2], real_T varargout_2[170])
{
  real_T varargin_1_data[260];
  real_T X[187];
  real_T U[33];
  real_T e;
  int32_T varargin_1_size[2];
  int32_T i;
  int32_T varargout_1_size_tmp;
  int16_T input_sizes_idx_0;
  boolean_T sizes_idx_1_tmp;
  getXUe_FXX9Wcuz(z, runtimedata_x, X, U, &e);
  root_model_stateEvolution_c(X, U, runtimedata_Parameters_f1,
    runtimedata_Parameters_f2, runtimedata_Parameters_f3,
    runtimedata_Parameters_f4, runtimedata_Parameters_f5, varargout_2);
  root_model_outputBounds_n(runtimedata_OutputMin, runtimedata_OutputMax, X, e,
    varargin_1_data, varargin_1_size);
  sizes_idx_1_tmp = ((varargin_1_size[0] != 0) && (varargin_1_size[1] != 0));
  if ((!sizes_idx_1_tmp) || sizes_idx_1_tmp) {
    input_sizes_idx_0 = static_cast<int16_T>(varargin_1_size[0]);
  } else {
    input_sizes_idx_0 = 0;
  }

  varargout_1_size[0] = input_sizes_idx_0;
  varargout_1_size_tmp = sizes_idx_1_tmp;
  varargout_1_size[1] = sizes_idx_1_tmp;
  for (i = 0; i < varargout_1_size_tmp; i++) {
    if (input_sizes_idx_0 - 1 >= 0) {
      std::memcpy(&varargout_1_data[0], &varargin_1_data[0],
                  static_cast<uint32_T>(input_sizes_idx_0) * sizeof(real_T));
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_evalObjAndConstr(int32_T obj_next_next_next_next_next_b_,
  const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *obj_next_next_next_next_next_ne, const
  s_pVH6DktXFODPXN9VaZK2hE_root_T *obj_next_next_next_next_next__0, const real_T
  x[177], real_T Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[170],
  real_T *fval, int32_T *status)
{
  __m128d tmp_1;
  __m128i tmp_0;
  real_T b_data[260];
  real_T X[187];
  real_T b_X[187];
  real_T c[170];
  real_T U[33];
  real_T b_U[33];
  real_T tmp_2[2];
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T duk_idx_2;
  real_T e;
  real_T fs;
  real_T umvk;
  real_T umvk_idx_0;
  real_T umvk_idx_1;
  real_T umvk_idx_2;
  int32_T ineqRange_data[260];
  int32_T b_size[2];
  int32_T i;
  int32_T ineqRange_size_idx_1;
  int32_T n;
  int32_T vectorUB;
  int32_T yk;
  boolean_T tmp;
  getXUe_FXX9Wcuz(x, obj_next_next_next_next_next__0->runtimedata.x, X, U, &e);
  fs = 0.0;
  for (n = 0; n < 11; n++) {
    for (i = 0; i < 17; i++) {
      b_X[i + 17 * n] = X[11 * i + n];
    }

    b_U[3 * n] = U[n];
    b_U[3 * n + 1] = U[n + 11];
    b_U[3 * n + 2] = U[n + 22];
  }

  for (i = 0; i < 10; i++) {
    umvk_idx_1 = 0.0;
    for (n = 0; n < 13; n++) {
      yk = 10 * n + i;
      umvk_idx_2 = (b_X[(i + 1) * 17 + n] -
                    obj_next_next_next_next_next__0->runtimedata.ref[yk]) *
        obj_next_next_next_next_next__0->runtimedata.OutputWeights[yk];
      umvk_idx_1 += umvk_idx_2 * umvk_idx_2;
    }

    fs += umvk_idx_1;
    umvk_idx_0 = b_U[3 * i];
    umvk_idx_1 = b_U[3 * i + 1];
    umvk_idx_2 = b_U[3 * i + 2];
    if (i == 0) {
      _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(umvk_idx_1, umvk_idx_0),
        _mm_loadu_pd(&obj_next_next_next_next_next__0->runtimedata.lastMV[0])));
      duk_idx_0 = tmp_2[0];
      duk_idx_1 = tmp_2[1];
      duk_idx_2 = umvk_idx_2 -
        obj_next_next_next_next_next__0->runtimedata.lastMV[2];
    } else {
      n = (i - 1) * 3;
      tmp_1 = _mm_sub_pd(_mm_set_pd(umvk_idx_1, umvk_idx_0), _mm_loadu_pd(&b_U[n]));
      _mm_storeu_pd(&tmp_2[0], tmp_1);
      duk_idx_0 = tmp_2[0];
      duk_idx_1 = tmp_2[1];
      duk_idx_2 = umvk_idx_2 - b_U[n + 2];
    }

    umvk_idx_0 = (umvk_idx_0 -
                  obj_next_next_next_next_next__0->runtimedata.MVScaledTarget[i])
      * obj_next_next_next_next_next__0->runtimedata.MVWeights[i];
    duk_idx_0 *= obj_next_next_next_next_next__0->runtimedata.MVRateWeights[i];
    umvk = umvk_idx_0 * umvk_idx_0;
    duk = duk_idx_0 * duk_idx_0;
    umvk_idx_0 = (umvk_idx_1 -
                  obj_next_next_next_next_next__0->runtimedata.MVScaledTarget[i
                  + 10]) *
      obj_next_next_next_next_next__0->runtimedata.MVWeights[i + 10];
    duk_idx_0 = obj_next_next_next_next_next__0->runtimedata.MVRateWeights[i +
      10] * duk_idx_1;
    umvk += umvk_idx_0 * umvk_idx_0;
    duk += duk_idx_0 * duk_idx_0;
    umvk_idx_0 = (umvk_idx_2 -
                  obj_next_next_next_next_next__0->runtimedata.MVScaledTarget[i
                  + 20]) *
      obj_next_next_next_next_next__0->runtimedata.MVWeights[i + 20];
    duk_idx_0 = obj_next_next_next_next_next__0->runtimedata.MVRateWeights[i +
      20] * duk_idx_2;
    fs = ((umvk_idx_0 * umvk_idx_0 + umvk) + fs) + (duk_idx_0 * duk_idx_0 + duk);
  }

  *fval = 100000.0 * e * e + fs;
  tmp = std::isnan(*fval);
  if (std::isinf(*fval) || tmp) {
    if (tmp) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    if (obj_next_next_next_next_next_b_ - 1 < 0) {
      n = 0;
    } else {
      n = static_cast<uint16_T>(obj_next_next_next_next_next_b_ - 1) + 1;
    }

    ineqRange_size_idx_1 = n;
    if (n > 0) {
      ineqRange_data[0] = 0;
      yk = 0;
      for (i = 2; i <= n; i++) {
        yk++;
        ineqRange_data[i - 1] = yk;
      }
    }

    i = n - 1;
    yk = (n / 4) << 2;
    vectorUB = yk - 4;
    for (n = 0; n <= vectorUB; n += 4) {
      tmp_0 = _mm_loadu_si128((const __m128i *)&ineqRange_data[n]);
      _mm_storeu_si128((__m128i *)&ineqRange_data[n], _mm_add_epi32(tmp_0,
        _mm_set1_epi32(ineq0)));
    }

    for (n = yk; n <= i; n++) {
      ineqRange_data[n] += ineq0;
    }

    root_model_c4_mpclib_anonFcn2_n(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax,
      obj_next_next_next_next_next_ne->Parameters.f1,
      obj_next_next_next_next_next_ne->Parameters.f2,
      obj_next_next_next_next_next_ne->Parameters.f3,
      obj_next_next_next_next_next_ne->Parameters.f4,
      obj_next_next_next_next_next_ne->Parameters.f5, x, b_data, b_size, c);
    for (n = 0; n < ineqRange_size_idx_1; n++) {
      Cineq_workspace_data[ineqRange_data[n] - 1] = b_data[n];
    }

    std::memcpy(&Ceq_workspace[0], &c[0], 170U * sizeof(real_T));
    *status = root_model_checkVectorNonFinite(obj_next_next_next_next_next_b_,
      Cineq_workspace_data, ineq0);
    if (*status == 1) {
      *status = checkVectorNonFinite_yVQywJ02(Ceq_workspace);
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_mod_computeLinearResiduals(const real_T x[177], int32_T nVar,
  real_T workspaceIneq_data[], const int32_T workspaceIneq_size[1], int32_T
  mLinIneq, const real_T AineqT_data[], const real_T bineq_data[], int32_T ldAi)
{
  if (mLinIneq > 0) {
    int32_T loop_ub;
    int32_T scalarLB;
    int32_T vectorUB;
    loop_ub = workspaceIneq_size[0];
    if (loop_ub - 1 >= 0) {
      std::memcpy(&root_model_B.y_data_cv[0], &workspaceIneq_data[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    std::memcpy(&root_model_B.y_data_cv[0], &bineq_data[0], static_cast<uint32_T>
                (mLinIneq) * sizeof(real_T));
    if (loop_ub - 1 >= 0) {
      std::memcpy(&workspaceIneq_data[0], &root_model_B.y_data_cv[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    loop_ub = static_cast<uint16_T>(mLinIneq);
    scalarLB = (static_cast<uint16_T>(mLinIneq) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&workspaceIneq_data[k]);
      _mm_storeu_pd(&workspaceIneq_data[k], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (int32_T k{scalarLB}; k < loop_ub; k++) {
      workspaceIneq_data[k] = -workspaceIneq_data[k];
    }

    scalarLB = 0;
    vectorUB = (mLinIneq - 1) * ldAi + 1;
    for (int32_T k{1}; ldAi < 0 ? k >= vectorUB : k <= vectorUB; k += ldAi) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = k + nVar;
      for (loop_ub = k; loop_ub < e; loop_ub++) {
        c += AineqT_data[loop_ub - 1] * x[loop_ub - k];
      }

      workspaceIneq_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static real_T root_model_computeMeritFcn(real_T obj_penaltyParam, real_T fval,
  const real_T Cineq_workspace_data[], int32_T mIneq, const real_T
  Ceq_workspace[170], boolean_T evalWellDefined)
{
  real_T val;
  if (evalWellDefined) {
    real_T constrViolationEq;
    real_T constrViolationIneq;
    int32_T k;
    constrViolationEq = 0.0;
    for (k = 0; k < 170; k++) {
      constrViolationEq += std::abs(Ceq_workspace[k]);
    }

    constrViolationIneq = 0.0;
    k = static_cast<uint16_T>(mIneq);
    for (int32_T idx{0}; idx < k; idx++) {
      real_T Cineq_workspace;
      Cineq_workspace = Cineq_workspace_data[idx];
      if (Cineq_workspace > 0.0) {
        constrViolationIneq += Cineq_workspace;
      }
    }

    val = (constrViolationEq + constrViolationIneq) * obj_penaltyParam + fval;
  } else {
    val = (rtInf);
  }

  return val;
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_linesearch(boolean_T *evalWellDefined, const real_T
  bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const real_T
  WorkingSet_Aineq_data[], s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, real_T
  MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
  MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
  FcnEvaluator_next_next_next_nex, const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *FcnEvaluator_next_next_next_n_0, const s_pVH6DktXFODPXN9VaZK2hE_root_T
  *FcnEvaluator_next_next_next_n_1, boolean_T socTaken, real_T *alpha, int32_T
  *exitflag)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T phi_alpha;
  int32_T b;
  int32_T b_tmp;
  int32_T exitg1;
  int32_T k;
  int32_T mLinIneq;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T exitg2;
  boolean_T tooSmallX;
  mLinIneq = TrialState->mIneq - TrialState->mNonlinIneq;
  *alpha = 1.0;
  *exitflag = 1;
  phi_alpha = MeritFunction_phiFullStep;
  if (WorkingSet_nVar - 1 >= 0) {
    std::memcpy(&TrialState->searchDir.data[0], &TrialState->delta_x.data[0],
                static_cast<uint32_T>(WorkingSet_nVar) * sizeof(real_T));
  }

  do {
    exitg1 = 0;
    if (TrialState->FunctionEvaluations < 17700) {
      if ((*evalWellDefined) && (phi_alpha <= *alpha * 0.0001 *
           MeritFunction_phiPrimePlus + MeritFunction_phi)) {
        exitg1 = 1;
      } else {
        *alpha *= 0.7;
        b_tmp = static_cast<uint16_T>(WorkingSet_nVar);
        scalarLB = (static_cast<uint16_T>(WorkingSet_nVar) / 2) << 1;
        vectorUB = scalarLB - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          tmp_0 = _mm_loadu_pd(&TrialState->xstar.data[k]);
          _mm_storeu_pd(&TrialState->delta_x.data[k], _mm_mul_pd(_mm_set1_pd
            (*alpha), tmp_0));
        }

        for (k = scalarLB; k < b_tmp; k++) {
          TrialState->delta_x.data[k] = *alpha * TrialState->xstar.data[k];
        }

        if (socTaken) {
          phi_alpha = *alpha * *alpha;
          if ((WorkingSet_nVar >= 1) && (!(phi_alpha == 0.0))) {
            b = (WorkingSet_nVar / 2) << 1;
            vectorUB = b - 2;
            for (k = 0; k <= vectorUB; k += 2) {
              tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[k]);
              tmp = _mm_loadu_pd(&TrialState->delta_x.data[k]);
              _mm_storeu_pd(&TrialState->delta_x.data[k], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(phi_alpha), tmp_0), tmp));
            }

            for (k = b; k < WorkingSet_nVar; k++) {
              TrialState->delta_x.data[k] += phi_alpha *
                TrialState->socDirection.data[k];
            }
          }
        }

        tooSmallX = true;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= static_cast<uint16_T>(WorkingSet_nVar) - 1)) {
          if (1.0E-6 * std::fmax(1.0, std::abs(TrialState->xstarsqp[k])) <= std::
              abs(TrialState->delta_x.data[k])) {
            tooSmallX = false;
            exitg2 = true;
          } else {
            k++;
          }
        }

        if (tooSmallX) {
          *exitflag = -2;
          exitg1 = 1;
        } else {
          vectorUB = scalarLB - 2;
          for (k = 0; k <= vectorUB; k += 2) {
            tmp_0 = _mm_loadu_pd(&TrialState->xstarsqp_old[k]);
            tmp = _mm_loadu_pd(&TrialState->delta_x.data[k]);
            _mm_storeu_pd(&TrialState->xstarsqp[k], _mm_add_pd(tmp_0, tmp));
          }

          for (k = scalarLB; k < b_tmp; k++) {
            TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k] +
              TrialState->delta_x.data[k];
          }

          root_model_evalObjAndConstr(FcnEvaluator_next_next_next_nex,
            FcnEvaluator_next_next_next_n_0, FcnEvaluator_next_next_next_n_1,
            TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
            TrialState->cEq, &TrialState->sqpFval, &k);
          root_mod_computeLinearResiduals(TrialState->xstarsqp, WorkingSet_nVar,
            TrialState->cIneq.data, TrialState->cIneq.size, mLinIneq,
            WorkingSet_Aineq_data, bineq_data, WorkingSet_ldA);
          TrialState->FunctionEvaluations++;
          *evalWellDefined = (k == 1);
          phi_alpha = root_model_computeMeritFcn(MeritFunction_penaltyParam,
            TrialState->sqpFval, TrialState->cIneq.data, TrialState->mIneq,
            TrialState->cEq, *evalWellDefined);
        }
      }
    } else {
      *exitflag = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_driver(const real_T bineq_data[], const real_T lb[177],
  s_7iu1rzNoAoHigERqeOFm2G_root_T *TrialState, sG8JZ69axY52WWR6RKyApQC_root__T
  *MeritFunction, const coder_internal_stickyStruct_2_T *FcnEvaluator,
  s_3DDm1jFKx2gyv5YUdHGqJC_root_T *memspace, s_Jrk1gF3jitJmkPBgphEknE_root_T
  *WorkingSet, s_Pct27jAhkqchx6iRs2zKpD_root_T *QRManager,
  s_mdyENYu4oaiCttMAPwg9iE_root_T *CholManager, s_1ECtvwuWb7KTJMqOG7pbsD_root_T *
  QPObjective, const int32_T fscales_lineq_constraint_size[1], const int32_T
  fscales_cineq_constraint_size[1], real_T Hessian[31329])
{
  __m128d tmp;
  __m128d tmp_0;
  s7RdrPWkr8UPAUyTdDJkLaG_root__T Flags;
  somzaGboVhDG7PNQS6E98jD_root__T expl_temp;
  real_T TrialState_lambdasqp;
  int32_T b_ix;
  int32_T b_iy;
  int32_T d_ix;
  int32_T ix;
  int32_T iy;
  int32_T k;
  int32_T loop_ub;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLinIneq;
  int32_T mUB;
  int32_T nVar;
  int32_T qpoptions_MaxIterations;
  int32_T scalarLB;
  int32_T u1;
  static const int8_T s[31329]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const char_T r[7]{ 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  somzaGboVhDG7PNQS6E98jD_root__T expl_temp_0;
  for (scalarLB = 0; scalarLB < 31329; scalarLB++) {
    Hessian[scalarLB] = s[scalarLB];
  }

  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 170;
  mLinIneq = WorkingSet->sizes[2] - TrialState->mNonlinIneq;
  u1 = ((WorkingSet->sizes[2] + WorkingSet->sizes[3]) + WorkingSet->sizes[4]) +
    (WorkingSet->sizes[0] << 1);
  if (WorkingSet->nVar >= u1) {
    u1 = WorkingSet->nVar;
  }

  qpoptions_MaxIterations = 10 * u1;
  TrialState->steplength = 1.0;
  root_model_test_exit(MeritFunction, WorkingSet, TrialState, lb, &Flags.gradOK,
                       &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
                       &Flags.failedLineSearch, &Flags.stepType);
  root_model_saveJacobian(TrialState, WorkingSet->nVar, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, TrialState->iNonIneq0, WorkingSet->Aeq.data,
    WorkingSet->ldA);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 177; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old.data[k] = TrialState->grad.data[k];
  }

  loop_ub = TrialState->cIneq_old.size[0];
  k = TrialState->cIneq_old.size[0];
  if (k - 1 >= 0) {
    std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq_old.data[0],
                static_cast<uint32_T>(k) * sizeof(real_T));
  }

  if (TrialState->mIneq - 1 >= 0) {
    std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq.data[0],
                static_cast<uint32_T>(TrialState->mIneq) * sizeof(real_T));
  }

  if (loop_ub - 1 >= 0) {
    std::memcpy(&TrialState->cIneq_old.data[0], &root_model_B.y_data_k[0],
                static_cast<uint32_T>(loop_ub) * sizeof(real_T));
  }

  std::memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 170U * sizeof(real_T));
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    if ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = (rtMinusInf);
      expl_temp.ConstraintTolerance = 1.0E-6;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (scalarLB = 0; scalarLB < 7; scalarLB++) {
        expl_temp.SolverName[scalarLB] = r[scalarLB];
      }
    }

    while ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        root_m_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet, mIneq,
          TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq, mLB,
          lb, mUB, mFixed);
      }

      expl_temp_0 = expl_temp;
      root_model_step_k(&Flags, Hessian, lb, TrialState, MeritFunction, memspace,
                        WorkingSet, QRManager, CholManager, QPObjective,
                        &expl_temp_0);
      if (Flags.stepAccepted) {
        u1 = static_cast<uint16_T>(nVar);
        scalarLB = (static_cast<uint16_T>(nVar) / 2) << 1;
        d_ix = scalarLB - 2;
        for (k = 0; k <= d_ix; k += 2) {
          tmp = _mm_loadu_pd(&TrialState->xstarsqp[k]);
          tmp_0 = _mm_loadu_pd(&TrialState->delta_x.data[k]);
          _mm_storeu_pd(&TrialState->xstarsqp[k], _mm_add_pd(tmp, tmp_0));
        }

        for (k = scalarLB; k < u1; k++) {
          TrialState->xstarsqp[k] += TrialState->delta_x.data[k];
        }

        root_model_evalObjAndConstr
          (FcnEvaluator->next.next.next.next.next.b_value,
           &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
           &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
           TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
           TrialState->cEq, &TrialState->sqpFval, &k);
        Flags.fevalOK = (k == 1);
        TrialState->FunctionEvaluations++;
        root_mod_computeLinearResiduals(TrialState->xstarsqp, nVar,
          TrialState->cIneq.data, TrialState->cIneq.size, mLinIneq,
          WorkingSet->Aineq.data, bineq_data, WorkingSet->ldA);
        MeritFunction->phiFullStep = root_model_computeMeritFcn
          (MeritFunction->penaltyParam, TrialState->sqpFval,
           TrialState->cIneq.data, mIneq, TrialState->cEq, Flags.fevalOK);
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        root_model_linesearch(&Flags.fevalOK, bineq_data, WorkingSet->nVar,
                              WorkingSet->ldA, WorkingSet->Aineq.data,
                              TrialState, MeritFunction->penaltyParam,
                              MeritFunction->phi, MeritFunction->phiPrimePlus,
                              MeritFunction->phiFullStep,
                              FcnEvaluator->next.next.next.next.next.b_value,
                              &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
                              &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
                              ((Flags.stepType == 3) && Flags.stepAccepted),
                              &TrialState_lambdasqp, &k);
        TrialState->steplength = TrialState_lambdasqp;
        if (k > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      u1 = static_cast<uint16_T>(nVar);
      scalarLB = (static_cast<uint16_T>(nVar) / 2) << 1;
      d_ix = scalarLB - 2;
      for (k = 0; k <= d_ix; k += 2) {
        tmp = _mm_loadu_pd(&TrialState->xstarsqp_old[k]);
        tmp_0 = _mm_loadu_pd(&TrialState->delta_x.data[k]);
        _mm_storeu_pd(&TrialState->xstarsqp[k], _mm_add_pd(tmp, tmp_0));
      }

      for (k = scalarLB; k < u1; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k] +
          TrialState->delta_x.data[k];
      }

      u1 = static_cast<uint16_T>(mConstr);
      scalarLB = (static_cast<uint16_T>(mConstr) / 2) << 1;
      d_ix = scalarLB - 2;
      for (k = 0; k <= d_ix; k += 2) {
        tmp = _mm_loadu_pd(&TrialState->lambda.data[k]);
        tmp_0 = _mm_loadu_pd(&TrialState->lambdasqp.data[k]);
        _mm_storeu_pd(&TrialState->lambdasqp.data[k], _mm_add_pd(_mm_mul_pd
          (_mm_sub_pd(tmp, tmp_0), _mm_set1_pd(TrialState->steplength)), tmp_0));
      }

      for (k = scalarLB; k < u1; k++) {
        TrialState_lambdasqp = TrialState->lambdasqp.data[k];
        TrialState->lambdasqp.data[k] = (TrialState->lambda.data[k] -
          TrialState_lambdasqp) * TrialState->steplength + TrialState_lambdasqp;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 177; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old.data[k] = TrialState->grad.data[k];
      }

      loop_ub = TrialState->cIneq_old.size[0];
      k = TrialState->cIneq_old.size[0];
      if (k - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq_old.data[0],
                    static_cast<uint32_T>(k) * sizeof(real_T));
      }

      if (TrialState->mIneq - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq.data[0],
                    static_cast<uint32_T>(TrialState->mIneq) * sizeof(real_T));
      }

      if (loop_ub - 1 >= 0) {
        std::memcpy(&TrialState->cIneq_old.data[0], &root_model_B.y_data_k[0],
                    static_cast<uint32_T>(loop_ub) * sizeof(real_T));
      }

      std::memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 170U * sizeof
                  (real_T));
      Flags.gradOK = true;
      evalObjAndConstrAndDerivatives
        (FcnEvaluator->next.next.next.next.next.b_value,
         &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
         &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
         TrialState->xstarsqp, TrialState->grad.data, TrialState->cIneq.data,
         TrialState->iNonIneq0, TrialState->cEq, WorkingSet->Aineq.data,
         TrialState->iNonIneq0, WorkingSet->ldA, WorkingSet->Aeq.data,
         WorkingSet->ldA, &TrialState->sqpFval, &k);
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (k == 1);
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      std::memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 177U *
                  sizeof(real_T));
      loop_ub = TrialState->cIneq.size[0];
      k = TrialState->cIneq.size[0];
      if (k - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq.data[0],
                    static_cast<uint32_T>(k) * sizeof(real_T));
      }

      if (TrialState->mIneq - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_k[0], &TrialState->cIneq_old.data[0],
                    static_cast<uint32_T>(TrialState->mIneq) * sizeof(real_T));
      }

      if (loop_ub - 1 >= 0) {
        std::memcpy(&TrialState->cIneq.data[0], &root_model_B.y_data_k[0],
                    static_cast<uint32_T>(loop_ub) * sizeof(real_T));
      }

      std::memcpy(&TrialState->cEq[0], &TrialState->cEq_old[0], 170U * sizeof
                  (real_T));
    }

    root_model_test_exit_n(&Flags, memspace, MeritFunction,
      fscales_lineq_constraint_size, fscales_cineq_constraint_size, WorkingSet,
      TrialState, QRManager, lb);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      u1 = (mFixed + TrialState->iNonIneq0) + 169;
      k = WorkingSet->ldA;
      std::memcpy(&TrialState->delta_gradLag.data[0], &TrialState->grad.data[0],
                  static_cast<uint16_T>(nVar) * sizeof(real_T));
      iy = TrialState->delta_gradLag.size[0];
      loop_ub = TrialState->delta_gradLag.size[0];
      if (loop_ub - 1 >= 0) {
        std::memcpy(&root_model_B.y_data_k[0], &TrialState->delta_gradLag.data[0],
                    static_cast<uint32_T>(loop_ub) * sizeof(real_T));
      }

      if (nVar >= 1) {
        scalarLB = (nVar / 2) << 1;
        d_ix = scalarLB - 2;
        for (loop_ub = 0; loop_ub <= d_ix; loop_ub += 2) {
          tmp = _mm_loadu_pd(&root_model_B.y_data_k[loop_ub]);
          tmp_0 = _mm_loadu_pd(&TrialState->grad_old.data[loop_ub]);
          _mm_storeu_pd(&root_model_B.y_data_k[loop_ub], _mm_sub_pd(tmp, tmp_0));
        }

        for (loop_ub = scalarLB; loop_ub < nVar; loop_ub++) {
          root_model_B.y_data_k[loop_ub] -= TrialState->grad_old.data[loop_ub];
        }
      }

      ix = mFixed;
      if (iy - 1 >= 0) {
        std::memcpy(&TrialState->delta_gradLag.data[0], &root_model_B.y_data_k[0],
                    static_cast<uint32_T>(iy) * sizeof(real_T));
      }

      iy = WorkingSet->ldA * 169 + 1;
      for (loop_ub = 1; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k) {
        b_iy = loop_ub + nVar;
        for (d_ix = loop_ub; d_ix < b_iy; d_ix++) {
          scalarLB = d_ix - loop_ub;
          TrialState->delta_gradLag.data[scalarLB] += WorkingSet->Aeq.data[d_ix
            - 1] * TrialState->lambdasqp.data[ix];
        }

        ix++;
      }

      b_ix = mFixed;
      for (loop_ub = 1; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k) {
        b_iy = loop_ub + nVar;
        for (d_ix = loop_ub; d_ix < b_iy; d_ix++) {
          scalarLB = d_ix - loop_ub;
          TrialState->delta_gradLag.data[scalarLB] +=
            TrialState->JacCeqTrans_old.data[d_ix - 1] *
            -TrialState->lambdasqp.data[b_ix];
        }

        b_ix++;
      }

      if (TrialState->mNonlinIneq > 0) {
        ix = (TrialState->iNonIneq0 - 1) * WorkingSet->ldA + 1;
        b_ix = u1;
        iy = (TrialState->mNonlinIneq - 1) * WorkingSet->ldA + ix;
        for (loop_ub = ix; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k)
        {
          b_iy = loop_ub + nVar;
          for (d_ix = loop_ub; d_ix < b_iy; d_ix++) {
            scalarLB = d_ix - loop_ub;
            TrialState->delta_gradLag.data[scalarLB] += WorkingSet->
              Aineq.data[d_ix - 1] * TrialState->lambdasqp.data[b_ix];
          }

          b_ix++;
        }

        d_ix = u1;
        b_ix = (TrialState->mNonlinIneq - 1) * WorkingSet->ldA + 1;
        for (u1 = 1; k < 0 ? u1 >= b_ix : u1 <= b_ix; u1 += k) {
          b_iy = u1 + nVar;
          for (loop_ub = u1; loop_ub < b_iy; loop_ub++) {
            scalarLB = loop_ub - u1;
            TrialState->delta_gradLag.data[scalarLB] +=
              TrialState->JacCineqTrans_old.data[loop_ub - 1] *
              -TrialState->lambdasqp.data[d_ix];
          }

          d_ix++;
        }
      }

      root_model_saveJacobian(TrialState, nVar, mIneq, WorkingSet->Aineq.data,
        TrialState->iNonIneq0, WorkingSet->Aeq.data, WorkingSet->ldA);
      root_model_BFGSUpdate(nVar, Hessian, TrialState->delta_x.data,
                            TrialState->delta_gradLag.data,
                            memspace->workspace_float.data);
      TrialState->sqpIterations++;
    }
  }
}

/* Function for MATLAB Function: '<S12>/NLMPC' */
static void root_model_fmincon(const s_WBLIzjzqQS4bg0a8DMrwYD_root_T
  *fun_workspace_runtimedata, const scI1GGq4Cpah2bZUcB8s80D_root__T
  *fun_workspace_userdata, const real_T x0[177], const real_T Aineq_data[],
  const real_T bineq_data[], const int32_T bineq_size[1], const real_T lb[177],
  const s_WBLIzjzqQS4bg0a8DMrwYD_root_T *nonlcon_workspace_runtimedata, real_T
  x[177], real_T *fval, real_T *exitflag, sttYSJM5GCi2c1Eu0R50efC_root__T
  *output)
{
  __m128d tmp;
  sG8JZ69axY52WWR6RKyApQC_root__T MeritFunction;
  real_T Cineq_data[260];
  real_T Ceq[170];
  real_T absxk;
  real_T b_c;
  real_T scale;
  real_T t;
  int32_T Cineq_size[2];
  int32_T JacCineqTrans_size[2];
  int32_T tmp_size[1];
  int32_T tmp_size_0[1];
  int32_T b_idx;
  int32_T exitg1;
  int32_T idx_row;
  int32_T k;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T mLinIneq;
  int32_T mLinIneq_tmp;
  int32_T mNonlinIneq;
  int32_T maxDims;
  int32_T vectorUB;
  int16_T WorkingSet_tmp[5];
  int16_T WorkingSet_tmp_0;
  root_model_c4_mpclib_anonFcn2(nonlcon_workspace_runtimedata->x,
    nonlcon_workspace_runtimedata->OutputMin,
    nonlcon_workspace_runtimedata->OutputMax,
    nonlcon_workspace_runtimedata->Parameters.f1,
    nonlcon_workspace_runtimedata->Parameters.f2,
    nonlcon_workspace_runtimedata->Parameters.f3,
    nonlcon_workspace_runtimedata->Parameters.f4,
    nonlcon_workspace_runtimedata->Parameters.f5, x0, Cineq_data, Cineq_size,
    Ceq, root_model_B.JacCineqTrans_data, JacCineqTrans_size,
    root_model_B.JacCeqTrans);
  mNonlinIneq = Cineq_size[0] * Cineq_size[1];
  mLinIneq_tmp = bineq_size[0];
  mIneq = bineq_size[0] + mNonlinIneq;
  mConstrMax = (mIneq + mIneq) + 865;
  if (mIneq + 518 >= mConstrMax) {
    maxDims = mIneq + 518;
  } else {
    maxDims = mConstrMax;
  }

  root_model_factoryConstruct(mIneq + 518, mConstrMax, mIneq, mNonlinIneq,
    &root_model_B.TrialState);
  std::memcpy(&root_model_B.TrialState.xstarsqp[0], &x0[0], 177U * sizeof(real_T));
  root_model_B.FcnEvaluator.next.next.next.next.next.b_value = mNonlinIneq;
  root_model_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata
    = *nonlcon_workspace_runtimedata;
  root_model_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata
    = *fun_workspace_runtimedata;
  root_model_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.userdata
    = *fun_workspace_userdata;
  root_mode_factoryConstruct_n0cp(mIneq + 518,
    root_model_B.QPObjective.grad.size, root_model_B.QPObjective.Hx.size,
    &root_model_B.QPObjective.hasLinear, &root_model_B.QPObjective.nvar,
    &root_model_B.QPObjective.maxVar, &root_model_B.QPObjective.beta,
    &root_model_B.QPObjective.rho, &root_model_B.QPObjective.objtype,
    &root_model_B.QPObjective.prev_objtype, &root_model_B.QPObjective.prev_nvar,
    &root_model_B.QPObjective.prev_hasLinear,
    &root_model_B.QPObjective.gammaScalar);
  root_model_B.QPObjective.nvar = 177;
  root_model_B.QPObjective.hasLinear = true;
  root_model_B.QPObjective.objtype = 3;
  root_model_B.memspace.workspace_float.size[0] = maxDims;
  root_model_B.memspace.workspace_float.size[1] = mIneq + 518;
  root_model_B.memspace.workspace_int.size[0] = maxDims;
  root_model_B.memspace.workspace_sort.size[0] = maxDims;
  root_mod_factoryConstruct_n0cp3(mIneq, mIneq + 518, mConstrMax,
    &root_model_B.WorkingSet);
  k = -1;
  for (b_idx = 0; b_idx < 177; b_idx++) {
    b_c = lb[b_idx];
    if ((!std::isinf(b_c)) && (!std::isnan(b_c))) {
      k++;
      root_model_B.WorkingSet.indexLB.data[k] = b_idx + 1;
    }
  }

  root_model_B.WorkingSet.mConstrMax = mConstrMax;
  b_idx = mIneq + k;
  root_model_B.WorkingSet.mConstr = b_idx + 171;
  root_model_B.WorkingSet.mConstrOrig = b_idx + 171;
  WorkingSet_tmp[0] = 0;
  WorkingSet_tmp[1] = 170;
  WorkingSet_tmp[2] = static_cast<int16_T>(mIneq);
  WorkingSet_tmp[3] = static_cast<int16_T>(k + 1);
  WorkingSet_tmp[4] = 0;
  for (mConstrMax = 0; mConstrMax < 5; mConstrMax++) {
    WorkingSet_tmp_0 = WorkingSet_tmp[mConstrMax];
    root_model_B.WorkingSet.sizes[mConstrMax] = WorkingSet_tmp_0;
    root_model_B.WorkingSet.sizesNormal[mConstrMax] = WorkingSet_tmp_0;
  }

  root_model_B.WorkingSet.sizesPhaseOne[0] = 0;
  root_model_B.WorkingSet.sizesPhaseOne[1] = 170;
  root_model_B.WorkingSet.sizesPhaseOne[2] = mIneq;
  root_model_B.WorkingSet.sizesPhaseOne[3] = k + 2;
  root_model_B.WorkingSet.sizesPhaseOne[4] = 0;
  root_model_B.WorkingSet.sizesRegularized[0] = 0;
  root_model_B.WorkingSet.sizesRegularized[1] = 170;
  root_model_B.WorkingSet.sizesRegularized[2] = mIneq;
  root_model_B.WorkingSet.sizesRegularized[3] = b_idx + 341;
  root_model_B.WorkingSet.sizesRegularized[4] = 0;
  root_model_B.WorkingSet.sizesRegPhaseOne[0] = 0;
  root_model_B.WorkingSet.sizesRegPhaseOne[1] = 170;
  root_model_B.WorkingSet.sizesRegPhaseOne[2] = mIneq;
  root_model_B.WorkingSet.sizesRegPhaseOne[3] = b_idx + 342;
  root_model_B.WorkingSet.sizesRegPhaseOne[4] = 0;
  root_model_B.WorkingSet.isActiveIdxNormal[0] = 1;
  root_model_B.WorkingSet.isActiveIdxNormal[1] = 0;
  root_model_B.WorkingSet.isActiveIdxNormal[2] = 170;
  root_model_B.WorkingSet.isActiveIdxNormal[3] = mIneq;
  root_model_B.WorkingSet.isActiveIdxNormal[4] = k + 1;
  root_model_B.WorkingSet.isActiveIdxNormal[5] = 0;
  for (mLinIneq = 0; mLinIneq < 6; mLinIneq++) {
    root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mLinIneq] =
      root_model_B.WorkingSet.isActiveIdxNormal[mLinIneq];
  }

  for (mConstrMax = 0; mConstrMax < 5; mConstrMax++) {
    root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax + 1] +=
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
  }

  for (mLinIneq = 0; mLinIneq < 6; mLinIneq++) {
    root_model_B.WorkingSet.isActiveIdx[mLinIneq] =
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mLinIneq];
  }

  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 170;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[4] = k + 2;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (mConstrMax = 0; mConstrMax < 5; mConstrMax++) {
    root_model_B.WorkingSet.isActiveIdxNormal[mConstrMax + 1] +=
      root_model_B.WorkingSet.isActiveIdxNormal[mConstrMax];
    root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax + 1] +=
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
  }

  for (mLinIneq = 0; mLinIneq < 6; mLinIneq++) {
    root_model_B.WorkingSet.isActiveIdxPhaseOne[mLinIneq] =
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mLinIneq];
  }

  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 170;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[4] = b_idx + 341;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (mConstrMax = 0; mConstrMax < 5; mConstrMax++) {
    root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax + 1] +=
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
  }

  for (mLinIneq = 0; mLinIneq < 6; mLinIneq++) {
    root_model_B.WorkingSet.isActiveIdxRegularized[mLinIneq] =
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mLinIneq];
  }

  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 170;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[4] = b_idx + 342;
  root_model_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (mConstrMax = 0; mConstrMax < 5; mConstrMax++) {
    root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax + 1] +=
      root_model_B.WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
  }

  if (mIneq > 0) {
    for (b_idx = 0; b_idx < mLinIneq_tmp; b_idx++) {
      for (idx_row = 0; idx_row < 177; idx_row++) {
        root_model_B.WorkingSet.Aineq.data[idx_row + root_model_B.WorkingSet.ldA
          * b_idx] = Aineq_data[mLinIneq_tmp * idx_row + b_idx];
      }
    }
  }

  for (mConstrMax = 0; mConstrMax <= k; mConstrMax++) {
    b_idx = root_model_B.WorkingSet.indexLB.data[mConstrMax];
    root_model_B.TrialState.xstarsqp[b_idx - 1] = std::fmax
      (root_model_B.TrialState.xstarsqp[b_idx - 1], lb[b_idx - 1]);
  }

  evalObjAndConstrAndDerivatives(mNonlinIneq, nonlcon_workspace_runtimedata,
    &root_model_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace,
    root_model_B.TrialState.xstarsqp, root_model_B.TrialState.grad.data,
    root_model_B.TrialState.cIneq.data, root_model_B.TrialState.iNonIneq0,
    root_model_B.TrialState.cEq, root_model_B.WorkingSet.Aineq.data,
    root_model_B.TrialState.iNonIneq0, root_model_B.WorkingSet.ldA,
    root_model_B.WorkingSet.Aeq.data, root_model_B.WorkingSet.ldA,
    &root_model_B.TrialState.sqpFval, &mConstrMax);
  root_model_B.TrialState.FunctionEvaluations = 1;
  mConstrMax = root_model_B.WorkingSet.ldA;
  if (bineq_size[0] > 0) {
    idx_row = root_model_B.TrialState.cIneq.size[0];
    if (idx_row - 1 >= 0) {
      std::memcpy(&root_model_B.y_data_c[0],
                  &root_model_B.TrialState.cIneq.data[0], static_cast<uint32_T>
                  (idx_row) * sizeof(real_T));
    }

    if (mLinIneq_tmp - 1 >= 0) {
      std::memcpy(&root_model_B.y_data_c[0], &bineq_data[0],
                  static_cast<uint32_T>(mLinIneq_tmp) * sizeof(real_T));
    }

    if (idx_row - 1 >= 0) {
      std::memcpy(&root_model_B.TrialState.cIneq.data[0],
                  &root_model_B.y_data_c[0], static_cast<uint32_T>(idx_row) *
                  sizeof(real_T));
    }

    idx_row = (bineq_size[0] / 2) << 1;
    vectorUB = idx_row - 2;
    for (b_idx = 0; b_idx <= vectorUB; b_idx += 2) {
      tmp = _mm_loadu_pd(&root_model_B.TrialState.cIneq.data[b_idx]);
      _mm_storeu_pd(&root_model_B.TrialState.cIneq.data[b_idx], _mm_mul_pd(tmp,
        _mm_set1_pd(-1.0)));
    }

    for (b_idx = idx_row; b_idx < mLinIneq_tmp; b_idx++) {
      root_model_B.TrialState.cIneq.data[b_idx] =
        -root_model_B.TrialState.cIneq.data[b_idx];
    }

    mLinIneq = 0;
    b_idx = (bineq_size[0] - 1) * root_model_B.WorkingSet.ldA + 1;
    for (idx_row = 1; mConstrMax < 0 ? idx_row >= b_idx : idx_row <= b_idx;
         idx_row += mConstrMax) {
      b_c = 0.0;
      for (vectorUB = idx_row; vectorUB <= idx_row + 176; vectorUB++) {
        b_c += root_model_B.WorkingSet.Aineq.data[vectorUB - 1] *
          root_model_B.TrialState.xstarsqp[vectorUB - idx_row];
      }

      root_model_B.TrialState.cIneq.data[mLinIneq] += b_c;
      mLinIneq++;
    }
  }

  for (mLinIneq = 0; mLinIneq <= 168; mLinIneq += 2) {
    tmp = _mm_loadu_pd(&root_model_B.TrialState.cEq[mLinIneq]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(-1.0));
    _mm_storeu_pd(&root_model_B.WorkingSet.beq[mLinIneq], tmp);
    _mm_storeu_pd(&root_model_B.WorkingSet.bwset.data[mLinIneq], tmp);
  }

  mLinIneq = 0;
  mConstrMax = 0;
  for (b_idx = 0; b_idx < 170; b_idx++) {
    std::memcpy(&root_model_B.WorkingSet.ATwset.data[mLinIneq],
                &root_model_B.WorkingSet.Aeq.data[mConstrMax], 177U * sizeof
                (real_T));
    mLinIneq += root_model_B.WorkingSet.ldA;
    mConstrMax = mLinIneq;
  }

  idx_row = (mIneq / 2) << 1;
  vectorUB = idx_row - 2;
  for (mLinIneq = 0; mLinIneq <= vectorUB; mLinIneq += 2) {
    tmp = _mm_loadu_pd(&root_model_B.TrialState.cIneq.data[mLinIneq]);
    _mm_storeu_pd(&root_model_B.WorkingSet.bineq.data[mLinIneq], _mm_mul_pd(tmp,
      _mm_set1_pd(-1.0)));
  }

  for (mLinIneq = idx_row; mLinIneq < mIneq; mLinIneq++) {
    root_model_B.WorkingSet.bineq.data[mLinIneq] =
      -root_model_B.TrialState.cIneq.data[mLinIneq];
  }

  for (mLinIneq = 0; mLinIneq <= k; mLinIneq++) {
    root_model_B.WorkingSet.lb.data[root_model_B.WorkingSet.indexLB.data[mLinIneq]
      - 1] = -lb[root_model_B.WorkingSet.indexLB.data[mLinIneq] - 1] +
      x0[root_model_B.WorkingSet.indexLB.data[mLinIneq] - 1];
  }

  root_model_initActiveSet(&root_model_B.WorkingSet);
  MeritFunction.initFval = root_model_B.TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  b_c = 0.0;
  for (k = 0; k < 170; k++) {
    b_c += std::abs(root_model_B.TrialState.cEq[k]);
  }

  MeritFunction.initConstrViolationEq = b_c;
  b_c = 0.0;
  for (k = 0; k < mIneq; k++) {
    scale = root_model_B.TrialState.cIneq.data[k];
    if (scale > 0.0) {
      b_c += scale;
    }
  }

  MeritFunction.initConstrViolationIneq = b_c;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  root_model_factoryConstruct_n0(maxDims, maxDims, &root_model_B.QRManager.ldq,
    root_model_B.QRManager.QR.size, root_model_B.QRManager.Q.data,
    root_model_B.QRManager.Q.size, root_model_B.QRManager.jpvt.data,
    root_model_B.QRManager.jpvt.size, &root_model_B.QRManager.mrows,
    &root_model_B.QRManager.ncols, root_model_B.QRManager.tau.size,
    &root_model_B.QRManager.minRowCol, &root_model_B.QRManager.usedPivoting);
  root_model_factoryConstruct_n0c(maxDims, root_model_B.CholManager.FMat.size,
    &root_model_B.CholManager.ldm, &root_model_B.CholManager.ndims,
    &root_model_B.CholManager.info, &root_model_B.CholManager.scaleFactor,
    &root_model_B.CholManager.ConvexCheck, &root_model_B.CholManager.regTol_,
    &root_model_B.CholManager.workspace_, &root_model_B.CholManager.workspace2_);
  tmp_size[0] = bineq_size[0];
  tmp_size_0[0] = mNonlinIneq;
  root_model_driver(bineq_data, lb, &root_model_B.TrialState, &MeritFunction,
                    &root_model_B.FcnEvaluator, &root_model_B.memspace,
                    &root_model_B.WorkingSet, &root_model_B.QRManager,
                    &root_model_B.CholManager, &root_model_B.QPObjective,
                    tmp_size, tmp_size_0, root_model_B.unusedExpr);
  *fval = root_model_B.TrialState.sqpFval;
  *exitflag = root_model_B.TrialState.sqpExitFlag;
  output->iterations = root_model_B.TrialState.sqpIterations;
  output->funcCount = root_model_B.TrialState.FunctionEvaluations;
  output->algorithm[0] = 's';
  output->algorithm[1] = 'q';
  output->algorithm[2] = 'p';
  output->constrviolation = MeritFunction.nlpPrimalFeasError;
  b_c = 0.0;
  scale = 3.3121686421112381E-170;
  for (mNonlinIneq = 0; mNonlinIneq < 177; mNonlinIneq++) {
    x[mNonlinIneq] = root_model_B.TrialState.xstarsqp[mNonlinIneq];
    absxk = std::abs(root_model_B.TrialState.delta_x.data[mNonlinIneq]);
    if (absxk > scale) {
      t = scale / absxk;
      b_c = b_c * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_c += t * t;
    }
  }

  b_c = scale * std::sqrt(b_c);
  if (std::isnan(b_c)) {
    mNonlinIneq = 0;
    do {
      exitg1 = 0;
      if (mNonlinIneq < 177) {
        if (std::isnan(root_model_B.TrialState.delta_x.data[mNonlinIneq])) {
          exitg1 = 1;
        } else {
          mNonlinIneq++;
        }
      } else {
        b_c = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  output->stepsize = b_c;
  output->lssteplength = root_model_B.TrialState.steplength;
  output->firstorderopt = MeritFunction.firstOrderOpt;
}

static real_T root_model_xnrm2_lx(int32_T n, const real_T x[578], int32_T ix0)
{
  real_T y;

  /* Start for MATLABSystem: '<S10>/MATLAB System' */
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T k;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
      if (std::isnan(y)) {
        k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k <= kend) {
            if (std::isnan(x[k - 1])) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            y = (rtInf);
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  /* End of Start for MATLABSystem: '<S10>/MATLAB System' */
  return y;
}

static void root_model_qrFactor_lx(const real_T A[289], const real_T S[289],
  const real_T Ns[289], real_T b_S[289])
{
  __m128d tmp;
  real_T b_R[289];
  real_T b_tau[17];
  real_T work[17];
  real_T M;
  real_T b_atmp;
  real_T xnorm;
  int32_T b_i;
  int32_T d;
  int32_T exitg1;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lastc;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T exitg2;

  /* Start for MATLABSystem: '<S10>/MATLAB System' */
  for (i = 0; i < 17; i++) {
    for (b_i = 0; b_i < 17; b_i++) {
      b_atmp = 0.0;
      for (ii = 0; ii < 17; ii++) {
        b_atmp += S[17 * b_i + ii] * A[17 * ii + i];
      }

      ii = 34 * i + b_i;
      root_model_B.M[ii] = b_atmp;
      root_model_B.M[ii + 17] = Ns[17 * b_i + i];
    }

    b_tau[i] = 0.0;
    work[i] = 0.0;
  }

  for (b_i = 0; b_i < 17; b_i++) {
    ii = b_i * 34 + b_i;
    lastv = ii + 2;
    b_atmp = root_model_B.M[ii];
    b_tau[b_i] = 0.0;
    xnorm = root_model_xnrm2_lx(33 - b_i, root_model_B.M, ii + 2);
    if (xnorm != 0.0) {
      M = root_model_B.M[ii];
      xnorm = rt_hypotd_snf(M, xnorm);
      if (M >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - b_i;
          scalarLB = (((((i - ii) + 33) / 2) << 1) + ii) + 2;
          vectorUB = scalarLB - 2;
          for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
            tmp = _mm_loadu_pd(&root_model_B.M[lastc - 1]);
            _mm_storeu_pd(&root_model_B.M[lastc - 1], _mm_mul_pd(tmp,
              _mm_set1_pd(9.9792015476736E+291)));
          }

          for (lastc = scalarLB; lastc <= i + 34; lastc++) {
            root_model_B.M[lastc - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt + 1 < 20));

        xnorm = rt_hypotd_snf(b_atmp, root_model_xnrm2_lx(33 - b_i,
          root_model_B.M, ii + 2));
        if (b_atmp >= 0.0) {
          xnorm = -xnorm;
        }

        b_tau[b_i] = (xnorm - b_atmp) / xnorm;
        b_atmp = 1.0 / (b_atmp - xnorm);
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&root_model_B.M[lastc - 1]);
          _mm_storeu_pd(&root_model_B.M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 34; lastc++) {
          root_model_B.M[lastc - 1] *= b_atmp;
        }

        for (lastc = 0; lastc <= knt; lastc++) {
          xnorm *= 1.0020841800044864E-292;
        }

        b_atmp = xnorm;
      } else {
        b_tau[b_i] = (xnorm - M) / xnorm;
        b_atmp = 1.0 / (M - xnorm);
        i = ii - b_i;
        scalarLB = (((((i - ii) + 33) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (lastc = lastv; lastc <= vectorUB; lastc += 2) {
          tmp = _mm_loadu_pd(&root_model_B.M[lastc - 1]);
          _mm_storeu_pd(&root_model_B.M[lastc - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (b_atmp)));
        }

        for (lastc = scalarLB; lastc <= i + 34; lastc++) {
          root_model_B.M[lastc - 1] *= b_atmp;
        }

        b_atmp = xnorm;
      }
    }

    root_model_B.M[ii] = b_atmp;
    if (b_i + 1 < 17) {
      root_model_B.M[ii] = 1.0;
      scalarLB = ii + 35;
      if (b_tau[b_i] != 0.0) {
        lastv = 34 - b_i;
        i = ii - b_i;
        while ((lastv > 0) && (root_model_B.M[i + 33] == 0.0)) {
          lastv--;
          i--;
        }

        lastc = 15 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          i = lastc * 34 + ii;
          knt = i + 35;
          do {
            exitg1 = 0;
            if (knt <= (i + lastv) + 34) {
              if (root_model_B.M[knt - 1] != 0.0) {
                exitg1 = 1;
              } else {
                knt++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }

      if (lastv > 0) {
        if (lastc + 1 != 0) {
          if (lastc >= 0) {
            std::memset(&work[0], 0, static_cast<uint32_T>(lastc + 1) * sizeof
                        (real_T));
          }

          i = 34 * lastc + ii;
          for (vectorUB = scalarLB; vectorUB <= i + 35; vectorUB += 34) {
            xnorm = 0.0;
            d = vectorUB + lastv;
            for (knt = vectorUB; knt < d; knt++) {
              xnorm += root_model_B.M[(ii + knt) - vectorUB] *
                root_model_B.M[knt - 1];
            }

            knt = div_nde_s32_floor((vectorUB - ii) - 35, 34);
            work[knt] += xnorm;
          }
        }

        if (!(-b_tau[b_i] == 0.0)) {
          knt = ii;
          for (scalarLB = 0; scalarLB <= lastc; scalarLB++) {
            xnorm = work[scalarLB];
            if (xnorm != 0.0) {
              xnorm *= -b_tau[b_i];
              i = knt + 35;
              vectorUB = (lastv + knt) + 34;
              for (d = i; d <= vectorUB; d++) {
                root_model_B.M[d - 1] += root_model_B.M[((ii + d) - knt) - 35] *
                  xnorm;
              }
            }

            knt += 34;
          }
        }
      }

      root_model_B.M[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 17; ii++) {
    for (b_i = 0; b_i <= ii; b_i++) {
      /* Start for MATLABSystem: '<S10>/MATLAB System' */
      b_R[b_i + 17 * ii] = root_model_B.M[34 * ii + b_i];
    }

    for (b_i = ii + 2; b_i < 18; b_i++) {
      b_R[(b_i + 17 * ii) - 1] = 0.0;
    }
  }

  for (b_i = 0; b_i < 17; b_i++) {
    for (ii = 0; ii < 17; ii++) {
      /* Start for MATLABSystem: '<S10>/MATLAB System' */
      b_S[ii + 17 * b_i] = b_R[17 * ii + b_i];
    }
  }
}

/* Model step function */
void root_model_step(void)
{
  __m128d tmp_7;
  __m128d tmp_8;
  __m128d tmp_9;
  __m128d tmp_a;
  __m128d tmp_b;
  __m128d tmp_c;
  __m128d tmp_d;
  scI1GGq4Cpah2bZUcB8s80D_root__T expl_temp;
  sttYSJM5GCi2c1Eu0R50efC_root__T Out;
  real_T A[289];
  real_T dFdx[289];
  real_T C[221];
  real_T K[221];
  real_T b_dHdx[221];
  real_T X[187];
  real_T tmp_4[177];
  real_T tmp_5[177];
  real_T z[177];
  real_T tmp_3[170];
  real_T Sy[169];
  real_T Sy_0[169];
  real_T B_data[120];
  real_T U[33];
  real_T tmp_0[30];
  real_T tmp_1[30];
  real_T imvec[17];
  real_T imz[17];
  real_T rtb_DataStoreReadX[17];
  real_T rtb_MATLABSystem_o1_0[17];
  real_T z_0[17];
  real_T tmp_2[13];
  real_T b[9];
  real_T b_0[9];
  real_T rtb_Product8_0[9];
  real_T a[6];
  real_T b_1[3];
  real_T tmp_6[2];
  real_T b_tmp;
  real_T relDist;
  real_T rtb_Product2_ki;
  real_T rtb_Product8;
  real_T rtb_Product_o;
  real_T rtb_prevbodyquaternion_idx_0;
  real_T rtb_prevbodyquaternion_idx_1;
  real_T rtb_prevbodyquaternion_idx_2;
  real_T rtb_prevbodyquaternion_idx_3;
  int32_T A_tmp;
  int32_T b_tmp_0;
  int32_T ctY;
  int32_T i;
  boolean_T tmp;
  boolean_T tmp_e;
  static const real_T c[30]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659, -0.3490658503988659,
    -0.3490658503988659, -0.3490658503988659 };

  static const real_T d[30]{ 167.0, 167.0, 167.0, 167.0, 167.0, 167.0, 167.0,
    167.0, 167.0, 167.0, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659,
    0.3490658503988659, 0.3490658503988659, 0.3490658503988659 };

  static const int8_T a_0[180]{ 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T b_2[3]{ 1, 0, 0 };

  static const int8_T c_0[3]{ 0, 1, 0 };

  static const int8_T b_b[3]{ 0, 0, 1 };

  int32_T A_size[2];
  int32_T B_size[1];
  if (rtmIsMajorTimeStep(root_model_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&root_model_M->solverInfo,
                          ((root_model_M->Timing.clockTick0+1)*
      root_model_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(root_model_M)) {
    root_model_M->Timing.t[0] = rtsiGetT(&root_model_M->solverInfo);
  }

  /* EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' incorporates:
   *  Concatenate: '<Root>/Vector Concatenate2'
   */
  root_model_B.VectorConcatenate2[10] = root_model_X.uDOFQuaternion_CSTATE[6];
  root_model_B.VectorConcatenate2[11] = root_model_X.uDOFQuaternion_CSTATE[7];
  root_model_B.VectorConcatenate2[12] = root_model_X.uDOFQuaternion_CSTATE[8];
  rtb_prevbodyquaternion_idx_0 = root_model_X.uDOFQuaternion_CSTATE[9];
  rtb_prevbodyquaternion_idx_1 = root_model_X.uDOFQuaternion_CSTATE[10];
  rtb_prevbodyquaternion_idx_2 = root_model_X.uDOFQuaternion_CSTATE[11];
  rtb_prevbodyquaternion_idx_3 = root_model_X.uDOFQuaternion_CSTATE[12];
  tmp_8 = _mm_set1_pd(1.0);
  tmp_7 = _mm_mul_pd(_mm_div_pd(tmp_8, _mm_set1_pd(std::sqrt
    (((root_model_X.uDOFQuaternion_CSTATE[9] *
       root_model_X.uDOFQuaternion_CSTATE[9] +
       root_model_X.uDOFQuaternion_CSTATE[10] *
       root_model_X.uDOFQuaternion_CSTATE[10]) +
      root_model_X.uDOFQuaternion_CSTATE[11] *
      root_model_X.uDOFQuaternion_CSTATE[11]) +
     root_model_X.uDOFQuaternion_CSTATE[12] *
     root_model_X.uDOFQuaternion_CSTATE[12]))), _mm_loadu_pd
                     (&root_model_X.uDOFQuaternion_CSTATE[9]));
  _mm_storeu_pd(&tmp_6[0], tmp_7);

  /* EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  rtb_Product_o = tmp_6[0];
  rtb_Product2_ki = tmp_6[1];
  tmp_8 = _mm_mul_pd(_mm_div_pd(tmp_8, _mm_set1_pd(std::sqrt
    (((root_model_X.uDOFQuaternion_CSTATE[9] *
       root_model_X.uDOFQuaternion_CSTATE[9] +
       root_model_X.uDOFQuaternion_CSTATE[10] *
       root_model_X.uDOFQuaternion_CSTATE[10]) +
      root_model_X.uDOFQuaternion_CSTATE[11] *
      root_model_X.uDOFQuaternion_CSTATE[11]) +
     root_model_X.uDOFQuaternion_CSTATE[12] *
     root_model_X.uDOFQuaternion_CSTATE[12]))), _mm_loadu_pd
                     (&root_model_X.uDOFQuaternion_CSTATE[11]));
  _mm_storeu_pd(&tmp_6[0], tmp_8);

  /* EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  tmp_8 = _mm_set1_pd(rtb_Product_o);
  tmp_7 = _mm_set1_pd(rtb_Product2_ki);
  tmp_9 = _mm_set1_pd(tmp_6[0]);
  tmp_a = _mm_set1_pd(tmp_6[1]);
  tmp_b = _mm_set_pd(1.0, -1.0);
  tmp_c = _mm_set_pd(-1.0, 1.0);

  /* EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  tmp_d = _mm_set1_pd(2.0);
  tmp_8 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_add_pd(_mm_add_pd
    (_mm_mul_pd(tmp_8, tmp_8), _mm_mul_pd(_mm_mul_pd(tmp_7, tmp_7), tmp_c)),
    _mm_mul_pd(_mm_mul_pd(tmp_9, tmp_9), tmp_b)), _mm_mul_pd(tmp_a, tmp_a)),
    _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[0])), _mm_mul_pd(_mm_mul_pd
    (_mm_add_pd(_mm_mul_pd(tmp_7, tmp_9), _mm_mul_pd(_mm_mul_pd(tmp_8, tmp_a),
    tmp_b)), tmp_d), _mm_set_pd(root_model_X.uDOFQuaternion_CSTATE[0],
    root_model_X.uDOFQuaternion_CSTATE[1]))), _mm_mul_pd(_mm_mul_pd(_mm_add_pd
    (_mm_mul_pd(_mm_set_pd(tmp_6[0], rtb_Product2_ki), tmp_a), _mm_mul_pd
     (_mm_mul_pd(tmp_8, _mm_set_pd(rtb_Product2_ki, tmp_6[0])), tmp_c)), tmp_d),
    _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[2])));

  /* Concatenate: '<Root>/Vector Concatenate2' incorporates:
   *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
   */
  _mm_storeu_pd(&root_model_B.VectorConcatenate2[3], tmp_8);

  /* EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' incorporates:
   *  Concatenate: '<Root>/Vector Concatenate2'
   */
  root_model_B.VectorConcatenate2[5] = ((rtb_Product2_ki * tmp_6[1] -
    rtb_Product_o * tmp_6[0]) * 2.0 * root_model_X.uDOFQuaternion_CSTATE[0] +
    (tmp_6[0] * tmp_6[1] + rtb_Product_o * rtb_Product2_ki) * 2.0 *
    root_model_X.uDOFQuaternion_CSTATE[1]) + (((rtb_Product_o * rtb_Product_o -
    rtb_Product2_ki * rtb_Product2_ki) - tmp_6[0] * tmp_6[0]) + tmp_6[1] *
    tmp_6[1]) * root_model_X.uDOFQuaternion_CSTATE[2];

  /* Checks_PiecewiseLinearBound: '<S1>/PWLCheck' */
  root_model_B.PWLCheck = true;

  /* Checks_PiecewiseLinearBound: '<S1>/PWLCheck' incorporates:
   *  Checks_PiecewiseLinearBound: '<S2>/PWLCheck'
   */
  rtb_Product2_ki = root_model_M->Timing.t[0];
  if ((rtb_Product2_ki >= 0.0) && (rtb_Product2_ki <= 5.0)) {
    relDist = rtb_Product2_ki / 5.0;

    /* Checks_PiecewiseLinearBound: '<S1>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck = ((!(root_model_X.uDOFQuaternion_CSTATE[3] > (1.0 -
      relDist) * 1.1 + relDist * 1.1)) && root_model_B.PWLCheck);
  }

  if ((rtb_Product2_ki >= 5.0) && (rtb_Product2_ki <= 10.0)) {
    relDist = (rtb_Product2_ki - 5.0) / 5.0;

    /* Checks_PiecewiseLinearBound: '<S1>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck = ((!(root_model_X.uDOFQuaternion_CSTATE[3] > (1.0 -
      relDist) * 1.01 + relDist * 1.01)) && root_model_B.PWLCheck);
  }

  tmp = rtmIsMajorTimeStep(root_model_M);
  if (tmp) {
    /* Assertion: '<S1>/Assertion' */
    utAssert(root_model_B.PWLCheck);
  }

  /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' */
  root_model_B.PWLCheck_m = true;

  /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' */
  if ((rtb_Product2_ki >= 0.0) && (rtb_Product2_ki <= 30.0)) {
    relDist = rtb_Product2_ki / 30.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] > (1.0 -
      relDist) * 55.0 + relDist * 55.0)) && root_model_B.PWLCheck_m);
  }

  tmp_e = ((rtb_Product2_ki >= 30.0) && (rtb_Product2_ki <= 45.0));
  if (tmp_e) {
    relDist = (rtb_Product2_ki - 30.0) / 15.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] > (1.0 -
      relDist) * 50.5 + relDist * 50.5)) && root_model_B.PWLCheck_m);
  }

  if (rtb_Product2_ki > 45.0) {
    relDist = (rtb_Product2_ki - 30.0) / 15.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] > (1.0 -
      relDist) * 50.5 + relDist * 50.5)) && root_model_B.PWLCheck_m);
  }

  if ((rtb_Product2_ki >= 0.0) && (rtb_Product2_ki <= 20.0)) {
    relDist = rtb_Product2_ki / 20.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] < (1.0 -
      relDist) * -0.5 + relDist * -0.5)) && root_model_B.PWLCheck_m);
  }

  if ((rtb_Product2_ki >= 20.0) && (rtb_Product2_ki <= 30.0)) {
    relDist = (rtb_Product2_ki - 20.0) / 10.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] < (1.0 -
      relDist) * 40.0 + relDist * 40.0)) && root_model_B.PWLCheck_m);
  }

  if (tmp_e) {
    relDist = (rtb_Product2_ki - 30.0) / 15.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] < (1.0 -
      relDist) * 49.5 + relDist * 49.5)) && root_model_B.PWLCheck_m);
  }

  if (rtb_Product2_ki > 45.0) {
    relDist = (rtb_Product2_ki - 30.0) / 15.0;

    /* Checks_PiecewiseLinearBound: '<S2>/PWLCheck' incorporates:
     *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
     */
    root_model_B.PWLCheck_m = ((!(root_model_X.uDOFQuaternion_CSTATE[5] < (1.0 -
      relDist) * 49.5 + relDist * 49.5)) && root_model_B.PWLCheck_m);
  }

  if (tmp) {
    /* Assertion: '<S2>/Assertion' */
    utAssert(root_model_B.PWLCheck_m);
  }

  /* SignalConversion generated from: '<Root>/Vector Concatenate2' incorporates:
   *  Concatenate: '<Root>/Vector Concatenate2'
   *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
   */
  root_model_B.VectorConcatenate2[0] = root_model_X.uDOFQuaternion_CSTATE[3];
  root_model_B.VectorConcatenate2[1] = root_model_X.uDOFQuaternion_CSTATE[4];
  root_model_B.VectorConcatenate2[2] = root_model_X.uDOFQuaternion_CSTATE[5];

  /* SignalConversion generated from: '<Root>/Vector Concatenate2' incorporates:
   *  Concatenate: '<Root>/Vector Concatenate2'
   *  EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)'
   */
  root_model_B.VectorConcatenate2[6] = root_model_X.uDOFQuaternion_CSTATE[9];
  root_model_B.VectorConcatenate2[7] = root_model_X.uDOFQuaternion_CSTATE[10];
  root_model_B.VectorConcatenate2[8] = root_model_X.uDOFQuaternion_CSTATE[11];
  root_model_B.VectorConcatenate2[9] = root_model_X.uDOFQuaternion_CSTATE[12];
  if (tmp) {
    /* Outputs for Enabled SubSystem: '<S3>/Correct1' incorporates:
     *  EnablePort: '<S8>/Enable'
     */
    /* MATLABSystem: '<S8>/MATLAB System' incorporates:
     *  Constant: '<S3>/R1'
     *  DataStoreRead: '<S8>/Data Store ReadP'
     *  DataStoreRead: '<S8>/Data Store ReadX'
     */
    /*  direct output */
    for (ctY = 0; ctY < 17; ctY++) {
      std::memcpy(&imvec[0], &root_model_DW.x[0], 17U * sizeof(real_T));
      rtb_Product_o = std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 *
        std::abs(root_model_DW.x[ctY]));
      imvec[ctY] = root_model_DW.x[ctY] + rtb_Product_o;

      /*  direct output */
      for (i = 0; i <= 10; i += 2) {
        tmp_8 = _mm_loadu_pd(&imvec[i]);
        tmp_7 = _mm_loadu_pd(&root_model_DW.x[i]);
        _mm_storeu_pd(&b_dHdx[i + 13 * ctY], _mm_div_pd(_mm_sub_pd(tmp_8, tmp_7),
          _mm_set1_pd(rtb_Product_o)));
      }

      for (i = 12; i < 13; i++) {
        b_dHdx[i + 13 * ctY] = (imvec[i] - root_model_DW.x[i]) / rtb_Product_o;
      }
    }

    /*  direct output */
    root_model_qrFactor(b_dHdx, root_model_DW.P, rtCP_R1_Value, Sy);
    for (i = 0; i < 17; i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' incorporates:
       *  DataStoreRead: '<S8>/Data Store ReadP'
       */
      std::memset(&A[i * 17], 0, 17U * sizeof(real_T));
      for (ctY = 0; ctY < 17; ctY++) {
        rtb_Product2_ki = root_model_DW.P[17 * ctY + i];
        for (b_tmp_0 = 0; b_tmp_0 <= 14; b_tmp_0 += 2) {
          tmp_8 = _mm_loadu_pd(&root_model_DW.P[17 * ctY + b_tmp_0]);
          A_tmp = 17 * i + b_tmp_0;
          tmp_7 = _mm_loadu_pd(&A[A_tmp]);
          _mm_storeu_pd(&A[A_tmp], _mm_add_pd(_mm_mul_pd(tmp_8, _mm_set1_pd
            (rtb_Product2_ki)), tmp_7));
        }

        for (b_tmp_0 = 16; b_tmp_0 < 17; b_tmp_0++) {
          A_tmp = 17 * i + b_tmp_0;
          A[A_tmp] += root_model_DW.P[17 * ctY + b_tmp_0] * rtb_Product2_ki;
        }
      }
    }

    for (i = 0; i < 13; i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      std::memset(&C[i * 17], 0, 17U * sizeof(real_T));
      for (ctY = 0; ctY < 17; ctY++) {
        rtb_Product2_ki = b_dHdx[13 * ctY + i];
        for (b_tmp_0 = 0; b_tmp_0 <= 14; b_tmp_0 += 2) {
          tmp_8 = _mm_loadu_pd(&A[17 * ctY + b_tmp_0]);
          A_tmp = 17 * i + b_tmp_0;
          tmp_7 = _mm_loadu_pd(&C[A_tmp]);
          _mm_storeu_pd(&C[A_tmp], _mm_add_pd(_mm_mul_pd(tmp_8, _mm_set1_pd
            (rtb_Product2_ki)), tmp_7));
        }

        for (b_tmp_0 = 16; b_tmp_0 < 17; b_tmp_0++) {
          A_tmp = 17 * i + b_tmp_0;
          C[A_tmp] += A[17 * ctY + b_tmp_0] * rtb_Product2_ki;
        }
      }
    }

    for (i = 0; i < 13; i++) {
      for (ctY = 0; ctY < 17; ctY++) {
        /* MATLABSystem: '<S8>/MATLAB System' */
        K[i + 13 * ctY] = C[17 * i + ctY];
      }
    }

    /* Start for MATLABSystem: '<S8>/MATLAB System' */
    root_model_trisolve(Sy, K);

    /* MATLABSystem: '<S8>/MATLAB System' */
    std::memcpy(&C[0], &K[0], 221U * sizeof(real_T));
    for (i = 0; i < 13; i++) {
      for (ctY = 0; ctY < 13; ctY++) {
        /* Start for MATLABSystem: '<S8>/MATLAB System' */
        Sy_0[ctY + 13 * i] = Sy[13 * ctY + i];
      }
    }

    /* Start for MATLABSystem: '<S8>/MATLAB System' */
    root_model_trisolve_l(Sy_0, C);

    /* MATLABSystem: '<S8>/MATLAB System' */
    for (i = 0; i < 13; i++) {
      for (ctY = 0; ctY < 17; ctY++) {
        K[ctY + 17 * i] = C[13 * ctY + i];
      }
    }

    for (i = 0; i <= 218; i += 2) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      tmp_8 = _mm_loadu_pd(&K[i]);
      _mm_storeu_pd(&C[i], _mm_mul_pd(tmp_8, _mm_set1_pd(-1.0)));
    }

    for (i = 220; i < 221; i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' */
      C[i] = -K[i];
    }

    /* MATLABSystem: '<S8>/MATLAB System' */
    for (i = 0; i < 17; i++) {
      std::memset(&A[i * 17], 0, 17U * sizeof(real_T));
      for (ctY = 0; ctY < 13; ctY++) {
        rtb_Product2_ki = b_dHdx[13 * i + ctY];
        for (b_tmp_0 = 0; b_tmp_0 <= 14; b_tmp_0 += 2) {
          tmp_8 = _mm_loadu_pd(&C[17 * ctY + b_tmp_0]);
          A_tmp = 17 * i + b_tmp_0;
          tmp_7 = _mm_loadu_pd(&A[A_tmp]);
          _mm_storeu_pd(&A[A_tmp], _mm_add_pd(_mm_mul_pd(tmp_8, _mm_set1_pd
            (rtb_Product2_ki)), tmp_7));
        }

        for (b_tmp_0 = 16; b_tmp_0 < 17; b_tmp_0++) {
          A_tmp = 17 * i + b_tmp_0;
          A[A_tmp] += C[17 * ctY + b_tmp_0] * rtb_Product2_ki;
        }
      }
    }

    for (i = 0; i < 17; i++) {
      A_tmp = 17 * i + i;
      A[A_tmp]++;
    }

    for (i = 0; i < 13; i++) {
      /* Start for MATLABSystem: '<S8>/MATLAB System' incorporates:
       *  Constant: '<S3>/R1'
       */
      std::memset(&C[i * 17], 0, 17U * sizeof(real_T));
      for (ctY = 0; ctY < 13; ctY++) {
        rtb_Product2_ki = rtCP_R1_Value[13 * i + ctY];
        for (b_tmp_0 = 0; b_tmp_0 <= 14; b_tmp_0 += 2) {
          tmp_8 = _mm_loadu_pd(&K[17 * ctY + b_tmp_0]);
          A_tmp = 17 * i + b_tmp_0;
          tmp_7 = _mm_loadu_pd(&C[A_tmp]);
          _mm_storeu_pd(&C[A_tmp], _mm_add_pd(_mm_mul_pd(tmp_8, _mm_set1_pd
            (rtb_Product2_ki)), tmp_7));
        }

        for (b_tmp_0 = 16; b_tmp_0 < 17; b_tmp_0++) {
          A_tmp = 17 * i + b_tmp_0;
          C[A_tmp] += K[17 * ctY + b_tmp_0] * rtb_Product2_ki;
        }
      }
    }

    /* DataStoreRead: '<S8>/Data Store ReadP' */
    std::memcpy(&dFdx[0], &root_model_DW.P[0], 289U * sizeof(real_T));

    /* Start for MATLABSystem: '<S8>/MATLAB System' incorporates:
     *  DataStoreRead: '<S8>/Data Store ReadX'
     *  DataStoreWrite: '<S8>/Data Store WriteP'
     *  ZeroOrderHold: '<Root>/Zero-Order Hold'
     */
    root_model_qrFactor_l(A, dFdx, C, root_model_DW.P);
    for (i = 0; i <= 10; i += 2) {
      tmp_8 = _mm_loadu_pd(&root_model_B.VectorConcatenate2[i]);
      tmp_7 = _mm_loadu_pd(&root_model_DW.x[i]);
      _mm_storeu_pd(&tmp_2[i], _mm_sub_pd(tmp_8, tmp_7));
    }

    for (i = 12; i < 13; i++) {
      tmp_2[i] = root_model_B.VectorConcatenate2[i] - root_model_DW.x[i];
    }

    /* DataStoreWrite: '<S8>/Data Store WriteX' incorporates:
     *  DataStoreRead: '<S8>/Data Store ReadX'
     *  MATLABSystem: '<S8>/MATLAB System'
     * */
    for (i = 0; i < 17; i++) {
      rtb_Product2_ki = 0.0;
      for (ctY = 0; ctY < 13; ctY++) {
        rtb_Product2_ki += K[17 * ctY + i] * tmp_2[ctY];
      }

      root_model_DW.x[i] += rtb_Product2_ki;
    }

    /* End of DataStoreWrite: '<S8>/Data Store WriteX' */
    /* End of Outputs for SubSystem: '<S3>/Correct1' */

    /* UnitDelay: '<Root>/Unit Delay' */
    root_model_B.expl_temp.lastMV[0] = root_model_DW.UnitDelay_DSTATE[0];
    root_model_B.expl_temp.lastMV[1] = root_model_DW.UnitDelay_DSTATE[1];
    root_model_B.expl_temp.lastMV[2] = root_model_DW.UnitDelay_DSTATE[2];

    /* MATLAB Function: '<S12>/NLMPC' incorporates:
     *  BusCreator: '<Root>/Bus Creator'
     *  Constant: '<Root>/Constant11'
     *  Constant: '<Root>/Constant13'
     *  Constant: '<Root>/Constant14'
     *  Constant: '<Root>/Constant2'
     *  Constant: '<Root>/Constant3'
     *  Constant: '<Root>/Constant4'
     *  Constant: '<Root>/Constant5'
     *  Constant: '<Root>/Constant6'
     *  Constant: '<Root>/Constant7'
     *  Constant: '<Root>/Constant8'
     *  Constant: '<Root>/Zero'
     *  DataStoreRead: '<S9>/Data Store Read'
     *  Math: '<Root>/Transpose'
     *  UnitDelay: '<Root>/Unit Delay'
     */
    for (i = 0; i < 13; i++) {
      root_model_B.expl_temp.ref[10 * i] = root_model_ConstB.refrowvecinput[i];
      root_model_B.expl_temp.OutputWeights[10 * i] = rtCP_Constant2_Value[i];
      for (ctY = 0; ctY < 9; ctY++) {
        b_tmp_0 = (10 * i + ctY) + 1;
        root_model_B.expl_temp.ref[b_tmp_0] = root_model_ConstB.refrowvecinput[i];
        root_model_B.expl_temp.OutputWeights[b_tmp_0] = rtCP_Constant2_Value[i];
      }
    }

    root_model_B.expl_temp.MVWeights[0] = 0.064460336253326159;
    root_model_B.expl_temp.MVWeights[10] = 0.099999999999889136;
    root_model_B.expl_temp.MVWeights[20] = 0.0999999999981664;
    root_model_B.expl_temp.MVRateWeights[0] = 0.10010613758435991;
    root_model_B.expl_temp.MVRateWeights[10] = 0.49999999998360573;
    root_model_B.expl_temp.MVRateWeights[20] = 0.500000000017081;
    for (i = 0; i < 9; i++) {
      root_model_B.expl_temp.MVWeights[i + 1] = 0.064460336253326159;
      root_model_B.expl_temp.MVWeights[i + 11] = 0.099999999999889136;
      root_model_B.expl_temp.MVWeights[i + 21] = 0.0999999999981664;
      root_model_B.expl_temp.MVRateWeights[i + 1] = 0.10010613758435991;
      root_model_B.expl_temp.MVRateWeights[i + 11] = 0.49999999998360573;
      root_model_B.expl_temp.MVRateWeights[i + 21] = 0.500000000017081;
    }

    for (i = 0; i < 30; i++) {
      tmp_0[i] = (rtMinusInf);
      tmp_1[i] = (rtInf);
    }

    root_model_getUBounds(root_model_B.expl_temp.lastMV, c, d, tmp_0, tmp_1,
                          root_model_B.A_data, A_size, B_data, B_size);
    root_model_B.expl_temp.Parameters.f1 = 0.0005967898212;
    root_model_B.expl_temp.Parameters.f2 = 1.206;
    root_model_B.expl_temp.Parameters.f3 = 9.81;
    std::memcpy(&root_model_B.expl_temp.Parameters.f4[0], &rtCP_Constant7_Value
                [0], 9U * sizeof(real_T));
    root_model_B.expl_temp.Parameters.f5 = 0.242;
    for (i = 0; i < 30; i++) {
      root_model_B.expl_temp.MVScaledTarget[i] = 0.0;
      root_model_B.expl_temp.MVRateMax[i] = (rtInf);
      root_model_B.expl_temp.MVRateMin[i] = (rtMinusInf);
      root_model_B.expl_temp.MVMax[i] = d[i];
      root_model_B.expl_temp.MVMin[i] = c[i];
    }

    for (i = 0; i < 170; i++) {
      root_model_B.expl_temp.StateMax[i] = (rtInf);
      root_model_B.expl_temp.StateMin[i] = (rtMinusInf);
    }

    for (i = 0; i < 130; i++) {
      root_model_B.expl_temp.OutputMax[i] = (rtInf);
      root_model_B.expl_temp.OutputMin[i] = (rtMinusInf);
    }

    root_model_B.expl_temp.ECRWeight = 100000.0;

    /* Outputs for Atomic SubSystem: '<S3>/Output' */
    std::memcpy(&root_model_B.expl_temp.x[0], &root_model_DW.x[0], 17U * sizeof
                (real_T));

    /* End of Outputs for SubSystem: '<S3>/Output' */
    expl_temp.PassivityUsePredictedX = true;
    expl_temp.OutputPassivityIndex = 0.1;
    expl_temp.InputPassivityIndex = 0.0;
    expl_temp.MVIndex[0] = 1.0;
    expl_temp.MVIndex[1] = 2.0;
    expl_temp.MVIndex[2] = 3.0;
    expl_temp.NumOfInputs = 3.0;
    expl_temp.NumOfOutputs = 13.0;
    expl_temp.NumOfStates = 17.0;
    expl_temp.PredictionHorizon = 10.0;
    std::memset(&expl_temp.MVTarget[0], 0, 30U * sizeof(real_T));
    std::memcpy(&expl_temp.References[0], &root_model_B.expl_temp.ref[0], 130U *
                sizeof(real_T));
    expl_temp.LastMV[0] = root_model_B.expl_temp.lastMV[0];
    expl_temp.LastMV[1] = root_model_B.expl_temp.lastMV[1];
    expl_temp.LastMV[2] = root_model_B.expl_temp.lastMV[2];

    /* Outputs for Atomic SubSystem: '<S3>/Output' */
    std::memcpy(&expl_temp.CurrentStates[0], &root_model_DW.x[0], 17U * sizeof
                (real_T));

    /* End of Outputs for SubSystem: '<S3>/Output' */
    expl_temp.Ts = 1.0;
    root_model_B.expl_temp_g.Parameters.f1 = 0.0005967898212;
    root_model_B.expl_temp_g.Parameters.f2 = 1.206;
    root_model_B.expl_temp_g.Parameters.f3 = 9.81;
    std::memcpy(&root_model_B.expl_temp_g.Parameters.f4[0],
                &rtCP_Constant7_Value[0], 9U * sizeof(real_T));
    root_model_B.expl_temp_g.Parameters.f5 = 0.242;
    for (i = 0; i < 30; i++) {
      root_model_B.expl_temp_g.MVScaledTarget[i] = 0.0;
      root_model_B.expl_temp_g.MVRateMax[i] = (rtInf);
      root_model_B.expl_temp_g.MVRateMin[i] = (rtMinusInf);
      root_model_B.expl_temp_g.MVMax[i] = d[i];
      root_model_B.expl_temp_g.MVMin[i] = c[i];
    }

    for (i = 0; i < 170; i++) {
      root_model_B.expl_temp_g.StateMax[i] = (rtInf);
      root_model_B.expl_temp_g.StateMin[i] = (rtMinusInf);
    }

    for (i = 0; i < 130; i++) {
      root_model_B.expl_temp_g.OutputMax[i] = (rtInf);
      root_model_B.expl_temp_g.OutputMin[i] = (rtMinusInf);
    }

    root_model_B.expl_temp_g.ECRWeight = 100000.0;
    std::memcpy(&root_model_B.expl_temp_g.MVRateWeights[0],
                &root_model_B.expl_temp.MVRateWeights[0], 30U * sizeof(real_T));
    std::memcpy(&root_model_B.expl_temp_g.MVWeights[0],
                &root_model_B.expl_temp.MVWeights[0], 30U * sizeof(real_T));
    std::memcpy(&root_model_B.expl_temp_g.OutputWeights[0],
                &root_model_B.expl_temp.OutputWeights[0], 130U * sizeof(real_T));
    std::memcpy(&root_model_B.expl_temp_g.ref[0], &root_model_B.expl_temp.ref[0],
                130U * sizeof(real_T));
    root_model_B.expl_temp_g.lastMV[0] = root_model_B.expl_temp.lastMV[0];
    root_model_B.expl_temp_g.lastMV[1] = root_model_B.expl_temp.lastMV[1];
    root_model_B.expl_temp_g.lastMV[2] = root_model_B.expl_temp.lastMV[2];

    /* Outputs for Atomic SubSystem: '<S3>/Output' */
    std::memcpy(&root_model_B.expl_temp_g.x[0], &root_model_DW.x[0], 17U *
                sizeof(real_T));

    /* End of Outputs for SubSystem: '<S3>/Output' */
    std::memcpy(&tmp_3[0], &rtCP_Constant14_Value[0], 17U * sizeof(real_T));
    tmp_0[0] = 140.79831387973042;
    tmp_0[1] = 0.0;
    tmp_0[2] = 0.0;
    for (i = 0; i < 9; i++) {
      std::memcpy(&tmp_3[i * 17 + 17], &rtCP_Constant14_Value[0], 17U * sizeof
                  (real_T));
      ctY = (i + 1) * 3;
      tmp_0[ctY] = 140.79831387973042;
      tmp_0[ctY + 1] = 0.0;
      tmp_0[ctY + 2] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      a[i] = 0.0;
    }

    for (i = 0; i < 30; i++) {
      rtb_Product2_ki = tmp_0[i];
      for (ctY = 0; ctY < 6; ctY++) {
        a[ctY] += static_cast<real_T>(a_0[6 * i + ctY]) * rtb_Product2_ki;
      }
    }

    std::memcpy(&tmp_4[0], &tmp_3[0], 170U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      tmp_4[i + 170] = a[i];
    }

    tmp_4[176] = 0.0;
    for (i = 0; i < 170; i++) {
      tmp_5[i] = (rtMinusInf);
    }

    for (i = 0; i < 6; i++) {
      tmp_5[i + 170] = (rtMinusInf);
    }

    tmp_5[176] = 0.0;
    root_model_fmincon(&root_model_B.expl_temp, &expl_temp, tmp_4,
                       root_model_B.A_data, B_data, B_size, tmp_5,
                       &root_model_B.expl_temp_g, z, &rtb_Product2_ki,
                       &rtb_Product_o, &Out);
    if ((rtb_Product_o == 0.0) && (Out.constrviolation > 1.0E-6)) {
      rtb_Product_o = -2.0;
    }

    /* Outputs for Atomic SubSystem: '<S3>/Output' */
    getXUe_FXX9Wcuz(z, root_model_DW.x, X, U, &rtb_Product2_ki);

    /* End of Outputs for SubSystem: '<S3>/Output' */
    if (rtb_Product_o > 0.0) {
      root_model_B.mv[0] = U[0];
      root_model_B.mv[1] = U[11];
      root_model_B.mv[2] = U[22];
    } else {
      root_model_B.mv[0] = root_model_B.expl_temp.lastMV[0];
      root_model_B.mv[1] = root_model_B.expl_temp.lastMV[1];
      root_model_B.mv[2] = root_model_B.expl_temp.lastMV[2];
    }

    /* End of MATLAB Function: '<S12>/NLMPC' */

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' */
    root_model_B.VectorConcatenate1[0] = root_model_B.mv[0];
    root_model_B.VectorConcatenate1[1] = root_model_B.mv[1];
    root_model_B.VectorConcatenate1[2] = root_model_B.mv[2];

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
     *  Constant: '<Root>/Constant16'
     */
    root_model_B.VectorConcatenate1[3] = 0.0005967898212;

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
     *  Constant: '<Root>/Constant15'
     */
    root_model_B.VectorConcatenate1[4] = 1.206;

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
     *  Constant: '<Root>/Constant17'
     */
    root_model_B.VectorConcatenate1[5] = 9.81;

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
     *  Reshape: '<Root>/Reshape'
     */
    std::memcpy(&root_model_B.VectorConcatenate1[6], &root_model_ConstB.Reshape
                [0], 9U * sizeof(real_T));

    /* SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
     *  Constant: '<Root>/Constant19'
     */
    root_model_B.VectorConcatenate1[15] = 0.242;

    /* Outputs for Atomic SubSystem: '<S3>/Predict' */
    /* DataStoreRead: '<S10>/Data Store ReadX' */
    std::memcpy(&rtb_DataStoreReadX[0], &root_model_DW.x[0], 17U * sizeof(real_T));

    /* DataStoreRead: '<S10>/Data Store ReadP' */
    std::memcpy(&A[0], &root_model_DW.P[0], 289U * sizeof(real_T));

    /* Start for MATLABSystem: '<S10>/MATLAB System' */
    ekf_t_func(rtb_DataStoreReadX, z_0);

    /* MATLABSystem: '<S10>/MATLAB System' */
    for (ctY = 0; ctY < 17; ctY++) {
      std::memcpy(&imvec[0], &rtb_DataStoreReadX[0], 17U * sizeof(real_T));

      /* Start for MATLABSystem: '<S10>/MATLAB System' */
      rtb_Product2_ki = rtb_DataStoreReadX[ctY];
      rtb_Product_o = std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 *
        std::abs(rtb_Product2_ki));
      imvec[ctY] = rtb_Product2_ki + rtb_Product_o;

      /* Start for MATLABSystem: '<S10>/MATLAB System' */
      ekf_t_func(imvec, imz);
      for (i = 0; i <= 14; i += 2) {
        /* Start for MATLABSystem: '<S10>/MATLAB System' */
        tmp_8 = _mm_loadu_pd(&imz[i]);
        tmp_7 = _mm_loadu_pd(&z_0[i]);
        _mm_storeu_pd(&dFdx[i + 17 * ctY], _mm_div_pd(_mm_sub_pd(tmp_8, tmp_7),
          _mm_set1_pd(rtb_Product_o)));
      }

      for (i = 16; i < 17; i++) {
        dFdx[i + 17 * ctY] = (imz[i] - z_0[i]) / rtb_Product_o;
      }
    }

    /* Start for MATLABSystem: '<S10>/MATLAB System' incorporates:
     *  Constant: '<S3>/Q'
     *  DataStoreWrite: '<S10>/Data Store WriteP'
     */
    ekf_t_func(rtb_DataStoreReadX, rtb_MATLABSystem_o1_0);
    root_model_qrFactor_lx(dFdx, A, rtCP_Q_Value, root_model_DW.P);

    /* DataStoreWrite: '<S10>/Data Store WriteX' incorporates:
     *  MATLABSystem: '<S10>/MATLAB System'
     */
    std::memcpy(&root_model_DW.x[0], &rtb_MATLABSystem_o1_0[0], 17U * sizeof
                (real_T));

    /* End of Outputs for SubSystem: '<S3>/Predict' */

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Constant: '<Root>/Constant1'
     */
    rtb_Product_o = root_model_B.mv[0] * root_model_B.mv[0] * 0.0005967898212;
    rtb_Product2_ki = 57.295779513082323 * root_model_B.mv[1];
    relDist = rtb_Product2_ki;
    sind_CjVy6Jw4(&relDist);
    cosd_ZK11uGDT(&rtb_Product2_ki);
    rtb_Product8 = 57.295779513082323 * root_model_B.mv[2];
    b_tmp = rtb_Product8;
    sind_CjVy6Jw4(&b_tmp);
    cosd_ZK11uGDT(&rtb_Product8);
    b[1] = 0.0;
    b[4] = rtb_Product2_ki;
    b[7] = -relDist;
    b[2] = 0.0;
    b[5] = relDist;
    b[8] = rtb_Product2_ki;
    rtb_Product8_0[0] = rtb_Product8;
    rtb_Product8_0[3] = 0.0;
    rtb_Product8_0[6] = b_tmp;
    rtb_Product8_0[2] = -b_tmp;
    rtb_Product8_0[5] = 0.0;
    rtb_Product8_0[8] = rtb_Product8;
    for (i = 0; i < 3; i++) {
      b[3 * i] = b_2[i];
      ctY = 3 * i + 1;
      rtb_Product8_0[ctY] = c_0[i];
      b_0[3 * i] = 0.0;
      b_0[ctY] = 0.0;
      b_0[3 * i + 2] = 0.0;
    }

    for (i = 0; i < 3; i++) {
      relDist = b_0[3 * i];
      b_tmp_0 = 3 * i + 1;
      rtb_Product8 = b_0[b_tmp_0];
      A_tmp = 3 * i + 2;
      b_tmp = b_0[A_tmp];
      for (ctY = 0; ctY < 3; ctY++) {
        rtb_Product2_ki = rtb_Product8_0[3 * i + ctY];
        tmp_8 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&b[3 * ctY]), _mm_set1_pd
          (rtb_Product2_ki)), _mm_set_pd(rtb_Product8, relDist));
        _mm_storeu_pd(&tmp_6[0], tmp_8);
        relDist = tmp_6[0];
        rtb_Product8 = tmp_6[1];
        b_tmp += b[3 * ctY + 2] * rtb_Product2_ki;
      }

      b_0[A_tmp] = b_tmp;
      b_0[b_tmp_0] = rtb_Product8;
      b_0[3 * i] = relDist;
    }

    relDist = 0.0;
    rtb_Product8 = 0.0;
    b_tmp = 0.0;
    for (i = 0; i < 3; i++) {
      rtb_Product2_ki = b_b[i];
      tmp_8 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&b_0[3 * i]), _mm_set1_pd
        (rtb_Product2_ki)), _mm_set_pd(rtb_Product8, relDist));
      _mm_storeu_pd(&tmp_6[0], tmp_8);
      relDist = tmp_6[0];
      rtb_Product8 = tmp_6[1];
      b_tmp += b_0[3 * i + 2] * rtb_Product2_ki;
    }

    b_1[2] = b_tmp;
    b_1[1] = rtb_Product8;
    b_1[0] = relDist;
    tmp_8 = _mm_mul_pd(_mm_set1_pd(rtb_Product_o), _mm_loadu_pd(&b_1[0]));
    _mm_storeu_pd(&root_model_B.prop_force[0], tmp_8);
    root_model_B.prop_force[2] = rtb_Product_o * b_tmp;
    _mm_storeu_pd(&root_model_B.prop_torque[0], _mm_sub_pd(_mm_mul_pd(_mm_set_pd
      (root_model_B.prop_force[0], 0.0), _mm_set_pd(0.242,
      root_model_B.prop_force[2])), _mm_mul_pd(_mm_set_pd(0.0,
      root_model_B.prop_force[1]), _mm_set_pd(root_model_B.prop_force[2], 0.242))));
    root_model_B.prop_torque[2] = 0.0 * root_model_B.prop_force[1] -
      root_model_B.prop_force[0] * 0.0;

    /* End of MATLAB Function: '<Root>/MATLAB Function' */
  }

  /* Sqrt: '<S38>/sqrt' incorporates:
   *  Product: '<S39>/Product'
   *  Product: '<S39>/Product1'
   *  Product: '<S39>/Product2'
   *  Product: '<S39>/Product3'
   *  Sum: '<S39>/Sum'
   */
  rtb_Product_o = std::sqrt(((rtb_prevbodyquaternion_idx_0 *
    rtb_prevbodyquaternion_idx_0 + rtb_prevbodyquaternion_idx_1 *
    rtb_prevbodyquaternion_idx_1) + rtb_prevbodyquaternion_idx_2 *
    rtb_prevbodyquaternion_idx_2) + rtb_prevbodyquaternion_idx_3 *
    rtb_prevbodyquaternion_idx_3);

  /* Product: '<S34>/Product2' */
  rtb_Product2_ki = rtb_prevbodyquaternion_idx_2 / rtb_Product_o;

  /* Product: '<S34>/Product3' */
  relDist = rtb_prevbodyquaternion_idx_3 / rtb_Product_o;

  /* Product: '<S34>/Product1' */
  rtb_Product8 = rtb_prevbodyquaternion_idx_1 / rtb_Product_o;

  /* Product: '<S34>/Product' */
  rtb_Product_o = rtb_prevbodyquaternion_idx_0 / rtb_Product_o;

  /* Product: '<S36>/Product6' incorporates:
   *  Product: '<S35>/Product6'
   */
  tmp_8 = _mm_set_pd(rtb_Product8, rtb_Product2_ki);

  /* Product: '<S35>/Product7' */
  tmp_7 = _mm_set1_pd(relDist);

  /* Gain: '<S35>/Gain2' */
  tmp_9 = _mm_set1_pd(2.0);

  /* Product: '<S35>/Product8' incorporates:
   *  Constant: '<Root>/ext_forces'
   */
  tmp_a = _mm_set1_pd(0.0);

  /* Sum: '<S35>/Sum3' incorporates:
   *  Constant: '<Root>/ext_forces'
   *  Constant: '<S35>/Constant'
   *  Product: '<S35>/Product'
   *  Product: '<S35>/Product1'
   *  Product: '<S35>/Product2'
   *  Product: '<S35>/Product3'
   *  Product: '<S35>/Product5'
   *  Product: '<S36>/Product2'
   *  Product: '<S36>/Product3'
   *  Sum: '<Root>/Add'
   */
  tmp_8 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd(_mm_sub_pd
    (_mm_sub_pd(_mm_set1_pd(0.5), _mm_mul_pd(tmp_8, tmp_8)), _mm_mul_pd(tmp_7,
    tmp_7)), tmp_9), tmp_a), _mm_mul_pd(_mm_mul_pd(_mm_add_pd(_mm_mul_pd
    (_mm_set1_pd(rtb_Product8), _mm_set1_pd(rtb_Product2_ki)), _mm_mul_pd
    (_mm_mul_pd(_mm_set1_pd(rtb_Product_o), tmp_7), tmp_c)), tmp_9), tmp_a)),
    _mm_mul_pd(_mm_mul_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(rtb_Product_o,
    rtb_Product8), _mm_set_pd(rtb_Product8, relDist)), _mm_mul_pd(_mm_mul_pd
    (_mm_set_pd(rtb_Product2_ki, rtb_Product_o), _mm_set_pd(relDist,
    rtb_Product2_ki)), tmp_b)), tmp_9), _mm_set1_pd(-11.83086))), _mm_loadu_pd
                     (&root_model_B.prop_force[0]));

  /* Sum: '<Root>/Add' incorporates:
   *  Constant: '<Root>/ext_forces'
   *  Constant: '<S37>/Constant'
   *  Gain: '<S37>/Gain'
   *  Gain: '<S37>/Gain1'
   *  Gain: '<S37>/Gain2'
   *  Product: '<S37>/Product'
   *  Product: '<S37>/Product1'
   *  Product: '<S37>/Product2'
   *  Product: '<S37>/Product3'
   *  Product: '<S37>/Product4'
   *  Product: '<S37>/Product5'
   *  Product: '<S37>/Product6'
   *  Product: '<S37>/Product7'
   *  Product: '<S37>/Product8'
   *  Sum: '<S37>/Sum'
   *  Sum: '<S37>/Sum1'
   *  Sum: '<S37>/Sum2'
   *  Sum: '<S37>/Sum3'
   */
  _mm_storeu_pd(&root_model_B.Add[0], tmp_8);
  root_model_B.Add[2] = (((rtb_Product8 * relDist + rtb_Product_o *
    rtb_Product2_ki) * 2.0 * 0.0 + (rtb_Product2_ki * relDist - rtb_Product_o *
    rtb_Product8) * 2.0 * 0.0) + ((0.5 - rtb_Product8 * rtb_Product8) -
    rtb_Product2_ki * rtb_Product2_ki) * 2.0 * -11.83086) +
    root_model_B.prop_force[2];
  if (rtmIsMajorTimeStep(root_model_M)) {
    if (rtmIsMajorTimeStep(root_model_M)) {
      /* Update for UnitDelay: '<Root>/Unit Delay' */
      root_model_DW.UnitDelay_DSTATE[0] = root_model_B.mv[0];
      root_model_DW.UnitDelay_DSTATE[1] = root_model_B.mv[1];
      root_model_DW.UnitDelay_DSTATE[2] = root_model_B.mv[2];
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(root_model_M)) {
    rt_ertODEUpdateContinuousStates(&root_model_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++root_model_M->Timing.clockTick0;
    root_model_M->Timing.t[0] = rtsiGetSolverStopTime(&root_model_M->solverInfo);

    {
      /* Update absolute timer for sample time: [1.0s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 1.0, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      root_model_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void root_model_derivatives(void)
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  XDot_root_model_T *_rtXdot;
  real_T tmp_1[2];
  real_T pData_idx_0;
  real_T pData_idx_1;
  real_T pData_idx_2;
  _rtXdot = ((XDot_root_model_T *) root_model_M->derivs);

  /* Derivatives for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  std::memset(&_rtXdot->uDOFQuaternion_CSTATE[0], 0, 13U * sizeof(real_T));
  tmp_7 = _mm_set_pd(root_model_X.uDOFQuaternion_CSTATE[6],
                     root_model_X.uDOFQuaternion_CSTATE[8]);
  tmp = _mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&root_model_B.Add[0]), _mm_set1_pd
    (1.206)), _mm_sub_pd(_mm_mul_pd(_mm_set_pd
    (root_model_X.uDOFQuaternion_CSTATE[0], root_model_X.uDOFQuaternion_CSTATE[2]),
    _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[7])), _mm_mul_pd
    (_mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[1]), tmp_7)));
  _mm_storeu_pd(&_rtXdot->uDOFQuaternion_CSTATE[0], tmp);
  _rtXdot->uDOFQuaternion_CSTATE[2] = root_model_B.Add[2] / 1.206 -
    (root_model_X.uDOFQuaternion_CSTATE[1] * root_model_X.uDOFQuaternion_CSTATE
     [6] - root_model_X.uDOFQuaternion_CSTATE[0] *
     root_model_X.uDOFQuaternion_CSTATE[7]);
  tmp = _mm_div_pd(_mm_set1_pd(1.0), _mm_set1_pd(std::sqrt
    (((root_model_X.uDOFQuaternion_CSTATE[9] *
       root_model_X.uDOFQuaternion_CSTATE[9] +
       root_model_X.uDOFQuaternion_CSTATE[10] *
       root_model_X.uDOFQuaternion_CSTATE[10]) +
      root_model_X.uDOFQuaternion_CSTATE[11] *
      root_model_X.uDOFQuaternion_CSTATE[11]) +
     root_model_X.uDOFQuaternion_CSTATE[12] *
     root_model_X.uDOFQuaternion_CSTATE[12])));
  tmp_0 = _mm_mul_pd(tmp, _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[9]));
  _mm_storeu_pd(&tmp_1[0], tmp_0);

  /* Derivatives for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  pData_idx_0 = tmp_1[0];
  pData_idx_1 = tmp_1[1];
  tmp = _mm_mul_pd(tmp, _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[11]));
  _mm_storeu_pd(&tmp_1[0], tmp);

  /* Derivatives for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  tmp = _mm_set1_pd(pData_idx_0);
  tmp_0 = _mm_set1_pd(pData_idx_1);
  tmp_2 = _mm_set1_pd(tmp_1[0]);
  tmp_3 = _mm_set1_pd(tmp_1[1]);
  tmp_4 = _mm_set_pd(1.0, -1.0);
  tmp_5 = _mm_set_pd(-1.0, 1.0);

  /* Derivatives for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  tmp_6 = _mm_set1_pd(2.0);
  tmp = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_add_pd(_mm_add_pd
    (_mm_mul_pd(tmp, tmp), _mm_mul_pd(_mm_mul_pd(tmp_0, tmp_0), tmp_5)),
    _mm_mul_pd(_mm_mul_pd(tmp_2, tmp_2), tmp_4)), _mm_mul_pd(tmp_3, tmp_3)),
    _mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[0])), _mm_mul_pd(_mm_mul_pd
    (_mm_add_pd(_mm_mul_pd(tmp_0, tmp_2), _mm_mul_pd(_mm_mul_pd(tmp, tmp_3),
    tmp_4)), tmp_6), _mm_set_pd(root_model_X.uDOFQuaternion_CSTATE[0],
    root_model_X.uDOFQuaternion_CSTATE[1]))), _mm_mul_pd(_mm_mul_pd(_mm_add_pd
    (_mm_mul_pd(_mm_set_pd(tmp_1[0], pData_idx_1), tmp_3), _mm_mul_pd(_mm_mul_pd
    (tmp, _mm_set_pd(pData_idx_1, tmp_1[0])), tmp_5)), tmp_6), _mm_set1_pd
    (root_model_X.uDOFQuaternion_CSTATE[2])));
  _mm_storeu_pd(&_rtXdot->uDOFQuaternion_CSTATE[3], tmp);
  _rtXdot->uDOFQuaternion_CSTATE[5] = ((pData_idx_1 * tmp_1[1] - pData_idx_0 *
    tmp_1[0]) * 2.0 * root_model_X.uDOFQuaternion_CSTATE[0] + (tmp_1[0] * tmp_1
    [1] + pData_idx_0 * pData_idx_1) * 2.0 * root_model_X.uDOFQuaternion_CSTATE
    [1]) + (((pData_idx_0 * pData_idx_0 - pData_idx_1 * pData_idx_1) - tmp_1[0] *
             tmp_1[0]) + tmp_1[1] * tmp_1[1]) *
    root_model_X.uDOFQuaternion_CSTATE[2];
  tmp = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[6]);
  tmp_0 = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[7]);
  tmp_2 = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[8]);
  _mm_storeu_pd(&tmp_1[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(0.0,
    0.01938645), tmp), _mm_mul_pd(_mm_set_pd(0.01938645, 0.0), tmp_0)),
    _mm_mul_pd(_mm_set1_pd(0.0), tmp_2)));
  pData_idx_0 = tmp_1[0];
  pData_idx_1 = tmp_1[1];
  pData_idx_2 = (0.0 * root_model_X.uDOFQuaternion_CSTATE[6] + 0.0 *
                 root_model_X.uDOFQuaternion_CSTATE[7]) + 0.000738675 *
    root_model_X.uDOFQuaternion_CSTATE[8];
  tmp_3 = _mm_set1_pd(0.0);
  tmp_7 = _mm_sub_pd(_mm_sub_pd(_mm_loadu_pd(&root_model_B.prop_torque[0]),
    _mm_sub_pd(_mm_mul_pd(_mm_loadu_pd(&root_model_X.uDOFQuaternion_CSTATE[7]),
    _mm_set_pd(tmp_1[0], pData_idx_2)), _mm_mul_pd(tmp_7, _mm_set_pd(pData_idx_2,
    tmp_1[1])))), _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_3, tmp), _mm_mul_pd(tmp_3,
    tmp_0)), _mm_mul_pd(tmp_3, tmp_2)));
  _mm_storeu_pd(&tmp_1[0], tmp_7);

  /* Derivatives for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  pData_idx_2 = (root_model_B.prop_torque[2] -
                 (root_model_X.uDOFQuaternion_CSTATE[6] * pData_idx_1 -
                  root_model_X.uDOFQuaternion_CSTATE[7] * pData_idx_0)) - ((0.0 *
    root_model_X.uDOFQuaternion_CSTATE[6] + 0.0 *
    root_model_X.uDOFQuaternion_CSTATE[7]) + 0.0 *
    root_model_X.uDOFQuaternion_CSTATE[8]);
  pData_idx_0 = -0.0 * tmp_1[1];
  _rtXdot->uDOFQuaternion_CSTATE[6] = (51.582419679724758 * tmp_1[0] +
    pData_idx_0) + 0.0 * pData_idx_2;
  _rtXdot->uDOFQuaternion_CSTATE[7] = (-0.0 * tmp_1[0] + 51.582419679724758 *
    tmp_1[1]) + -0.0 * pData_idx_2;
  _rtXdot->uDOFQuaternion_CSTATE[8] = (0.0 * tmp_1[0] + pData_idx_0) +
    1353.7753409821639 * pData_idx_2;
  tmp_7 = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[9]);
  tmp = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[10]);
  tmp_0 = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[11]);
  tmp_2 = _mm_set1_pd(root_model_X.uDOFQuaternion_CSTATE[12]);
  tmp_3 = _mm_set1_pd(0.5);
  _mm_storeu_pd(&_rtXdot->uDOFQuaternion_CSTATE[9], _mm_mul_pd(_mm_add_pd
    (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
    (root_model_X.uDOFQuaternion_CSTATE[6], 0.0), tmp_7), _mm_mul_pd(_mm_set_pd
    (0.0, -root_model_X.uDOFQuaternion_CSTATE[6]), tmp)), _mm_mul_pd(_mm_set_pd
    (root_model_X.uDOFQuaternion_CSTATE[8], -root_model_X.uDOFQuaternion_CSTATE
     [7]), tmp_0)), _mm_mul_pd(_mm_set_pd(-root_model_X.uDOFQuaternion_CSTATE[7],
    -root_model_X.uDOFQuaternion_CSTATE[8]), tmp_2)), tmp_3));
  tmp_7 = _mm_mul_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
    (&root_model_X.uDOFQuaternion_CSTATE[7]), tmp_7), _mm_mul_pd(_mm_set_pd
    (root_model_X.uDOFQuaternion_CSTATE[7], -root_model_X.uDOFQuaternion_CSTATE
     [8]), tmp)), _mm_mul_pd(_mm_set_pd(-root_model_X.uDOFQuaternion_CSTATE[6],
    0.0), tmp_0)), _mm_mul_pd(_mm_set_pd(0.0,
    root_model_X.uDOFQuaternion_CSTATE[6]), tmp_2)), tmp_3);
  _mm_storeu_pd(&_rtXdot->uDOFQuaternion_CSTATE[11], tmp_7);
}

/* Model initialize function */
void root_model_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&root_model_M->solverInfo,
                          &root_model_M->Timing.simTimeStep);
    rtsiSetTPtr(&root_model_M->solverInfo, &rtmGetTPtr(root_model_M));
    rtsiSetStepSizePtr(&root_model_M->solverInfo,
                       &root_model_M->Timing.stepSize0);
    rtsiSetdXPtr(&root_model_M->solverInfo, &root_model_M->derivs);
    rtsiSetContStatesPtr(&root_model_M->solverInfo, (real_T **)
                         &root_model_M->contStates);
    rtsiSetNumContStatesPtr(&root_model_M->solverInfo,
      &root_model_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&root_model_M->solverInfo,
      &root_model_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&root_model_M->solverInfo,
      &root_model_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&root_model_M->solverInfo,
      &root_model_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&root_model_M->solverInfo, (boolean_T**)
      &root_model_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&root_model_M->solverInfo, (&rtmGetErrorStatus
      (root_model_M)));
    rtsiSetRTModelPtr(&root_model_M->solverInfo, root_model_M);
  }

  rtsiSetSimTimeStep(&root_model_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&root_model_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&root_model_M->solverInfo, false);
  root_model_M->intgData.y = root_model_M->odeY;
  root_model_M->intgData.f[0] = root_model_M->odeF[0];
  root_model_M->intgData.f[1] = root_model_M->odeF[1];
  root_model_M->intgData.f[2] = root_model_M->odeF[2];
  root_model_M->contStates = ((X_root_model_T *) &root_model_X);
  root_model_M->contStateDisabled = ((XDis_root_model_T *) &root_model_XDis);
  root_model_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&root_model_M->solverInfo, static_cast<void *>
                    (&root_model_M->intgData));
  rtsiSetSolverName(&root_model_M->solverInfo,"ode3");
  rtmSetTPtr(root_model_M, &root_model_M->Timing.tArray[0]);
  root_model_M->Timing.stepSize0 = 1.0;

  /* block I/O */
  (void) std::memset((static_cast<void *>(&root_model_B)), 0,
                     sizeof(B_root_model_T));

  /* states (continuous) */
  {
    (void) std::memset(static_cast<void *>(&root_model_X), 0,
                       sizeof(X_root_model_T));
  }

  /* disabled states */
  {
    (void) std::memset(static_cast<void *>(&root_model_XDis), 0,
                       sizeof(XDis_root_model_T));
  }

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&root_model_DW), 0,
                     sizeof(DW_root_model_T));

  /* Start for DataStoreMemory: '<S3>/DataStoreMemory - P' */
  std::memcpy(&root_model_DW.P[0], &rtCP_DataStoreMemoryP_InitialVa[0], 289U *
              sizeof(real_T));

  /* Start for DataStoreMemory: '<S3>/DataStoreMemory - x' */
  std::memset(&root_model_DW.x[0], 0, 17U * sizeof(real_T));

  /* InitializeConditions for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  root_model_X.uDOFQuaternion_CSTATE[0] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[3] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[6] = 0.0;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  root_model_DW.UnitDelay_DSTATE[0] = 0.0;

  /* InitializeConditions for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  root_model_X.uDOFQuaternion_CSTATE[1] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[4] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[7] = 0.0;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  root_model_DW.UnitDelay_DSTATE[1] = 0.0;

  /* InitializeConditions for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  root_model_X.uDOFQuaternion_CSTATE[2] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[5] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[8] = 0.0;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  root_model_DW.UnitDelay_DSTATE[2] = 0.0;

  /* InitializeConditions for EOM6DOFBodyQuat: '<Root>/6DOF (Quaternion)' */
  root_model_X.uDOFQuaternion_CSTATE[9] = 1.0;
  root_model_X.uDOFQuaternion_CSTATE[10] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[11] = 0.0;
  root_model_X.uDOFQuaternion_CSTATE[12] = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S3>/Correct1' */
  /* Start for MATLABSystem: '<S8>/MATLAB System' */
  root_model_DW.objisempty_d = true;
  root_model_DW.obj.isInitialized = 1;

  /* End of SystemInitialize for SubSystem: '<S3>/Correct1' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Predict' */
  /* Start for MATLABSystem: '<S10>/MATLAB System' */
  root_model_DW.objisempty = true;
  root_model_DW.obj_l.isInitialized = 1;

  /* End of SystemInitialize for SubSystem: '<S3>/Predict' */
}

/* Model terminate function */
void root_model_terminate(void)
{
  /* (no terminate code required) */
}
