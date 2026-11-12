/*
 * root_model_types.h
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

#ifndef root_model_types_h_
#define root_model_types_h_
#include "rtwtypes.h"
#include "coder_bounded_array.h"
#ifndef DEFINED_TYPEDEF_FOR_mpc_params_bus_3_
#define DEFINED_TYPEDEF_FOR_mpc_params_bus_3_

struct mpc_params_bus_3
{
  real_T Constant5;
  real_T Constant3;
  real_T Constant6;
  real_T Constant7[9];
  real_T Constant8;
};

#endif

#ifndef struct_matlabshared_tracking_interna_T
#define struct_matlabshared_tracking_interna_T

struct matlabshared_tracking_interna_T
{
  int32_T isInitialized;
};

#endif                              /* struct_matlabshared_tracking_interna_T */

#ifndef struct_matlabshared_tracking_inter_l_T
#define struct_matlabshared_tracking_inter_l_T

struct matlabshared_tracking_inter_l_T
{
  int32_T isInitialized;
};

#endif                              /* struct_matlabshared_tracking_inter_l_T */

/* Custom Type definition for MATLAB Function: '<S12>/NLMPC' */
#ifndef struct_sG8JZ69axY52WWR6RKyApQC_root__T
#define struct_sG8JZ69axY52WWR6RKyApQC_root__T

struct sG8JZ69axY52WWR6RKyApQC_root__T
{
  real_T penaltyParam;
  real_T threshold;
  int32_T nPenaltyDecreases;
  real_T linearizedConstrViol;
  real_T initFval;
  real_T initConstrViolationEq;
  real_T initConstrViolationIneq;
  real_T phi;
  real_T phiPrimePlus;
  real_T phiFullStep;
  real_T feasRelativeFactor;
  real_T nlpPrimalFeasError;
  real_T nlpDualFeasError;
  real_T nlpComplError;
  real_T firstOrderOpt;
  boolean_T hasObjective;
};

#endif                              /* struct_sG8JZ69axY52WWR6RKyApQC_root__T */

#ifndef struct_s7RdrPWkr8UPAUyTdDJkLaG_root__T
#define struct_s7RdrPWkr8UPAUyTdDJkLaG_root__T

struct s7RdrPWkr8UPAUyTdDJkLaG_root__T
{
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int32_T stepType;
};

#endif                              /* struct_s7RdrPWkr8UPAUyTdDJkLaG_root__T */

#ifndef struct_sttYSJM5GCi2c1Eu0R50efC_root__T
#define struct_sttYSJM5GCi2c1Eu0R50efC_root__T

struct sttYSJM5GCi2c1Eu0R50efC_root__T
{
  real_T iterations;
  real_T funcCount;
  char_T algorithm[3];
  real_T constrviolation;
  real_T stepsize;
  real_T lssteplength;
  real_T firstorderopt;
};

#endif                              /* struct_sttYSJM5GCi2c1Eu0R50efC_root__T */

#ifndef struct_somzaGboVhDG7PNQS6E98jD_root__T
#define struct_somzaGboVhDG7PNQS6E98jD_root__T

struct somzaGboVhDG7PNQS6E98jD_root__T
{
  char_T SolverName[7];
  int32_T MaxIterations;
  real_T StepTolerance;
  real_T OptimalityTolerance;
  real_T ConstraintTolerance;
  real_T ObjectiveLimit;
  real_T PricingTolerance;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
  boolean_T IterDisplayQP;
};

#endif                              /* struct_somzaGboVhDG7PNQS6E98jD_root__T */

#ifndef struct_cell_0_root_model_T
#define struct_cell_0_root_model_T

struct cell_0_root_model_T
{
  real_T f1;
  real_T f2;
  real_T f3;
  real_T f4[9];
  real_T f5;
};

#endif                                 /* struct_cell_0_root_model_T */

#ifndef struct_s_7iu1rzNoAoHigERqeOFm2G_root_T
#define struct_s_7iu1rzNoAoHigERqeOFm2G_root_T

struct s_7iu1rzNoAoHigERqeOFm2G_root_T
{
  int32_T nVarMax;
  int32_T mNonlinIneq;
  int32_T mNonlinEq;
  int32_T mIneq;
  int32_T mEq;
  int32_T iNonIneq0;
  int32_T iNonEq0;
  real_T sqpFval;
  real_T sqpFval_old;
  real_T xstarsqp[177];
  real_T xstarsqp_old[177];
  coder::bounded_array<real_T, 380U, 1U> cIneq;
  coder::bounded_array<real_T, 380U, 1U> cIneq_old;
  real_T cEq[170];
  real_T cEq_old[170];
  coder::bounded_array<real_T, 898U, 1U> grad;
  coder::bounded_array<real_T, 898U, 1U> grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  coder::bounded_array<real_T, 1625U, 1U> lambdasqp;
  coder::bounded_array<real_T, 1625U, 1U> lambdaStopTest;
  coder::bounded_array<real_T, 1625U, 1U> lambdaStopTestPrev;
  real_T steplength;
  coder::bounded_array<real_T, 1625U, 1U> delta_x;
  coder::bounded_array<real_T, 898U, 1U> socDirection;
  coder::bounded_array<int32_T, 1625U, 1U> workingset_old;
  coder::bounded_array<real_T, 233480U, 2U> JacCineqTrans_old;
  coder::bounded_array<real_T, 152660U, 2U> JacCeqTrans_old;
  coder::bounded_array<real_T, 898U, 1U> gradLag;
  coder::bounded_array<real_T, 898U, 1U> delta_gradLag;
  coder::bounded_array<real_T, 1625U, 1U> xstar;
  real_T fstar;
  real_T firstorderopt;
  coder::bounded_array<real_T, 1625U, 1U> lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  coder::bounded_array<real_T, 1625U, 1U> searchDir;
};

#endif                              /* struct_s_7iu1rzNoAoHigERqeOFm2G_root_T */

#ifndef struct_s_3DDm1jFKx2gyv5YUdHGqJC_root_T
#define struct_s_3DDm1jFKx2gyv5YUdHGqJC_root_T

struct s_3DDm1jFKx2gyv5YUdHGqJC_root_T
{
  coder::bounded_array<real_T, 1459250U, 2U> workspace_float;
  coder::bounded_array<int32_T, 1625U, 1U> workspace_int;
  coder::bounded_array<int32_T, 1625U, 1U> workspace_sort;
};

#endif                              /* struct_s_3DDm1jFKx2gyv5YUdHGqJC_root_T */

#ifndef struct_s_Jrk1gF3jitJmkPBgphEknE_root_T
#define struct_s_Jrk1gF3jitJmkPBgphEknE_root_T

struct s_Jrk1gF3jitJmkPBgphEknE_root_T
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  coder::bounded_array<real_T, 341240U, 1U> Aineq;
  coder::bounded_array<real_T, 380U, 1U> bineq;
  coder::bounded_array<real_T, 152660U, 1U> Aeq;
  real_T beq[170];
  coder::bounded_array<real_T, 898U, 1U> lb;
  coder::bounded_array<real_T, 898U, 1U> ub;
  coder::bounded_array<int32_T, 898U, 1U> indexLB;
  coder::bounded_array<int32_T, 898U, 1U> indexUB;
  coder::bounded_array<int32_T, 898U, 1U> indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[170];
  coder::bounded_array<real_T, 1459250U, 1U> ATwset;
  coder::bounded_array<real_T, 1625U, 1U> bwset;
  int32_T nActiveConstr;
  coder::bounded_array<real_T, 1625U, 1U> maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  coder::bounded_array<boolean_T, 1625U, 1U> isActiveConstr;
  coder::bounded_array<int32_T, 1625U, 1U> Wid;
  coder::bounded_array<int32_T, 1625U, 1U> Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

#endif                              /* struct_s_Jrk1gF3jitJmkPBgphEknE_root_T */

#ifndef struct_s_1ECtvwuWb7KTJMqOG7pbsD_root_T
#define struct_s_1ECtvwuWb7KTJMqOG7pbsD_root_T

struct s_1ECtvwuWb7KTJMqOG7pbsD_root_T
{
  coder::bounded_array<real_T, 898U, 1U> grad;
  coder::bounded_array<real_T, 897U, 1U> Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

#endif                              /* struct_s_1ECtvwuWb7KTJMqOG7pbsD_root_T */

#ifndef struct_s_Pct27jAhkqchx6iRs2zKpD_root_T
#define struct_s_Pct27jAhkqchx6iRs2zKpD_root_T

struct s_Pct27jAhkqchx6iRs2zKpD_root_T
{
  int32_T ldq;
  coder::bounded_array<real_T, 2640625U, 2U> QR;
  coder::bounded_array<real_T, 2640625U, 2U> Q;
  coder::bounded_array<int32_T, 1625U, 1U> jpvt;
  int32_T mrows;
  int32_T ncols;
  coder::bounded_array<real_T, 1625U, 1U> tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
};

#endif                              /* struct_s_Pct27jAhkqchx6iRs2zKpD_root_T */

#ifndef struct_s_mdyENYu4oaiCttMAPwg9iE_root_T
#define struct_s_mdyENYu4oaiCttMAPwg9iE_root_T

struct s_mdyENYu4oaiCttMAPwg9iE_root_T
{
  coder::bounded_array<real_T, 2640625U, 2U> FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_;
  real_T workspace2_;
};

#endif                              /* struct_s_mdyENYu4oaiCttMAPwg9iE_root_T */

#ifndef struct_s_WBLIzjzqQS4bg0a8DMrwYD_root_T
#define struct_s_WBLIzjzqQS4bg0a8DMrwYD_root_T

struct s_WBLIzjzqQS4bg0a8DMrwYD_root_T
{
  real_T x[17];
  real_T lastMV[3];
  real_T ref[130];
  real_T OutputWeights[130];
  real_T MVWeights[30];
  real_T MVRateWeights[30];
  real_T ECRWeight;
  real_T OutputMin[130];
  real_T OutputMax[130];
  real_T StateMin[170];
  real_T StateMax[170];
  real_T MVMin[30];
  real_T MVMax[30];
  real_T MVRateMin[30];
  real_T MVRateMax[30];
  real_T MVScaledTarget[30];
  cell_0_root_model_T Parameters;
};

#endif                              /* struct_s_WBLIzjzqQS4bg0a8DMrwYD_root_T */

#ifndef struct_scI1GGq4Cpah2bZUcB8s80D_root__T
#define struct_scI1GGq4Cpah2bZUcB8s80D_root__T

struct scI1GGq4Cpah2bZUcB8s80D_root__T
{
  real_T Ts;
  real_T CurrentStates[17];
  real_T LastMV[3];
  real_T References[130];
  real_T MVTarget[30];
  real_T PredictionHorizon;
  real_T NumOfStates;
  real_T NumOfOutputs;
  real_T NumOfInputs;
  real_T MVIndex[3];
  real_T InputPassivityIndex;
  real_T OutputPassivityIndex;
  boolean_T PassivityUsePredictedX;
};

#endif                              /* struct_scI1GGq4Cpah2bZUcB8s80D_root__T */

#ifndef struct_s_pVH6DktXFODPXN9VaZK2hE_root_T
#define struct_s_pVH6DktXFODPXN9VaZK2hE_root_T

struct s_pVH6DktXFODPXN9VaZK2hE_root_T
{
  s_WBLIzjzqQS4bg0a8DMrwYD_root_T runtimedata;
  scI1GGq4Cpah2bZUcB8s80D_root__T userdata;
};

#endif                              /* struct_s_pVH6DktXFODPXN9VaZK2hE_root_T */

#ifndef struct_anonymous_function_root_model_T
#define struct_anonymous_function_root_model_T

struct anonymous_function_root_model_T
{
  s_pVH6DktXFODPXN9VaZK2hE_root_T workspace;
};

#endif                              /* struct_anonymous_function_root_model_T */

#ifndef struct_coder_internal_stickyStruct_1_T
#define struct_coder_internal_stickyStruct_1_T

struct coder_internal_stickyStruct_1_T
{
  anonymous_function_root_model_T b_value;
};

#endif                              /* struct_coder_internal_stickyStruct_1_T */

#ifndef struct_coder_internal_stickyStruct_k_T
#define struct_coder_internal_stickyStruct_k_T

struct coder_internal_stickyStruct_k_T
{
  anonymous_function_root_model_T b_value;
  coder_internal_stickyStruct_1_T next;
};

#endif                              /* struct_coder_internal_stickyStruct_k_T */

#ifndef struct_coder_internal_stickyStruct_n_T
#define struct_coder_internal_stickyStruct_n_T

struct coder_internal_stickyStruct_n_T
{
  coder_internal_stickyStruct_k_T next;
};

#endif                              /* struct_coder_internal_stickyStruct_n_T */

#ifndef struct_coder_internal_stickyStruc_kx_T
#define struct_coder_internal_stickyStruc_kx_T

struct coder_internal_stickyStruc_kx_T
{
  int32_T b_value;
  coder_internal_stickyStruct_n_T next;
};

#endif                              /* struct_coder_internal_stickyStruc_kx_T */

#ifndef struct_coder_internal_stickyStruct_e_T
#define struct_coder_internal_stickyStruct_e_T

struct coder_internal_stickyStruct_e_T
{
  coder_internal_stickyStruc_kx_T next;
};

#endif                              /* struct_coder_internal_stickyStruct_e_T */

#ifndef struct_coder_internal_stickyStruc_kz_T
#define struct_coder_internal_stickyStruc_kz_T

struct coder_internal_stickyStruc_kz_T
{
  coder_internal_stickyStruct_e_T next;
};

#endif                              /* struct_coder_internal_stickyStruc_kz_T */

#ifndef struct_coder_internal_stickyStruct_g_T
#define struct_coder_internal_stickyStruct_g_T

struct coder_internal_stickyStruct_g_T
{
  coder_internal_stickyStruc_kz_T next;
};

#endif                              /* struct_coder_internal_stickyStruct_g_T */

#ifndef struct_coder_internal_stickyStruc_gj_T
#define struct_coder_internal_stickyStruc_gj_T

struct coder_internal_stickyStruc_gj_T
{
  coder_internal_stickyStruct_g_T next;
};

#endif                              /* struct_coder_internal_stickyStruc_gj_T */

#ifndef struct_coder_internal_stickyStruct_2_T
#define struct_coder_internal_stickyStruct_2_T

struct coder_internal_stickyStruct_2_T
{
  coder_internal_stickyStruc_gj_T next;
};

#endif                              /* struct_coder_internal_stickyStruct_2_T */

#ifndef struct_s_iCcJrJA6AdN0t7bEVlwtZ_root__T
#define struct_s_iCcJrJA6AdN0t7bEVlwtZ_root__T

struct s_iCcJrJA6AdN0t7bEVlwtZ_root__T
{
  anonymous_function_root_model_T objfun;
  anonymous_function_root_model_T nonlin;
  real_T f_1;
  coder::bounded_array<real_T, 260U, 1U> cIneq_1;
  real_T cEq_1[170];
  real_T f_2;
  coder::bounded_array<real_T, 260U, 1U> cIneq_2;
  real_T cEq_2[170];
  int32_T nVar;
  int32_T mIneq;
  int32_T mEq;
  int32_T numEvals;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T isEmptyNonlcon;
  boolean_T hasLB[177];
  boolean_T hasUB[177];
  boolean_T hasBounds;
  int32_T FiniteDifferenceType;
};

#endif                              /* struct_s_iCcJrJA6AdN0t7bEVlwtZ_root__T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_root_model_T RT_MODEL_root_model_T;

#endif                                 /* root_model_types_h_ */
