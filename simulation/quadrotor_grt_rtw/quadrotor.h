/*
 * quadrotor.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "quadrotor".
 *
 * Model version              : 1.40
 * Simulink Coder version : 9.1 (R2019a) 23-Nov-2018
 * C source code generated on : Thu May 23 13:00:01 2019
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_quadrotor_h_
#define RTW_HEADER_quadrotor_h_
#include <math.h>
#include <string.h>
#ifndef quadrotor_COMMON_INCLUDES_
# define quadrotor_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* quadrotor_COMMON_INCLUDES_ */

#include "quadrotor_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Selector1[9];                 /* '<S5>/Selector1' */
  real_T Selector[9];                  /* '<S5>/Selector' */
  real_T Selector2[9];                 /* '<S5>/Selector2' */
  real_T q0;                           /* '<S14>/q0' */
  real_T q1;                           /* '<S14>/q1' */
  real_T q2;                           /* '<S14>/q2' */
  real_T q3;                           /* '<S14>/q3' */
  real_T TmpSignalConversionAtq0q1q2q3In[4];
                                     /* '<S4>/Rotation Angles to Quaternions' */
  real_T VectorConcatenate[9];         /* '<S25>/Vector Concatenate' */
  real_T StateSpace;                   /* '<Root>/State-Space' */
  real_T StateSpace1;                  /* '<Root>/State-Space1' */
  real_T StateSpace2;                  /* '<Root>/State-Space2' */
  real_T StateSpace3;                  /* '<Root>/State-Space3' */
  real_T pqr[3];                       /* '<S1>/p,q,r ' */
  real_T TmpSignalConversionAtq0q1q2q3_g[4];/* '<S4>/qdot' */
  real_T Merge[4];                     /* '<S2>/Merge' */
  real_T Product2[3];                  /* '<S5>/Product2' */
  real_T ubvbwb[3];                    /* '<S1>/ub,vb,wb' */
  real_T Sum[3];                       /* '<S1>/Sum' */
  real_T Product[3];                   /* '<S11>/Product' */
  real_T VectorConcatenate_c[3];       /* '<S29>/Vector Concatenate' */
  real_T xeyeze[3];                    /* '<S1>/xe,ye,ze' */
  real_T Merge_m;                      /* '<S31>/Merge' */
} B_quadrotor_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Product2_DWORK4[9];           /* '<S5>/Product2' */
  void* Assertion_slioAccessor;        /* '<S75>/Assertion' */
  void* Assertion_slioAccessor_l;      /* '<S76>/Assertion' */
  void* Assertion_slioAccessor_k;      /* '<S77>/Assertion' */
  void* Assertion_slioAccessor_h;      /* '<S78>/Assertion' */
  int_T q0q1q2q3_IWORK;                /* '<S4>/q0 q1 q2 q3' */
  int8_T If_ActiveSubsystem;           /* '<S2>/If' */
  int8_T If_ActiveSubsystem_h;         /* '<S31>/If' */
  int8_T If1_ActiveSubsystem;          /* '<S46>/If1' */
  int8_T FindMaximumDiagonalValue_Active;/* '<S44>/Find Maximum Diagonal Value' */
} DW_quadrotor_T;

/* Continuous states (default storage) */
typedef struct {
  real_T q0q1q2q3_CSTATE[4];           /* '<S4>/q0 q1 q2 q3' */
  real_T StateSpace_CSTATE;            /* '<Root>/State-Space' */
  real_T StateSpace1_CSTATE;           /* '<Root>/State-Space1' */
  real_T StateSpace2_CSTATE;           /* '<Root>/State-Space2' */
  real_T StateSpace3_CSTATE;           /* '<Root>/State-Space3' */
  real_T pqr_CSTATE[3];                /* '<S1>/p,q,r ' */
  real_T ubvbwb_CSTATE[3];             /* '<S1>/ub,vb,wb' */
  real_T xeyeze_CSTATE[3];             /* '<S1>/xe,ye,ze' */
} X_quadrotor_T;

/* State derivatives (default storage) */
typedef struct {
  real_T q0q1q2q3_CSTATE[4];           /* '<S4>/q0 q1 q2 q3' */
  real_T StateSpace_CSTATE;            /* '<Root>/State-Space' */
  real_T StateSpace1_CSTATE;           /* '<Root>/State-Space1' */
  real_T StateSpace2_CSTATE;           /* '<Root>/State-Space2' */
  real_T StateSpace3_CSTATE;           /* '<Root>/State-Space3' */
  real_T pqr_CSTATE[3];                /* '<S1>/p,q,r ' */
  real_T ubvbwb_CSTATE[3];             /* '<S1>/ub,vb,wb' */
  real_T xeyeze_CSTATE[3];             /* '<S1>/xe,ye,ze' */
} XDot_quadrotor_T;

/* State disabled  */
typedef struct {
  boolean_T q0q1q2q3_CSTATE[4];        /* '<S4>/q0 q1 q2 q3' */
  boolean_T StateSpace_CSTATE;         /* '<Root>/State-Space' */
  boolean_T StateSpace1_CSTATE;        /* '<Root>/State-Space1' */
  boolean_T StateSpace2_CSTATE;        /* '<Root>/State-Space2' */
  boolean_T StateSpace3_CSTATE;        /* '<Root>/State-Space3' */
  boolean_T pqr_CSTATE[3];             /* '<S1>/p,q,r ' */
  boolean_T ubvbwb_CSTATE[3];          /* '<S1>/ub,vb,wb' */
  boolean_T xeyeze_CSTATE[3];          /* '<S1>/xe,ye,ze' */
} XDis_quadrotor_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T rpm_cmd[4];                   /* '<Root>/rpm_cmd' */
} ExtU_quadrotor_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T pqr[3];                       /* '<Root>/pqr' */
  real_T zeta[3];                      /* '<Root>/zeta' */
  real_T xyz[3];                       /* '<Root>/xyz' */
  real_T uvw[3];                       /* '<Root>/uvw' */
  real_T pqr_dot[3];                   /* '<Root>/pqr_dot' */
  real_T uvw_dot[3];                   /* '<Root>/uvw_dot' */
  real_T uvw_earth[3];                 /* '<Root>/uvw_earth' */
  real_T q[4];                         /* '<Root>/q' */
  real_T rpm[4];                       /* '<Root>/rpm' */
} ExtY_quadrotor_T;

/* Real-time Model Data Structure */
struct tag_RTM_quadrotor_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_quadrotor_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[17];
  real_T odeF[4][17];
  ODE4_IntgData intgData;

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
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_quadrotor_T quadrotor_B;

/* Continuous states (default storage) */
extern X_quadrotor_T quadrotor_X;

/* Block states (default storage) */
extern DW_quadrotor_T quadrotor_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_quadrotor_T quadrotor_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_quadrotor_T quadrotor_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T simparam_I[9];           /* Variable: simparam_I
                                        * Referenced by: '<S6>/Constant1'
                                        */
extern real_T simparam_g;              /* Variable: simparam_g
                                        * Referenced by: '<Root>/g'
                                        */
extern real_T simparam_init_euler[3];  /* Variable: simparam_init_euler
                                        * Referenced by: '<S4>/Initial Euler Angles'
                                        */
extern real_T simparam_init_omega[3];  /* Variable: simparam_init_omega
                                        * Referenced by: '<S1>/p,q,r '
                                        */
extern real_T simparam_init_pos[3];    /* Variable: simparam_init_pos
                                        * Referenced by: '<S1>/xe,ye,ze'
                                        */
extern real_T simparam_init_rpm[4];    /* Variable: simparam_init_rpm
                                        * Referenced by:
                                        *   '<Root>/State-Space'
                                        *   '<Root>/State-Space1'
                                        *   '<Root>/State-Space2'
                                        *   '<Root>/State-Space3'
                                        */
extern real_T simparam_init_vel[3];    /* Variable: simparam_init_vel
                                        * Referenced by: '<S1>/ub,vb,wb'
                                        */
extern real_T simparam_kq;             /* Variable: simparam_kq
                                        * Referenced by: '<Root>/kq'
                                        */
extern real_T simparam_kt;             /* Variable: simparam_kt
                                        * Referenced by: '<Root>/kt'
                                        */
extern real_T simparam_m;              /* Variable: simparam_m
                                        * Referenced by:
                                        *   '<Root>/m'
                                        *   '<S6>/Constant'
                                        */
extern real_T simparam_r;              /* Variable: simparam_r
                                        * Referenced by: '<Root>/r'
                                        */
extern real_T simparam_tau;            /* Variable: simparam_tau
                                        * Referenced by:
                                        *   '<Root>/State-Space'
                                        *   '<Root>/State-Space1'
                                        *   '<Root>/State-Space2'
                                        *   '<Root>/State-Space3'
                                        */

/* Model entry point functions */
extern void quadrotor_initialize(void);
extern void quadrotor_step(void);
extern void quadrotor_terminate(void);

/* Real-time Model object */
extern RT_MODEL_quadrotor_T *const quadrotor_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S10>/Unit Conversion' : Unused code path elimination
 * Block '<S25>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S15>/High Gain Quaternion Normalization' : Eliminated nontunable gain of 1
 * Block '<S38>/Reshape1' : Reshape block reduction
 * Block '<S38>/Reshape2' : Reshape block reduction
 * Block '<S39>/Reshape1' : Reshape block reduction
 * Block '<S39>/Reshape2' : Reshape block reduction
 * Block '<S5>/Reshape' : Reshape block reduction
 * Block '<S5>/Reshape1' : Reshape block reduction
 * Block '<S8>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S9>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S11>/Reshape1' : Reshape block reduction
 * Block '<S11>/Reshape2' : Reshape block reduction
 * Block '<S2>/Reshape 3x3 -> 9' : Reshape block reduction
 * Block '<S73>/Reshape' : Reshape block reduction
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
 * '<Root>' : 'quadrotor'
 * '<S1>'   : 'quadrotor/6DOF (Quaternion)'
 * '<S2>'   : 'quadrotor/Direction Cosine Matrix  to Quaternions'
 * '<S3>'   : 'quadrotor/MATLAB Function'
 * '<S4>'   : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles'
 * '<S5>'   : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot'
 * '<S6>'   : 'quadrotor/6DOF (Quaternion)/Determine Force,  Mass & Inertia'
 * '<S7>'   : 'quadrotor/6DOF (Quaternion)/Vbxw'
 * '<S8>'   : 'quadrotor/6DOF (Quaternion)/Velocity Conversion'
 * '<S9>'   : 'quadrotor/6DOF (Quaternion)/Velocity Conversion1'
 * '<S10>'  : 'quadrotor/6DOF (Quaternion)/Velocity Conversion2'
 * '<S11>'  : 'quadrotor/6DOF (Quaternion)/transform to Inertial axes '
 * '<S12>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix'
 * '<S13>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles'
 * '<S14>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Rotation Angles to Quaternions'
 * '<S15>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/qdot'
 * '<S16>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A11'
 * '<S17>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A12'
 * '<S18>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A13'
 * '<S19>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A21'
 * '<S20>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A22'
 * '<S21>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A23'
 * '<S22>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A31'
 * '<S23>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A32'
 * '<S24>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A33'
 * '<S25>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S26>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S27>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S28>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S29>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation'
 * '<S30>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize'
 * '<S31>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
 * '<S32>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
 * '<S33>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
 * '<S34>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
 * '<S35>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
 * '<S36>'  : 'quadrotor/6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S37>'  : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product'
 * '<S38>'  : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot/I x w'
 * '<S39>'  : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot/I x w1'
 * '<S40>'  : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S41>'  : 'quadrotor/6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S42>'  : 'quadrotor/6DOF (Quaternion)/Vbxw/Subsystem'
 * '<S43>'  : 'quadrotor/6DOF (Quaternion)/Vbxw/Subsystem1'
 * '<S44>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace'
 * '<S45>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Positive Trace'
 * '<S46>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM'
 * '<S47>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/trace(DCM)'
 * '<S48>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
 * '<S49>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
 * '<S50>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
 * '<S51>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
 * '<S52>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S53>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S54>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S55>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
 * '<S56>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
 * '<S57>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S58>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S59>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S60>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
 * '<S61>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
 * '<S62>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S63>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S64>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S65>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
 * '<S66>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
 * '<S67>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S68>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S69>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S70>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
 * '<S71>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S72>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
 * '<S73>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S74>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
 * '<S75>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S76>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S77>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S78>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S79>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S80>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S81>'  : 'quadrotor/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 */
#endif                                 /* RTW_HEADER_quadrotor_h_ */
