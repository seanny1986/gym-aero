/*
 * quadrotor.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "quadrotor".
 *
 * Model version              : 1.55
 * Simulink Coder version : 9.1 (R2019a) 23-Nov-2018
 * C source code generated on : Sat Jun 29 14:42:08 2019
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "quadrotor.h"
#include "quadrotor_private.h"

/* Exported block parameters */
real_T simparam_I[9] = { 7.5e-3, 0.0, 0.0, 0.0, 7.5e-3, 0.0, 0.0, 0.0, 1.3e-2 } ;  /* Variable: simparam_I
                                                                                     * Referenced by: '<S6>/Constant1'
                                                                                     */

real_T simparam_g = 9.81;              /* Variable: simparam_g
                                        * Referenced by: '<Root>/g'
                                        */
real_T simparam_init_euler[3] = { 0.0, 0.0, 0.0 } ;/* Variable: simparam_init_euler
                                                    * Referenced by: '<S4>/Initial Euler Angles'
                                                    */

real_T simparam_init_omega[3] = { 0.0, 0.0, 0.0 } ;/* Variable: simparam_init_omega
                                                    * Referenced by: '<S1>/p,q,r '
                                                    */

real_T simparam_init_pos[3] = { 0.0, 0.0, 0.0 } ;/* Variable: simparam_init_pos
                                                  * Referenced by: '<S1>/xe,ye,ze'
                                                  */

real_T simparam_init_rpm[4] = { 0.0, 0.0, 0.0, 0.0 }; /* Variable: simparam_init_rpm
                                                       * Referenced by:
                                                       *   '<Root>/State-Space'
                                                       *   '<Root>/State-Space1'
                                                       *   '<Root>/State-Space2'
                                                       *   '<Root>/State-Space3'
                                                       */

real_T simparam_init_vel[3] = { 0.0, 0.0, 0.0 } ;/* Variable: simparam_init_vel
                                                  * Referenced by: '<S1>/ub,vb,wb'
                                                  */

real_T simparam_kq = 7.5e-7;              /* Variable: simparam_kq
                                        * Referenced by: '<Root>/kq'
                                        */
real_T simparam_kt = 3.13e-5;              /* Variable: simparam_kt
                                        * Referenced by: '<Root>/kt'
                                        */
real_T simparam_m = 0.65;               /* Variable: simparam_m
                                        * Referenced by:
                                        *   '<Root>/m'
                                        *   '<S6>/Constant'
                                        */
real_T simparam_r = 0.23;              /* Variable: simparam_r
                                        * Referenced by: '<Root>/r'
                                        */
real_T simparam_tau = 0.18;             /* Variable: simparam_tau
                                        * Referenced by:
                                        *   '<Root>/State-Space'
                                        *   '<Root>/State-Space1'
                                        *   '<Root>/State-Space2'
                                        *   '<Root>/State-Space3'
                                        */
real_T simparam_rpm_max = 5000.0;      /* Variable: simparam_rpm_max
                                        * Referenced by: '<Root>/Saturation'
                                        */
real_T simparam_rpm_min = 0.0;      /* Variable: simparam_rpm_min
                                        * Referenced by: '<Root>/Saturation'
                                        */

/* Block signals (default storage) */
B_quadrotor_T quadrotor_B;

/* Continuous states */
X_quadrotor_T quadrotor_X;

/* Block states (default storage) */
DW_quadrotor_T quadrotor_DW;

/* External inputs (root inport signals with default storage) */
ExtU_quadrotor_T quadrotor_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_quadrotor_T quadrotor_Y;

/* Real-time model */
RT_MODEL_quadrotor_T quadrotor_M_;
RT_MODEL_quadrotor_T *const quadrotor_M = &quadrotor_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 17;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  quadrotor_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  quadrotor_step();
  quadrotor_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  quadrotor_step();
  quadrotor_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  quadrotor_step();
  quadrotor_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

void rt_mrdivide_U1d1x3_U2d3x3_Yd1x3(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T A[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real_T maxval;
  real_T a21;
  int32_T rtemp;
  memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(u1[0]);
  a21 = fabs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[3 + r2] -= A[3 + r1] * A[r2];
  A[3 + r3] -= A[3 + r1] * A[r3];
  A[6 + r2] -= A[6 + r1] * A[r2];
  A[6 + r3] -= A[6 + r1] * A[r3];
  if (fabs(A[3 + r3]) > fabs(A[3 + r2])) {
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[3 + r3] /= A[3 + r2];
  A[6 + r3] -= A[3 + r3] * A[6 + r2];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[3 + r1] * y[r1];
  y[r3] = u0[2] - A[6 + r1] * y[r1];
  y[r2] /= A[3 + r2];
  y[r3] -= A[6 + r2] * y[r2];
  y[r3] /= A[6 + r3];
  y[r2] -= A[3 + r3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

/* Model step function */
void quadrotor_step(void)
{
  real_T T[4];
  real_T rtb_sincos_o1[3];
  real_T rtb_fcn2;
  real_T rtb_fcn4;
  real_T rtb_Product2_h;
  real_T rtb_Product1_p;
  real_T rtb_sincos_o2[3];
  real_T rtb_VectorConcatenate[18];
  real_T Product_tmp[9];
  real_T tmp[3];
  int32_T iU;
  real_T Q_idx_2;
  int32_T rtb_VectorConcatenate_tmp;
  int32_T rtb_VectorConcatenate_tmp_0;
  int32_T rtb_VectorConcatenate_tmp_1;
  real_T VectorConcatenate_tmp;
  real_T VectorConcatenate_tmp_0;
  real_T VectorConcatenate_tmp_1;
  real_T VectorConcatenate_tmp_2;
  real_T VectorConcatenate_tmp_3;
  real_T VectorConcatenate_tmp_4;
  real_T VectorConcatenate_tmp_5;
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* set solver stop time */
    if (!(quadrotor_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&quadrotor_M->solverInfo,
                            ((quadrotor_M->Timing.clockTickH0 + 1) *
        quadrotor_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&quadrotor_M->solverInfo,
                            ((quadrotor_M->Timing.clockTick0 + 1) *
        quadrotor_M->Timing.stepSize0 + quadrotor_M->Timing.clockTickH0 *
        quadrotor_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(quadrotor_M)) {
    quadrotor_M->Timing.t[0] = rtsiGetT(&quadrotor_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Gain: '<S14>/1//2' incorporates:
     *  Constant: '<S4>/Initial Euler Angles'
     */
    rtb_sincos_o1[0] = 0.5 * simparam_init_euler[2];
    rtb_sincos_o1[1] = 0.5 * simparam_init_euler[1];
    rtb_sincos_o1[2] = 0.5 * simparam_init_euler[0];
    for (iU = 0; iU < 3; iU++) {
      /* Concatenate: '<S6>/Vector Concatenate' incorporates:
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Constant2'
       *  Selector: '<S5>/Selector1'
       */
      rtb_VectorConcatenate[6 * iU] = simparam_I[3 * iU];
      rtb_VectorConcatenate_tmp = 3 + 6 * iU;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp] = 0.0;

      /* Selector: '<S5>/Selector1' */
      quadrotor_B.Selector1[3 * iU] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp];

      /* Selector: '<S5>/Selector' incorporates:
       *  Selector: '<S5>/Selector2'
       */
      rtb_fcn2 = rtb_VectorConcatenate[6 * iU];
      quadrotor_B.Selector[3 * iU] = rtb_fcn2;

      /* Selector: '<S5>/Selector2' */
      quadrotor_B.Selector2[3 * iU] = rtb_fcn2;

      /* Concatenate: '<S6>/Vector Concatenate' incorporates:
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Constant2'
       *  Selector: '<S5>/Selector'
       *  Selector: '<S5>/Selector1'
       *  Selector: '<S5>/Selector2'
       */
      rtb_VectorConcatenate_tmp = 3 * iU + 1;
      rtb_VectorConcatenate_tmp_1 = 1 + 6 * iU;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1] =
        simparam_I[rtb_VectorConcatenate_tmp];
      rtb_VectorConcatenate_tmp_0 = 4 + 6 * iU;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0] = 0.0;

      /* Selector: '<S5>/Selector1' */
      quadrotor_B.Selector1[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0];

      /* Selector: '<S5>/Selector' */
      quadrotor_B.Selector[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1];

      /* Selector: '<S5>/Selector2' */
      quadrotor_B.Selector2[rtb_VectorConcatenate_tmp] = rtb_VectorConcatenate[6
        * iU + 1];

      /* Concatenate: '<S6>/Vector Concatenate' incorporates:
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Constant2'
       *  Selector: '<S5>/Selector'
       *  Selector: '<S5>/Selector1'
       *  Selector: '<S5>/Selector2'
       */
      rtb_VectorConcatenate_tmp = 3 * iU + 2;
      rtb_VectorConcatenate_tmp_1 = 2 + 6 * iU;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1] =
        simparam_I[rtb_VectorConcatenate_tmp];
      rtb_VectorConcatenate_tmp_0 = 5 + 6 * iU;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0] = 0.0;

      /* Selector: '<S5>/Selector1' */
      quadrotor_B.Selector1[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0];

      /* Selector: '<S5>/Selector' */
      quadrotor_B.Selector[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1];

      /* Selector: '<S5>/Selector2' */
      quadrotor_B.Selector2[rtb_VectorConcatenate_tmp] = rtb_VectorConcatenate[6
        * iU + 2];

      /* Trigonometry: '<S14>/sincos' */
      rtb_sincos_o2[iU] = cos(rtb_sincos_o1[iU]);
      rtb_sincos_o1[iU] = sin(rtb_sincos_o1[iU]);
    }

    /* Fcn: '<S14>/q0' incorporates:
     *  Fcn: '<S14>/q1'
     */
    rtb_fcn2 = rtb_sincos_o2[0] * rtb_sincos_o2[1];
    rtb_Product1_p = rtb_sincos_o1[0] * rtb_sincos_o1[1];
    quadrotor_B.q0 = rtb_fcn2 * rtb_sincos_o2[2] + rtb_Product1_p *
      rtb_sincos_o1[2];

    /* Fcn: '<S14>/q1' */
    quadrotor_B.q1 = rtb_fcn2 * rtb_sincos_o1[2] - rtb_Product1_p *
      rtb_sincos_o2[2];

    /* Fcn: '<S14>/q2' incorporates:
     *  Fcn: '<S14>/q3'
     */
    rtb_fcn2 = rtb_sincos_o1[0] * rtb_sincos_o2[1];
    rtb_Product1_p = rtb_sincos_o2[0] * rtb_sincos_o1[1];
    quadrotor_B.q2 = rtb_Product1_p * rtb_sincos_o2[2] + rtb_fcn2 *
      rtb_sincos_o1[2];

    /* Fcn: '<S14>/q3' */
    quadrotor_B.q3 = rtb_fcn2 * rtb_sincos_o2[2] - rtb_Product1_p *
      rtb_sincos_o1[2];
  }

  /* SignalConversion: '<S4>/TmpSignal ConversionAtq0 q1 q2 q3Inport2' */
  quadrotor_B.TmpSignalConversionAtq0q1q2q3In[0] = quadrotor_B.q0;
  quadrotor_B.TmpSignalConversionAtq0q1q2q3In[1] = quadrotor_B.q1;
  quadrotor_B.TmpSignalConversionAtq0q1q2q3In[2] = quadrotor_B.q2;
  quadrotor_B.TmpSignalConversionAtq0q1q2q3In[3] = quadrotor_B.q3;

  /* Integrator: '<S4>/q0 q1 q2 q3' */
  if (quadrotor_DW.q0q1q2q3_IWORK != 0) {
    quadrotor_X.q0q1q2q3_CSTATE[0] =
      quadrotor_B.TmpSignalConversionAtq0q1q2q3In[0];
    quadrotor_X.q0q1q2q3_CSTATE[1] =
      quadrotor_B.TmpSignalConversionAtq0q1q2q3In[1];
    quadrotor_X.q0q1q2q3_CSTATE[2] =
      quadrotor_B.TmpSignalConversionAtq0q1q2q3In[2];
    quadrotor_X.q0q1q2q3_CSTATE[3] =
      quadrotor_B.TmpSignalConversionAtq0q1q2q3In[3];
  }

  /* Sqrt: '<S27>/sqrt' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   *  Product: '<S28>/Product'
   *  Product: '<S28>/Product1'
   *  Product: '<S28>/Product2'
   *  Product: '<S28>/Product3'
   *  Sum: '<S28>/Sum'
   */
  rtb_fcn2 = sqrt(((quadrotor_X.q0q1q2q3_CSTATE[0] *
                    quadrotor_X.q0q1q2q3_CSTATE[0] +
                    quadrotor_X.q0q1q2q3_CSTATE[1] *
                    quadrotor_X.q0q1q2q3_CSTATE[1]) +
                   quadrotor_X.q0q1q2q3_CSTATE[2] * quadrotor_X.q0q1q2q3_CSTATE
                   [2]) + quadrotor_X.q0q1q2q3_CSTATE[3] *
                  quadrotor_X.q0q1q2q3_CSTATE[3]);

  /* Product: '<S26>/Product' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_Product1_p = quadrotor_X.q0q1q2q3_CSTATE[0] / rtb_fcn2;

  /* Product: '<S26>/Product1' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_Product2_h = quadrotor_X.q0q1q2q3_CSTATE[1] / rtb_fcn2;

  /* Product: '<S26>/Product2' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_fcn4 = quadrotor_X.q0q1q2q3_CSTATE[2] / rtb_fcn2;

  /* Product: '<S26>/Product3' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_fcn2 = quadrotor_X.q0q1q2q3_CSTATE[3] / rtb_fcn2;

  /* Product: '<S16>/Product3' incorporates:
   *  Product: '<S20>/Product3'
   */
  Q_idx_2 = rtb_Product1_p * rtb_Product1_p;

  /* Product: '<S16>/Product2' incorporates:
   *  Product: '<S20>/Product2'
   */
  VectorConcatenate_tmp_1 = rtb_Product2_h * rtb_Product2_h;

  /* Product: '<S16>/Product1' incorporates:
   *  Product: '<S20>/Product1'
   *  Product: '<S24>/Product1'
   */
  VectorConcatenate_tmp_2 = rtb_fcn4 * rtb_fcn4;

  /* Product: '<S16>/Product' incorporates:
   *  Product: '<S20>/Product'
   *  Product: '<S24>/Product'
   */
  VectorConcatenate_tmp_3 = rtb_fcn2 * rtb_fcn2;

  /* Sum: '<S16>/Sum' incorporates:
   *  Product: '<S16>/Product'
   *  Product: '<S16>/Product1'
   *  Product: '<S16>/Product2'
   *  Product: '<S16>/Product3'
   */
  quadrotor_B.VectorConcatenate[0] = ((Q_idx_2 + VectorConcatenate_tmp_1) -
    VectorConcatenate_tmp_2) - VectorConcatenate_tmp_3;

  /* Product: '<S19>/Product3' incorporates:
   *  Product: '<S17>/Product3'
   */
  VectorConcatenate_tmp = rtb_fcn2 * rtb_Product1_p;

  /* Product: '<S19>/Product2' incorporates:
   *  Product: '<S17>/Product2'
   */
  VectorConcatenate_tmp_0 = rtb_Product2_h * rtb_fcn4;

  /* Gain: '<S19>/Gain' incorporates:
   *  Product: '<S19>/Product2'
   *  Product: '<S19>/Product3'
   *  Sum: '<S19>/Sum'
   */
  quadrotor_B.VectorConcatenate[1] = (VectorConcatenate_tmp_0 -
    VectorConcatenate_tmp) * 2.0;

  /* Product: '<S22>/Product2' incorporates:
   *  Product: '<S18>/Product2'
   */
  VectorConcatenate_tmp_4 = rtb_Product2_h * rtb_fcn2;

  /* Product: '<S22>/Product1' incorporates:
   *  Product: '<S18>/Product1'
   */
  VectorConcatenate_tmp_5 = rtb_Product1_p * rtb_fcn4;

  /* Gain: '<S22>/Gain' incorporates:
   *  Product: '<S22>/Product1'
   *  Product: '<S22>/Product2'
   *  Sum: '<S22>/Sum'
   */
  quadrotor_B.VectorConcatenate[2] = (VectorConcatenate_tmp_5 +
    VectorConcatenate_tmp_4) * 2.0;

  /* Gain: '<S17>/Gain' incorporates:
   *  Sum: '<S17>/Sum'
   */
  quadrotor_B.VectorConcatenate[3] = (VectorConcatenate_tmp +
    VectorConcatenate_tmp_0) * 2.0;

  /* Sum: '<S20>/Sum' incorporates:
   *  Sum: '<S24>/Sum'
   */
  Q_idx_2 -= VectorConcatenate_tmp_1;
  quadrotor_B.VectorConcatenate[4] = (Q_idx_2 + VectorConcatenate_tmp_2) -
    VectorConcatenate_tmp_3;

  /* Product: '<S23>/Product1' incorporates:
   *  Product: '<S21>/Product1'
   */
  VectorConcatenate_tmp_1 = rtb_Product1_p * rtb_Product2_h;

  /* Product: '<S23>/Product2' incorporates:
   *  Product: '<S21>/Product2'
   */
  VectorConcatenate_tmp = rtb_fcn4 * rtb_fcn2;

  /* Gain: '<S23>/Gain' incorporates:
   *  Product: '<S23>/Product1'
   *  Product: '<S23>/Product2'
   *  Sum: '<S23>/Sum'
   */
  quadrotor_B.VectorConcatenate[5] = (VectorConcatenate_tmp -
    VectorConcatenate_tmp_1) * 2.0;

  /* Gain: '<S18>/Gain' incorporates:
   *  Sum: '<S18>/Sum'
   */
  quadrotor_B.VectorConcatenate[6] = (VectorConcatenate_tmp_4 -
    VectorConcatenate_tmp_5) * 2.0;

  /* Gain: '<S21>/Gain' incorporates:
   *  Sum: '<S21>/Sum'
   */
  quadrotor_B.VectorConcatenate[7] = (VectorConcatenate_tmp_1 +
    VectorConcatenate_tmp) * 2.0;

  /* Sum: '<S24>/Sum' */
  quadrotor_B.VectorConcatenate[8] = (Q_idx_2 - VectorConcatenate_tmp_2) +
    VectorConcatenate_tmp_3;

  /* StateSpace: '<Root>/State-Space' */
  quadrotor_B.StateSpace = 0.0;
  quadrotor_B.StateSpace += quadrotor_X.StateSpace_CSTATE;

  /* StateSpace: '<Root>/State-Space1' */
  quadrotor_B.StateSpace1 = 0.0;
  quadrotor_B.StateSpace1 += quadrotor_X.StateSpace1_CSTATE;

  /* StateSpace: '<Root>/State-Space2' */
  quadrotor_B.StateSpace2 = 0.0;
  quadrotor_B.StateSpace2 += quadrotor_X.StateSpace2_CSTATE;

  /* StateSpace: '<Root>/State-Space3' */
  quadrotor_B.StateSpace3 = 0.0;
  quadrotor_B.StateSpace3 += quadrotor_X.StateSpace3_CSTATE;
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/rpm' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold8'
     */
    quadrotor_Y.rpm[0] = quadrotor_B.StateSpace;
    quadrotor_Y.rpm[1] = quadrotor_B.StateSpace1;
    quadrotor_Y.rpm[2] = quadrotor_B.StateSpace2;
    quadrotor_Y.rpm[3] = quadrotor_B.StateSpace3;
  }

  /* Integrator: '<S1>/p,q,r ' */
  quadrotor_B.pqr[0] = quadrotor_X.pqr_CSTATE[0];
  quadrotor_B.pqr[1] = quadrotor_X.pqr_CSTATE[1];
  quadrotor_B.pqr[2] = quadrotor_X.pqr_CSTATE[2];

  /* DotProduct: '<S15>/Dot Product' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   *  Sum: '<S36>/Sum'
   */
  rtb_fcn2 = ((quadrotor_X.q0q1q2q3_CSTATE[0] * quadrotor_X.q0q1q2q3_CSTATE[0] +
               quadrotor_X.q0q1q2q3_CSTATE[1] * quadrotor_X.q0q1q2q3_CSTATE[1])
              + quadrotor_X.q0q1q2q3_CSTATE[2] * quadrotor_X.q0q1q2q3_CSTATE[2])
    + quadrotor_X.q0q1q2q3_CSTATE[3] * quadrotor_X.q0q1q2q3_CSTATE[3];

  /* SignalConversion: '<S4>/TmpSignal ConversionAtq0 q1 q2 q3Inport1' incorporates:
   *  Constant: '<S15>/Constant'
   *  DotProduct: '<S15>/Dot Product'
   *  Fcn: '<S15>/q0dot'
   *  Fcn: '<S15>/q1dot'
   *  Fcn: '<S15>/q2dot'
   *  Fcn: '<S15>/q3dot'
   *  Integrator: '<S4>/q0 q1 q2 q3'
   *  Sum: '<S15>/Sum'
   */
  quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[0] =
    ((quadrotor_X.q0q1q2q3_CSTATE[1] * quadrotor_B.pqr[0] +
      quadrotor_X.q0q1q2q3_CSTATE[2] * quadrotor_B.pqr[1]) +
     quadrotor_X.q0q1q2q3_CSTATE[3] * quadrotor_B.pqr[2]) * -0.5 + (1.0 -
    rtb_fcn2) * quadrotor_X.q0q1q2q3_CSTATE[0];
  quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[1] =
    ((quadrotor_X.q0q1q2q3_CSTATE[0] * quadrotor_B.pqr[0] +
      quadrotor_X.q0q1q2q3_CSTATE[2] * quadrotor_B.pqr[2]) -
     quadrotor_X.q0q1q2q3_CSTATE[3] * quadrotor_B.pqr[1]) * 0.5 + (1.0 -
    rtb_fcn2) * quadrotor_X.q0q1q2q3_CSTATE[1];
  quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[2] =
    ((quadrotor_X.q0q1q2q3_CSTATE[0] * quadrotor_B.pqr[1] +
      quadrotor_X.q0q1q2q3_CSTATE[3] * quadrotor_B.pqr[0]) -
     quadrotor_X.q0q1q2q3_CSTATE[1] * quadrotor_B.pqr[2]) * 0.5 + (1.0 -
    rtb_fcn2) * quadrotor_X.q0q1q2q3_CSTATE[2];
  quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[3] =
    ((quadrotor_X.q0q1q2q3_CSTATE[0] * quadrotor_B.pqr[2] +
      quadrotor_X.q0q1q2q3_CSTATE[1] * quadrotor_B.pqr[1]) -
     quadrotor_X.q0q1q2q3_CSTATE[2] * quadrotor_B.pqr[0]) * 0.5 + (1.0 -
    rtb_fcn2) * quadrotor_X.q0q1q2q3_CSTATE[3];

  /* Sum: '<S47>/Add' */
  rtb_Product1_p = (quadrotor_B.VectorConcatenate[0] +
                    quadrotor_B.VectorConcatenate[4]) +
    quadrotor_B.VectorConcatenate[8];

  /* If: '<S2>/If' incorporates:
   *  Sum: '<S47>/Add'
   */
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    quadrotor_DW.If_ActiveSubsystem = (int8_T)(rtb_Product1_p <= 0.0);
  }

  switch (quadrotor_DW.If_ActiveSubsystem) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S2>/Positive Trace' incorporates:
     *  ActionPort: '<S45>/Action Port'
     */
    /* Sqrt: '<S45>/sqrt' incorporates:
     *  Constant: '<S45>/Constant'
     *  Sum: '<S45>/Sum'
     *  Sum: '<S47>/Add'
     */
    rtb_Product1_p = sqrt(rtb_Product1_p + 1.0);

    /* Gain: '<S45>/Gain' */
    quadrotor_B.Merge[0] = 0.5 * rtb_Product1_p;

    /* Gain: '<S45>/Gain1' */
    rtb_Product1_p *= 2.0;

    /* Product: '<S45>/Product' incorporates:
     *  Sum: '<S67>/Add'
     *  Sum: '<S68>/Add'
     *  Sum: '<S69>/Add'
     */
    quadrotor_B.Merge[1] = (quadrotor_B.VectorConcatenate[7] -
      quadrotor_B.VectorConcatenate[5]) / rtb_Product1_p;
    quadrotor_B.Merge[2] = (quadrotor_B.VectorConcatenate[2] -
      quadrotor_B.VectorConcatenate[6]) / rtb_Product1_p;
    quadrotor_B.Merge[3] = (quadrotor_B.VectorConcatenate[3] -
      quadrotor_B.VectorConcatenate[1]) / rtb_Product1_p;

    /* End of Outputs for SubSystem: '<S2>/Positive Trace' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S2>/Negative Trace' incorporates:
     *  ActionPort: '<S44>/Action Port'
     */
    /* If: '<S44>/Find Maximum Diagonal Value' */
    if (rtmIsMajorTimeStep(quadrotor_M)) {
      if ((quadrotor_B.VectorConcatenate[4] > quadrotor_B.VectorConcatenate[0]) &&
          (quadrotor_B.VectorConcatenate[4] > quadrotor_B.VectorConcatenate[8]))
      {
        quadrotor_DW.FindMaximumDiagonalValue_Active = 0;
      } else if (quadrotor_B.VectorConcatenate[8] >
                 quadrotor_B.VectorConcatenate[0]) {
        quadrotor_DW.FindMaximumDiagonalValue_Active = 1;
      } else {
        quadrotor_DW.FindMaximumDiagonalValue_Active = 2;
      }
    }

    switch (quadrotor_DW.FindMaximumDiagonalValue_Active) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S44>/Maximum Value at DCM(2,2)' incorporates:
       *  ActionPort: '<S49>/Action Port'
       */
      /* Sqrt: '<S49>/sqrt' incorporates:
       *  Constant: '<S61>/Constant'
       *  Sum: '<S61>/Add'
       */
      rtb_fcn4 = sqrt(((quadrotor_B.VectorConcatenate[4] -
                        quadrotor_B.VectorConcatenate[0]) -
                       quadrotor_B.VectorConcatenate[8]) + 1.0);

      /* Switch: '<S60>/Switch' incorporates:
       *  Constant: '<S60>/Constant1'
       */
      if (rtb_fcn4 != 0.0) {
        rtb_Product1_p = 0.5;
        rtb_Product2_h = rtb_fcn4;
      } else {
        rtb_Product1_p = 0.0;
        rtb_Product2_h = 1.0;
      }

      /* End of Switch: '<S60>/Switch' */

      /* Product: '<S60>/Product' */
      rtb_Product1_p /= rtb_Product2_h;

      /* Gain: '<S49>/Gain1' incorporates:
       *  Product: '<S49>/Product'
       *  Sum: '<S59>/Add'
       */
      quadrotor_B.Merge[1] = (quadrotor_B.VectorConcatenate[1] +
        quadrotor_B.VectorConcatenate[3]) * rtb_Product1_p;

      /* Gain: '<S49>/Gain3' incorporates:
       *  Product: '<S49>/Product'
       *  Sum: '<S58>/Add'
       */
      quadrotor_B.Merge[3] = (quadrotor_B.VectorConcatenate[5] +
        quadrotor_B.VectorConcatenate[7]) * rtb_Product1_p;

      /* Gain: '<S49>/Gain4' incorporates:
       *  Product: '<S49>/Product'
       *  Sum: '<S57>/Add'
       */
      quadrotor_B.Merge[0] = (quadrotor_B.VectorConcatenate[2] -
        quadrotor_B.VectorConcatenate[6]) * rtb_Product1_p;

      /* Gain: '<S49>/Gain' */
      quadrotor_B.Merge[2] = 0.5 * rtb_fcn4;

      /* End of Outputs for SubSystem: '<S44>/Maximum Value at DCM(2,2)' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S44>/Maximum Value at DCM(3,3)' incorporates:
       *  ActionPort: '<S50>/Action Port'
       */
      /* Sqrt: '<S50>/sqrt' incorporates:
       *  Constant: '<S66>/Constant'
       *  Sum: '<S66>/Add'
       */
      rtb_fcn4 = sqrt(((quadrotor_B.VectorConcatenate[8] -
                        quadrotor_B.VectorConcatenate[0]) -
                       quadrotor_B.VectorConcatenate[4]) + 1.0);

      /* Switch: '<S65>/Switch' incorporates:
       *  Constant: '<S65>/Constant1'
       */
      if (rtb_fcn4 != 0.0) {
        rtb_Product1_p = 0.5;
        rtb_Product2_h = rtb_fcn4;
      } else {
        rtb_Product1_p = 0.0;
        rtb_Product2_h = 1.0;
      }

      /* End of Switch: '<S65>/Switch' */

      /* Product: '<S65>/Product' */
      rtb_Product1_p /= rtb_Product2_h;

      /* Gain: '<S50>/Gain1' incorporates:
       *  Product: '<S50>/Product'
       *  Sum: '<S62>/Add'
       */
      quadrotor_B.Merge[1] = (quadrotor_B.VectorConcatenate[2] +
        quadrotor_B.VectorConcatenate[6]) * rtb_Product1_p;

      /* Gain: '<S50>/Gain2' incorporates:
       *  Product: '<S50>/Product'
       *  Sum: '<S63>/Add'
       */
      quadrotor_B.Merge[2] = (quadrotor_B.VectorConcatenate[5] +
        quadrotor_B.VectorConcatenate[7]) * rtb_Product1_p;

      /* Gain: '<S50>/Gain3' incorporates:
       *  Product: '<S50>/Product'
       *  Sum: '<S64>/Add'
       */
      quadrotor_B.Merge[0] = (quadrotor_B.VectorConcatenate[3] -
        quadrotor_B.VectorConcatenate[1]) * rtb_Product1_p;

      /* Gain: '<S50>/Gain' */
      quadrotor_B.Merge[3] = 0.5 * rtb_fcn4;

      /* End of Outputs for SubSystem: '<S44>/Maximum Value at DCM(3,3)' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S44>/Maximum Value at DCM(1,1)' incorporates:
       *  ActionPort: '<S48>/Action Port'
       */
      /* Sqrt: '<S48>/sqrt' incorporates:
       *  Constant: '<S56>/Constant'
       *  Sum: '<S56>/Add'
       */
      rtb_fcn4 = sqrt(((quadrotor_B.VectorConcatenate[0] -
                        quadrotor_B.VectorConcatenate[4]) -
                       quadrotor_B.VectorConcatenate[8]) + 1.0);

      /* Switch: '<S55>/Switch' incorporates:
       *  Constant: '<S55>/Constant1'
       */
      if (rtb_fcn4 != 0.0) {
        rtb_Product1_p = 0.5;
        rtb_Product2_h = rtb_fcn4;
      } else {
        rtb_Product1_p = 0.0;
        rtb_Product2_h = 1.0;
      }

      /* End of Switch: '<S55>/Switch' */

      /* Product: '<S55>/Product' */
      rtb_Product1_p /= rtb_Product2_h;

      /* Gain: '<S48>/Gain1' incorporates:
       *  Product: '<S48>/Product'
       *  Sum: '<S54>/Add'
       */
      quadrotor_B.Merge[2] = (quadrotor_B.VectorConcatenate[1] +
        quadrotor_B.VectorConcatenate[3]) * rtb_Product1_p;

      /* Gain: '<S48>/Gain2' incorporates:
       *  Product: '<S48>/Product'
       *  Sum: '<S52>/Add'
       */
      quadrotor_B.Merge[3] = (quadrotor_B.VectorConcatenate[2] +
        quadrotor_B.VectorConcatenate[6]) * rtb_Product1_p;

      /* Gain: '<S48>/Gain3' incorporates:
       *  Product: '<S48>/Product'
       *  Sum: '<S53>/Add'
       */
      quadrotor_B.Merge[0] = (quadrotor_B.VectorConcatenate[7] -
        quadrotor_B.VectorConcatenate[5]) * rtb_Product1_p;

      /* Gain: '<S48>/Gain' */
      quadrotor_B.Merge[1] = 0.5 * rtb_fcn4;

      /* End of Outputs for SubSystem: '<S44>/Maximum Value at DCM(1,1)' */
      break;
    }

    /* End of If: '<S44>/Find Maximum Diagonal Value' */
    /* End of Outputs for SubSystem: '<S2>/Negative Trace' */
    break;
  }

  /* End of If: '<S2>/If' */
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/q' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold7'
     */
    quadrotor_Y.q[0] = quadrotor_B.Merge[0];
    quadrotor_Y.q[1] = quadrotor_B.Merge[1];
    quadrotor_Y.q[2] = quadrotor_B.Merge[2];
    quadrotor_Y.q[3] = quadrotor_B.Merge[3];

    /* Outport: '<Root>/pqr' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold4'
     */
    quadrotor_Y.pqr[0] = quadrotor_B.pqr[0];
    quadrotor_Y.pqr[1] = quadrotor_B.pqr[1];
    quadrotor_Y.pqr[2] = quadrotor_B.pqr[2];
  }

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Constant: '<Root>/kq'
   *  Constant: '<Root>/kt'
   *  Product: '<S38>/Product'
   *  Product: '<S39>/Product'
   *  SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport1'
   */
  T[0] = quadrotor_B.StateSpace / 60.0 * 2.0 * 3.1415926535897931;
  T[1] = quadrotor_B.StateSpace1 / 60.0 * 2.0 * 3.1415926535897931;
  T[2] = quadrotor_B.StateSpace2 / 60.0 * 2.0 * 3.1415926535897931;
  T[3] = quadrotor_B.StateSpace3 / 60.0 * 2.0 * 3.1415926535897931;
  rtb_Product1_p = T[0] * T[0];
  T[0] = simparam_kt * rtb_Product1_p;
  rtb_Product2_h = simparam_kq * rtb_Product1_p;
  rtb_Product1_p = T[1] * T[1];
  T[1] = simparam_kt * rtb_Product1_p;
  rtb_fcn4 = simparam_kq * rtb_Product1_p;
  rtb_Product1_p = T[2] * T[2];
  T[2] = simparam_kt * rtb_Product1_p;
  Q_idx_2 = simparam_kq * rtb_Product1_p;
  rtb_Product1_p = T[3] * T[3];
  VectorConcatenate_tmp_1 = simparam_kt * rtb_Product1_p;
  rtb_Product1_p *= simparam_kq;
  T[3] = VectorConcatenate_tmp_1;
  VectorConcatenate_tmp_2 = T[0];
  for (iU = 0; iU < 3; iU++) {
    VectorConcatenate_tmp_2 += T[iU + 1];
    rtb_sincos_o1[iU] = quadrotor_B.Selector[iU + 6] * quadrotor_B.pqr[2] +
      (quadrotor_B.Selector[iU + 3] * quadrotor_B.pqr[1] +
       quadrotor_B.Selector[iU] * quadrotor_B.pqr[0]);
    tmp[iU] = quadrotor_B.Selector1[iU + 6] * quadrotor_B.pqr[2] +
      (quadrotor_B.Selector1[iU + 3] * quadrotor_B.pqr[1] +
       quadrotor_B.Selector1[iU] * quadrotor_B.pqr[0]);
  }

  /* Sum: '<S5>/Sum2' incorporates:
   *  Constant: '<Root>/r'
   *  MATLAB Function: '<Root>/MATLAB Function'
   *  Product: '<S39>/Product'
   *  Product: '<S40>/i x j'
   *  Product: '<S40>/j x k'
   *  Product: '<S40>/k x i'
   *  Product: '<S41>/i x k'
   *  Product: '<S41>/j x i'
   *  Product: '<S41>/k x j'
   *  Sum: '<S37>/Sum'
   */
  rtb_sincos_o2[0] = ((-T[1] + VectorConcatenate_tmp_1) * simparam_r - tmp[0]) -
    (quadrotor_B.pqr[1] * rtb_sincos_o1[2] - quadrotor_B.pqr[2] * rtb_sincos_o1
     [1]);
  rtb_sincos_o2[1] = ((T[0] - T[2]) * simparam_r - tmp[1]) - (quadrotor_B.pqr[2]
    * rtb_sincos_o1[0] - quadrotor_B.pqr[0] * rtb_sincos_o1[2]);
  rtb_sincos_o2[2] = ((((-rtb_Product2_h + rtb_fcn4) - Q_idx_2) + rtb_Product1_p)
                      - tmp[2]) - (quadrotor_B.pqr[0] * rtb_sincos_o1[1] -
    quadrotor_B.pqr[1] * rtb_sincos_o1[0]);

  /* Product: '<S5>/Product2' */
  rt_mrdivide_U1d1x3_U2d3x3_Yd1x3(rtb_sincos_o2, quadrotor_B.Selector2,
    quadrotor_B.Product2);

  /* Integrator: '<S1>/ub,vb,wb' */
  quadrotor_B.ubvbwb[0] = quadrotor_X.ubvbwb_CSTATE[0];
  quadrotor_B.ubvbwb[1] = quadrotor_X.ubvbwb_CSTATE[1];
  quadrotor_B.ubvbwb[2] = quadrotor_X.ubvbwb_CSTATE[2];

  /* Sum: '<S1>/Sum' incorporates:
   *  Constant: '<Root>/g'
   *  Constant: '<Root>/m'
   *  Constant: '<S6>/Constant'
   *  MATLAB Function: '<Root>/MATLAB Function'
   *  Product: '<Root>/Product'
   *  Product: '<S1>/Product'
   *  Product: '<S42>/i x j'
   *  Product: '<S42>/j x k'
   *  Product: '<S42>/k x i'
   *  Product: '<S43>/i x k'
   *  Product: '<S43>/j x i'
   *  Product: '<S43>/k x j'
   *  Selector: '<Root>/Col3'
   *  Sum: '<Root>/Add'
   *  Sum: '<S7>/Sum'
   */
  quadrotor_B.Sum[0] = quadrotor_B.VectorConcatenate[6] * simparam_m *
    simparam_g / simparam_m + (quadrotor_B.ubvbwb[1] * quadrotor_B.pqr[2] -
    quadrotor_B.ubvbwb[2] * quadrotor_B.pqr[1]);
  quadrotor_B.Sum[1] = quadrotor_B.VectorConcatenate[7] * simparam_m *
    simparam_g / simparam_m + (quadrotor_B.ubvbwb[2] * quadrotor_B.pqr[0] -
    quadrotor_B.ubvbwb[0] * quadrotor_B.pqr[2]);
  quadrotor_B.Sum[2] = (quadrotor_B.VectorConcatenate[8] * simparam_m *
                        simparam_g + -VectorConcatenate_tmp_2) / simparam_m +
    (quadrotor_B.ubvbwb[0] * quadrotor_B.pqr[1] - quadrotor_B.ubvbwb[1] *
     quadrotor_B.pqr[0]);
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/pqr_dot' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold5'
     */
    quadrotor_Y.pqr_dot[0] = quadrotor_B.Product2[0];
    quadrotor_Y.pqr_dot[1] = quadrotor_B.Product2[1];
    quadrotor_Y.pqr_dot[2] = quadrotor_B.Product2[2];

    /* Outport: '<Root>/uvw_dot' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold6'
     */
    quadrotor_Y.uvw_dot[0] = quadrotor_B.Sum[0];

    /* Outport: '<Root>/uvw' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold3'
     */
    quadrotor_Y.uvw[0] = quadrotor_B.ubvbwb[0];

    /* Outport: '<Root>/uvw_dot' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold6'
     */
    quadrotor_Y.uvw_dot[1] = quadrotor_B.Sum[1];

    /* Outport: '<Root>/uvw' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold3'
     */
    quadrotor_Y.uvw[1] = quadrotor_B.ubvbwb[1];

    /* Outport: '<Root>/uvw_dot' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold6'
     */
    quadrotor_Y.uvw_dot[2] = quadrotor_B.Sum[2];

    /* Outport: '<Root>/uvw' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold3'
     */
    quadrotor_Y.uvw[2] = quadrotor_B.ubvbwb[2];
  }

  /* Outputs for IfAction SubSystem: '<S46>/If Warning//Error' incorporates:
   *  ActionPort: '<S70>/if'
   */
  for (iU = 0; iU < 3; iU++) {
    /* If: '<S46>/If1' incorporates:
     *  Math: '<S1>/Transpose'
     *  Math: '<S73>/Math Function'
     */
    Product_tmp[3 * iU] = quadrotor_B.VectorConcatenate[iU];
    Product_tmp[1 + 3 * iU] = quadrotor_B.VectorConcatenate[iU + 3];
    Product_tmp[2 + 3 * iU] = quadrotor_B.VectorConcatenate[iU + 6];
  }

  /* End of Outputs for SubSystem: '<S46>/If Warning//Error' */

  /* Product: '<S11>/Product' incorporates:
   *  Math: '<S1>/Transpose'
   */
  for (iU = 0; iU < 3; iU++) {
    quadrotor_B.Product[iU] = 0.0;
    quadrotor_B.Product[iU] += Product_tmp[iU] * quadrotor_B.ubvbwb[0];
    quadrotor_B.Product[iU] += Product_tmp[iU + 3] * quadrotor_B.ubvbwb[1];
    quadrotor_B.Product[iU] += Product_tmp[iU + 6] * quadrotor_B.ubvbwb[2];
  }

  /* End of Product: '<S11>/Product' */
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/uvw_earth' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold'
     */
    quadrotor_Y.uvw_earth[0] = quadrotor_B.Product[0];
    quadrotor_Y.uvw_earth[1] = quadrotor_B.Product[1];
    quadrotor_Y.uvw_earth[2] = quadrotor_B.Product[2];
  }

  /* Sqrt: '<S35>/sqrt' */
  rtb_fcn4 = sqrt(rtb_fcn2);

  /* Product: '<S30>/Product' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  Q_idx_2 = quadrotor_X.q0q1q2q3_CSTATE[0] / rtb_fcn4;

  /* Product: '<S30>/Product1' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_Product1_p = quadrotor_X.q0q1q2q3_CSTATE[1] / rtb_fcn4;

  /* Product: '<S30>/Product2' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_Product2_h = quadrotor_X.q0q1q2q3_CSTATE[2] / rtb_fcn4;

  /* Product: '<S30>/Product3' incorporates:
   *  Integrator: '<S4>/q0 q1 q2 q3'
   */
  rtb_fcn4 = quadrotor_X.q0q1q2q3_CSTATE[3] / rtb_fcn4;

  /* Fcn: '<S13>/fcn2' incorporates:
   *  Fcn: '<S13>/fcn5'
   */
  VectorConcatenate_tmp_1 = Q_idx_2 * Q_idx_2;
  VectorConcatenate_tmp_2 = rtb_Product1_p * rtb_Product1_p;
  VectorConcatenate_tmp_3 = rtb_Product2_h * rtb_Product2_h;
  VectorConcatenate_tmp = rtb_fcn4 * rtb_fcn4;

  /* Trigonometry: '<S29>/Trigonometric Function1' incorporates:
   *  Fcn: '<S13>/fcn1'
   *  Fcn: '<S13>/fcn2'
   */
  quadrotor_B.VectorConcatenate_c[0] = atan2((rtb_Product1_p * rtb_Product2_h +
    Q_idx_2 * rtb_fcn4) * 2.0, ((VectorConcatenate_tmp_1 +
    VectorConcatenate_tmp_2) - VectorConcatenate_tmp_3) - VectorConcatenate_tmp);

  /* Fcn: '<S13>/fcn3' */
  rtb_fcn2 = (rtb_Product1_p * rtb_fcn4 - Q_idx_2 * rtb_Product2_h) * -2.0;

  /* If: '<S31>/If' incorporates:
   *  Constant: '<S32>/Constant'
   *  Constant: '<S33>/Constant'
   *  Inport: '<S34>/In'
   */
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    if (rtb_fcn2 > 1.0) {
      quadrotor_DW.If_ActiveSubsystem_h = 0;
    } else if (rtb_fcn2 < -1.0) {
      quadrotor_DW.If_ActiveSubsystem_h = 1;
    } else {
      quadrotor_DW.If_ActiveSubsystem_h = 2;
    }
  }

  switch (quadrotor_DW.If_ActiveSubsystem_h) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S31>/If Action Subsystem' incorporates:
     *  ActionPort: '<S32>/Action Port'
     */
    if (rtmIsMajorTimeStep(quadrotor_M)) {
      quadrotor_B.Merge_m = 1.0;
    }

    /* End of Outputs for SubSystem: '<S31>/If Action Subsystem' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S31>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S33>/Action Port'
     */
    if (rtmIsMajorTimeStep(quadrotor_M)) {
      quadrotor_B.Merge_m = 1.0;
    }

    /* End of Outputs for SubSystem: '<S31>/If Action Subsystem1' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S31>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S34>/Action Port'
     */
    quadrotor_B.Merge_m = rtb_fcn2;

    /* End of Outputs for SubSystem: '<S31>/If Action Subsystem2' */
    break;
  }

  /* End of If: '<S31>/If' */

  /* Trigonometry: '<S29>/trigFcn' */
  if (quadrotor_B.Merge_m > 1.0) {
    rtb_fcn2 = 1.0;
  } else if (quadrotor_B.Merge_m < -1.0) {
    rtb_fcn2 = -1.0;
  } else {
    rtb_fcn2 = quadrotor_B.Merge_m;
  }

  quadrotor_B.VectorConcatenate_c[1] = asin(rtb_fcn2);

  /* End of Trigonometry: '<S29>/trigFcn' */

  /* Trigonometry: '<S29>/Trigonometric Function3' incorporates:
   *  Fcn: '<S13>/fcn4'
   *  Fcn: '<S13>/fcn5'
   */
  quadrotor_B.VectorConcatenate_c[2] = atan2((rtb_Product2_h * rtb_fcn4 +
    Q_idx_2 * rtb_Product1_p) * 2.0, ((VectorConcatenate_tmp_1 -
    VectorConcatenate_tmp_2) - VectorConcatenate_tmp_3) + VectorConcatenate_tmp);
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/zeta' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold2'
     */
    quadrotor_Y.zeta[0] = quadrotor_B.VectorConcatenate_c[2];
    quadrotor_Y.zeta[1] = quadrotor_B.VectorConcatenate_c[1];
    quadrotor_Y.zeta[2] = quadrotor_B.VectorConcatenate_c[0];

    /* If: '<S46>/If1' */
    if (rtmIsMajorTimeStep(quadrotor_M)) {
      quadrotor_DW.If1_ActiveSubsystem = -1;
    }
  }

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/rpm_cmd'
   */
  if (quadrotor_U.rpm_cmd[0] > simparam_rpm_max) {
    quadrotor_B.Saturation[0] = simparam_rpm_max;
  } else if (quadrotor_U.rpm_cmd[0] < simparam_rpm_min) {
    quadrotor_B.Saturation[0] = simparam_rpm_min;
  } else {
    quadrotor_B.Saturation[0] = quadrotor_U.rpm_cmd[0];
  }

  if (quadrotor_U.rpm_cmd[1] > simparam_rpm_max) {
    quadrotor_B.Saturation[1] = simparam_rpm_max;
  } else if (quadrotor_U.rpm_cmd[1] < simparam_rpm_min) {
    quadrotor_B.Saturation[1] = simparam_rpm_min;
  } else {
    quadrotor_B.Saturation[1] = quadrotor_U.rpm_cmd[1];
  }

  if (quadrotor_U.rpm_cmd[2] > simparam_rpm_max) {
    quadrotor_B.Saturation[2] = simparam_rpm_max;
  } else if (quadrotor_U.rpm_cmd[2] < simparam_rpm_min) {
    quadrotor_B.Saturation[2] = simparam_rpm_min;
  } else {
    quadrotor_B.Saturation[2] = quadrotor_U.rpm_cmd[2];
  }

  if (quadrotor_U.rpm_cmd[3] > simparam_rpm_max) {
    quadrotor_B.Saturation[3] = simparam_rpm_max;
  } else if (quadrotor_U.rpm_cmd[3] < simparam_rpm_min) {
    quadrotor_B.Saturation[3] = simparam_rpm_min;
  } else {
    quadrotor_B.Saturation[3] = quadrotor_U.rpm_cmd[3];
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Integrator: '<S1>/xe,ye,ze' */
  quadrotor_B.xeyeze[0] = quadrotor_X.xeyeze_CSTATE[0];
  quadrotor_B.xeyeze[1] = quadrotor_X.xeyeze_CSTATE[1];
  quadrotor_B.xeyeze[2] = quadrotor_X.xeyeze_CSTATE[2];
  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Outport: '<Root>/xyz' incorporates:
     *  ZeroOrderHold: '<Root>/Zero-Order Hold1'
     */
    quadrotor_Y.xyz[0] = quadrotor_B.xeyeze[0];
    quadrotor_Y.xyz[1] = quadrotor_B.xeyeze[1];
    quadrotor_Y.xyz[2] = quadrotor_B.xeyeze[2];
  }

  if (rtmIsMajorTimeStep(quadrotor_M)) {
    /* Update for Integrator: '<S4>/q0 q1 q2 q3' */
    quadrotor_DW.q0q1q2q3_IWORK = 0;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(quadrotor_M)) {
    rt_ertODEUpdateContinuousStates(&quadrotor_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++quadrotor_M->Timing.clockTick0)) {
      ++quadrotor_M->Timing.clockTickH0;
    }

    quadrotor_M->Timing.t[0] = rtsiGetSolverStopTime(&quadrotor_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      quadrotor_M->Timing.clockTick1++;
      if (!quadrotor_M->Timing.clockTick1) {
        quadrotor_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void quadrotor_derivatives(void)
{
  XDot_quadrotor_T *_rtXdot;
  _rtXdot = ((XDot_quadrotor_T *) quadrotor_M->derivs);

  /* Derivatives for Integrator: '<S4>/q0 q1 q2 q3' */
  _rtXdot->q0q1q2q3_CSTATE[0] = quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[0];
  _rtXdot->q0q1q2q3_CSTATE[1] = quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[1];
  _rtXdot->q0q1q2q3_CSTATE[2] = quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[2];
  _rtXdot->q0q1q2q3_CSTATE[3] = quadrotor_B.TmpSignalConversionAtq0q1q2q3_g[3];

  /* Derivatives for StateSpace: '<Root>/State-Space' */
  _rtXdot->StateSpace_CSTATE = 0.0;
  _rtXdot->StateSpace_CSTATE += -1.0 / simparam_tau *
    quadrotor_X.StateSpace_CSTATE;
  _rtXdot->StateSpace_CSTATE += 1.0 / simparam_tau * quadrotor_B.Saturation[0];

  /* Derivatives for StateSpace: '<Root>/State-Space1' */
  _rtXdot->StateSpace1_CSTATE = 0.0;
  _rtXdot->StateSpace1_CSTATE += -1.0 / simparam_tau *
    quadrotor_X.StateSpace1_CSTATE;
  _rtXdot->StateSpace1_CSTATE += 1.0 / simparam_tau * quadrotor_B.Saturation[1];

  /* Derivatives for StateSpace: '<Root>/State-Space2' */
  _rtXdot->StateSpace2_CSTATE = 0.0;
  _rtXdot->StateSpace2_CSTATE += -1.0 / simparam_tau *
    quadrotor_X.StateSpace2_CSTATE;
  _rtXdot->StateSpace2_CSTATE += 1.0 / simparam_tau * quadrotor_B.Saturation[2];

  /* Derivatives for StateSpace: '<Root>/State-Space3' */
  _rtXdot->StateSpace3_CSTATE = 0.0;
  _rtXdot->StateSpace3_CSTATE += -1.0 / simparam_tau *
    quadrotor_X.StateSpace3_CSTATE;
  _rtXdot->StateSpace3_CSTATE += 1.0 / simparam_tau * quadrotor_B.Saturation[3];

  /* Derivatives for Integrator: '<S1>/p,q,r ' */
  _rtXdot->pqr_CSTATE[0] = quadrotor_B.Product2[0];

  /* Derivatives for Integrator: '<S1>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[0] = quadrotor_B.Sum[0];

  /* Derivatives for Integrator: '<S1>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[0] = quadrotor_B.Product[0];

  /* Derivatives for Integrator: '<S1>/p,q,r ' */
  _rtXdot->pqr_CSTATE[1] = quadrotor_B.Product2[1];

  /* Derivatives for Integrator: '<S1>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[1] = quadrotor_B.Sum[1];

  /* Derivatives for Integrator: '<S1>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[1] = quadrotor_B.Product[1];

  /* Derivatives for Integrator: '<S1>/p,q,r ' */
  _rtXdot->pqr_CSTATE[2] = quadrotor_B.Product2[2];

  /* Derivatives for Integrator: '<S1>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[2] = quadrotor_B.Sum[2];

  /* Derivatives for Integrator: '<S1>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[2] = quadrotor_B.Product[2];
}

/* Model initialize function */
void quadrotor_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)quadrotor_M, 0,
                sizeof(RT_MODEL_quadrotor_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&quadrotor_M->solverInfo,
                          &quadrotor_M->Timing.simTimeStep);
    rtsiSetTPtr(&quadrotor_M->solverInfo, &rtmGetTPtr(quadrotor_M));
    rtsiSetStepSizePtr(&quadrotor_M->solverInfo, &quadrotor_M->Timing.stepSize0);
    rtsiSetdXPtr(&quadrotor_M->solverInfo, &quadrotor_M->derivs);
    rtsiSetContStatesPtr(&quadrotor_M->solverInfo, (real_T **)
                         &quadrotor_M->contStates);
    rtsiSetNumContStatesPtr(&quadrotor_M->solverInfo,
      &quadrotor_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&quadrotor_M->solverInfo,
      &quadrotor_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&quadrotor_M->solverInfo,
      &quadrotor_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&quadrotor_M->solverInfo,
      &quadrotor_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&quadrotor_M->solverInfo, (&rtmGetErrorStatus
      (quadrotor_M)));
    rtsiSetRTModelPtr(&quadrotor_M->solverInfo, quadrotor_M);
  }

  rtsiSetSimTimeStep(&quadrotor_M->solverInfo, MAJOR_TIME_STEP);
  quadrotor_M->intgData.y = quadrotor_M->odeY;
  quadrotor_M->intgData.f[0] = quadrotor_M->odeF[0];
  quadrotor_M->intgData.f[1] = quadrotor_M->odeF[1];
  quadrotor_M->intgData.f[2] = quadrotor_M->odeF[2];
  quadrotor_M->intgData.f[3] = quadrotor_M->odeF[3];
  quadrotor_M->contStates = ((X_quadrotor_T *) &quadrotor_X);
  rtsiSetSolverData(&quadrotor_M->solverInfo, (void *)&quadrotor_M->intgData);
  rtsiSetSolverName(&quadrotor_M->solverInfo,"ode4");
  rtmSetTPtr(quadrotor_M, &quadrotor_M->Timing.tArray[0]);
  quadrotor_M->Timing.stepSize0 = 0.01;
  rtmSetFirstInitCond(quadrotor_M, 1);

  /* block I/O */
  (void) memset(((void *) &quadrotor_B), 0,
                sizeof(B_quadrotor_T));

  /* states (continuous) */
  {
    (void) memset((void *)&quadrotor_X, 0,
                  sizeof(X_quadrotor_T));
  }

  /* states (dwork) */
  (void) memset((void *)&quadrotor_DW, 0,
                sizeof(DW_quadrotor_T));

  /* external inputs */
  (void)memset(&quadrotor_U, 0, sizeof(ExtU_quadrotor_T));

  /* external outputs */
  (void) memset((void *)&quadrotor_Y, 0,
                sizeof(ExtY_quadrotor_T));

  /* Start for If: '<S2>/If' */
  quadrotor_DW.If_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S2>/Negative Trace' */
  /* Start for If: '<S44>/Find Maximum Diagonal Value' */
  quadrotor_DW.FindMaximumDiagonalValue_Active = -1;

  /* End of Start for SubSystem: '<S2>/Negative Trace' */

  /* Start for If: '<S31>/If' */
  quadrotor_DW.If_ActiveSubsystem_h = -1;

  /* Start for If: '<S46>/If1' */
  quadrotor_DW.If1_ActiveSubsystem = -1;

  /* InitializeConditions for Integrator: '<S4>/q0 q1 q2 q3' */
  if (rtmIsFirstInitCond(quadrotor_M)) {
    quadrotor_X.q0q1q2q3_CSTATE[0] = 0.0;
    quadrotor_X.q0q1q2q3_CSTATE[1] = 0.0;
    quadrotor_X.q0q1q2q3_CSTATE[2] = 0.0;
    quadrotor_X.q0q1q2q3_CSTATE[3] = 0.0;
  }

  quadrotor_DW.q0q1q2q3_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S4>/q0 q1 q2 q3' */

  /* InitializeConditions for StateSpace: '<Root>/State-Space' */
  quadrotor_X.StateSpace_CSTATE = simparam_init_rpm[0];

  /* InitializeConditions for StateSpace: '<Root>/State-Space1' */
  quadrotor_X.StateSpace1_CSTATE = simparam_init_rpm[1];

  /* InitializeConditions for StateSpace: '<Root>/State-Space2' */
  quadrotor_X.StateSpace2_CSTATE = simparam_init_rpm[2];

  /* InitializeConditions for StateSpace: '<Root>/State-Space3' */
  quadrotor_X.StateSpace3_CSTATE = simparam_init_rpm[3];

  quadrotor_Y.rpm[0] = simparam_init_rpm[0];
  quadrotor_Y.rpm[1] = simparam_init_rpm[1];
  quadrotor_Y.rpm[2] = simparam_init_rpm[2];
  quadrotor_Y.rpm[3] = simparam_init_rpm[3];

  /* InitializeConditions for Integrator: '<S1>/p,q,r ' */
  quadrotor_X.pqr_CSTATE[0] = simparam_init_omega[0];

  /* InitializeConditions for Integrator: '<S1>/ub,vb,wb' */
  quadrotor_X.ubvbwb_CSTATE[0] = simparam_init_vel[0];

  /* InitializeConditions for Integrator: '<S1>/xe,ye,ze' */
  quadrotor_X.xeyeze_CSTATE[0] = simparam_init_pos[0];

  /* InitializeConditions for Integrator: '<S1>/p,q,r ' */
  quadrotor_X.pqr_CSTATE[1] = simparam_init_omega[1];

  /* InitializeConditions for Integrator: '<S1>/ub,vb,wb' */
  quadrotor_X.ubvbwb_CSTATE[1] = simparam_init_vel[1];

  /* InitializeConditions for Integrator: '<S1>/xe,ye,ze' */
  quadrotor_X.xeyeze_CSTATE[1] = simparam_init_pos[1];

  /* InitializeConditions for Integrator: '<S1>/p,q,r ' */
  quadrotor_X.pqr_CSTATE[2] = simparam_init_omega[2];

  /* InitializeConditions for Integrator: '<S1>/ub,vb,wb' */
  quadrotor_X.ubvbwb_CSTATE[2] = simparam_init_vel[2];

  /* InitializeConditions for Integrator: '<S1>/xe,ye,ze' */
  quadrotor_X.xeyeze_CSTATE[2] = simparam_init_pos[2];

  /* SystemInitialize for Merge: '<S2>/Merge' */
  quadrotor_B.Merge[0] = 1.0;
  quadrotor_B.Merge[1] = 0.0;
  quadrotor_B.Merge[2] = 0.0;
  quadrotor_B.Merge[3] = 0.0;

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(quadrotor_M)) {
    rtmSetFirstInitCond(quadrotor_M, 0);
  }
}

/* Model terminate function */
void quadrotor_terminate(void)
{
  /* (no terminate code required) */
}