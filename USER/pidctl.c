/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pidctl.c
 *
 * Code generated for Simulink model 'pidctl'.
 *
 * Model version                  : 2.8
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Fri Apr 12 16:48:25 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "pidctl.h"
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<S18>/Chart1' */
#define IN_ADD                         ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_SUB                         ((uint8_T)3U)

/* Named constants for Chart: '<S18>/Chart2' */
#define IN_DB                          ((uint8_T)1U)
#define IN_NDB                         ((uint8_T)2U)

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void IfAction_speedSubsystem(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T *rty_Out1, DW_IfAction_speedSubsystem *localDW);
static void Chart1_Init(real_T *rty_u2);
static void Chart1(real_T rtu_u1, real_T *rty_u2, DW_Chart1 *localDW);
static void Chart2_Init(real_T *rty_op);
static void Chart2(real_T rtu_in, real_T rtu_noabsin, real_T *rty_op, DW_Chart2 *
                   localDW);
static void IfAction_speedSubsystem1_Init(DW_IfAction_speedSubsystem1 *localDW);
static void IfAction_speedSubsystem1(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T rtu_yaw_ecd, real_T rtu_yaw_last_ecd, real_T
  rtu_yaw_circle, real_T *rty_Out1, real_T *rty_yaw_circle_out,
  DW_IfAction_speedSubsystem1 *localDW);
static void IfAction_speedSubsystem_l(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T *rty_Out1, DW_IfAction_speedSubsystem_a *localDW);
static void IfAction_speedSubsystem1_m_Init(DW_IfAction_speedSubsystem1_j
  *localDW);
static void IfAction_speedSubsystem1_c(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T rtu_yaw_ecd, real_T rtu_yaw_last_ecd, real_T
  rtu_yaw_circle, real_T *rty_Out1, real_T *rty_yaw_circle_out,
  DW_IfAction_speedSubsystem1_j *localDW);

/*
 * Output and update for action system:
 *    '<S1>/If Action_speed Subsystem'
 *    '<S2>/If Action_speed Subsystem'
 *    '<S9>/If Action_speed Subsystem'
 *    '<S10>/If Action_speed Subsystem'
 *    '<S11>/If Action_speed Subsystem'
 *    '<S12>/If Action_speed Subsystem'
 *    '<S13>/If Action_speed Subsystem'
 *    '<S14>/If Action_speed Subsystem'
 */
static void IfAction_speedSubsystem(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T *rty_Out1, DW_IfAction_speedSubsystem *localDW)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_Sum;
  real_T u0;

  /* Sum: '<S17>/Sum' */
  rtb_Sum = rtu_yaw_target - rtu_yaw_speed_rpm;

  /* Gain: '<S54>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S46>/Filter'
   *  Gain: '<S45>/Derivative Gain'
   *  Sum: '<S46>/SumD'
   */
  rtb_FilterCoefficient = (rtP.Kd3508 * rtb_Sum - localDW->Filter_DSTATE) *
    100.0;

  /* Sum: '<S60>/Sum' incorporates:
   *  DiscreteIntegrator: '<S51>/Integrator'
   *  Gain: '<S56>/Proportional Gain'
   */
  u0 = (rtP.Kp3508 * rtb_Sum + localDW->Integrator_DSTATE) +
    rtb_FilterCoefficient;

  /* Saturate: '<S58>/Saturation' */
  if (u0 > 16384.0) {
    *rty_Out1 = 16384.0;
  } else if (u0 < -16384.0) {
    *rty_Out1 = -16384.0;
  } else {
    *rty_Out1 = u0;
  }

  /* End of Saturate: '<S58>/Saturation' */

  /* Update for DiscreteIntegrator: '<S46>/Filter' */
  localDW->Filter_DSTATE += 0.001 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S51>/Integrator' incorporates:
   *  Gain: '<S48>/Integral Gain'
   */
  localDW->Integrator_DSTATE += rtP.Ki3508 * rtb_Sum * 0.001;
}

/*
 * System initialize for atomic system:
 *    '<S18>/Chart1'
 *    '<S169>/Chart1'
 *    '<S320>/Chart1'
 *    '<S471>/Chart1'
 *    '<S622>/Chart1'
 *    '<S773>/Chart1'
 *    '<S924>/Chart1'
 *    '<S1075>/Chart1'
 *    '<S1226>/Chart1'
 *    '<S1377>/Chart1'
 *    ...
 */
static void Chart1_Init(real_T *rty_u2)
{
  *rty_u2 = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S18>/Chart1'
 *    '<S169>/Chart1'
 *    '<S320>/Chart1'
 *    '<S471>/Chart1'
 *    '<S622>/Chart1'
 *    '<S773>/Chart1'
 *    '<S924>/Chart1'
 *    '<S1075>/Chart1'
 *    '<S1226>/Chart1'
 *    '<S1377>/Chart1'
 *    ...
 */
static void Chart1(real_T rtu_u1, real_T *rty_u2, DW_Chart1 *localDW)
{
  /* Chart: '<S18>/Chart1' */
  if (localDW->is_active_c3_pidctl == 0U) {
    localDW->is_active_c3_pidctl = 1U;
    localDW->is_c3_pidctl = IN_IDLE;
  } else {
    switch (localDW->is_c3_pidctl) {
     case IN_ADD:
      if (rtu_u1 <= -5000.0) {
        localDW->is_c3_pidctl = IN_SUB;
        *rty_u2 = -1.0;
      } else if ((rtu_u1 < 5000.0) && (rtu_u1 > -5000.0)) {
        localDW->is_c3_pidctl = IN_IDLE;
      }
      break;

     case IN_IDLE:
      if (rtu_u1 >= 5000.0) {
        localDW->is_c3_pidctl = IN_ADD;
        *rty_u2 = 1.0;
      } else if (rtu_u1 <= -5000.0) {
        localDW->is_c3_pidctl = IN_SUB;
        *rty_u2 = -1.0;
      } else {
        *rty_u2 = 0.0;
      }
      break;

     default:
      /* case IN_SUB: */
      if (rtu_u1 >= 5000.0) {
        localDW->is_c3_pidctl = IN_ADD;
        *rty_u2 = 1.0;
      } else if ((rtu_u1 < 5000.0) && (rtu_u1 > -5000.0)) {
        localDW->is_c3_pidctl = IN_IDLE;
      }
      break;
    }
  }

  /* End of Chart: '<S18>/Chart1' */
}

/*
 * System initialize for atomic system:
 *    '<S18>/Chart2'
 *    '<S169>/Chart2'
 *    '<S320>/Chart2'
 *    '<S471>/Chart2'
 *    '<S622>/Chart2'
 *    '<S773>/Chart2'
 *    '<S924>/Chart2'
 *    '<S1075>/Chart2'
 *    '<S1226>/Chart2'
 *    '<S1377>/Chart2'
 *    ...
 */
static void Chart2_Init(real_T *rty_op)
{
  *rty_op = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S18>/Chart2'
 *    '<S169>/Chart2'
 *    '<S320>/Chart2'
 *    '<S471>/Chart2'
 *    '<S622>/Chart2'
 *    '<S773>/Chart2'
 *    '<S924>/Chart2'
 *    '<S1075>/Chart2'
 *    '<S1226>/Chart2'
 *    '<S1377>/Chart2'
 *    ...
 */
static void Chart2(real_T rtu_in, real_T rtu_noabsin, real_T *rty_op, DW_Chart2 *
                   localDW)
{
  /* Chart: '<S18>/Chart2' */
  if (localDW->is_active_c4_pidctl == 0U) {
    localDW->is_active_c4_pidctl = 1U;
    localDW->is_c4_pidctl = IN_DB;
  } else if (localDW->is_c4_pidctl == IN_DB) {
    if (rtu_in >= rtP.DeadBand) {
      localDW->is_c4_pidctl = IN_NDB;
    } else {
      *rty_op = 0.0;
    }

    /* case IN_NDB: */
  } else if (rtu_in <= rtP.DeadBand) {
    localDW->is_c4_pidctl = IN_DB;
  } else {
    *rty_op = rtu_noabsin;
  }

  /* End of Chart: '<S18>/Chart2' */
}

/*
 * System initialize for action system:
 *    '<S1>/If Action_speed Subsystem1'
 *    '<S2>/If Action_speed Subsystem1'
 *    '<S9>/If Action_speed Subsystem1'
 *    '<S10>/If Action_speed Subsystem1'
 *    '<S11>/If Action_speed Subsystem1'
 *    '<S12>/If Action_speed Subsystem1'
 *    '<S13>/If Action_speed Subsystem1'
 *    '<S14>/If Action_speed Subsystem1'
 */
static void IfAction_speedSubsystem1_Init(DW_IfAction_speedSubsystem1 *localDW)
{
  real_T op;

  /* SystemInitialize for Chart: '<S18>/Chart1' */
  Chart1_Init(&localDW->u2);

  /* SystemInitialize for Chart: '<S18>/Chart2' */
  Chart2_Init(&op);
}

/*
 * Output and update for action system:
 *    '<S1>/If Action_speed Subsystem1'
 *    '<S2>/If Action_speed Subsystem1'
 *    '<S9>/If Action_speed Subsystem1'
 *    '<S10>/If Action_speed Subsystem1'
 *    '<S11>/If Action_speed Subsystem1'
 *    '<S12>/If Action_speed Subsystem1'
 *    '<S13>/If Action_speed Subsystem1'
 *    '<S14>/If Action_speed Subsystem1'
 */
static void IfAction_speedSubsystem1(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T rtu_yaw_ecd, real_T rtu_yaw_last_ecd, real_T
  rtu_yaw_circle, real_T *rty_Out1, real_T *rty_yaw_circle_out,
  DW_IfAction_speedSubsystem1 *localDW)
{
  real_T op;
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_d;
  real_T rtb_Filter_kr;
  real_T rtb_Switch1;

  /* Sum: '<S18>/Sum' incorporates:
   *  Gain: '<S18>/Gain'
   *  Sum: '<S18>/Sum2'
   */
  rtb_Filter_kr = rtu_yaw_target - (8191.0 * rtu_yaw_circle + rtu_yaw_ecd);

  /* Abs: '<S18>/Abs' */
  rtb_Switch1 = fabs(rtb_Filter_kr);

  /* Switch: '<S18>/Switch2' incorporates:
   *  Constant: '<S18>/Constant'
   */
  if (!(rtb_Switch1 > rtP.DeadBand)) {
    rtb_Filter_kr = 0.0;
  }

  /* End of Switch: '<S18>/Switch2' */

  /* Gain: '<S154>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S146>/Filter'
   *  Gain: '<S145>/Derivative Gain'
   *  Sum: '<S146>/SumD'
   */
  rtb_FilterCoefficient = (rtP.Kd3508 * rtb_Filter_kr - localDW->Filter_DSTATE) *
    100.0;

  /* Switch: '<S18>/Switch1' incorporates:
   *  Constant: '<S18>/Constant'
   *  Gain: '<S18>/Multiply'
   *  Saturate: '<S158>/Saturation'
   *  Sum: '<S18>/Sum1'
   */
  if (rtb_Switch1 > rtP.DeadBand) {
    /* Sum: '<S160>/Sum' incorporates:
     *  DiscreteIntegrator: '<S151>/Integrator'
     *  Gain: '<S156>/Proportional Gain'
     */
    op = (rtP.Kp3508 * rtb_Filter_kr + localDW->Integrator_DSTATE) +
      rtb_FilterCoefficient;

    /* Saturate: '<S158>/Saturation' */
    if (op > 16384.0) {
      op = 16384.0;
    } else if (op < -16384.0) {
      op = -16384.0;
    }

    rtb_Switch1 = rtP.trans * op - rtu_yaw_speed_rpm;
  } else {
    rtb_Switch1 = 0.0;
  }

  /* End of Switch: '<S18>/Switch1' */

  /* Gain: '<S106>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S98>/Filter'
   *  Gain: '<S97>/Derivative Gain'
   *  Sum: '<S98>/SumD'
   */
  rtb_FilterCoefficient_d = (rtP.KAd3508 * rtb_Switch1 -
    localDW->Filter_DSTATE_n) * 100.0;

  /* Sum: '<S112>/Sum' incorporates:
   *  DiscreteIntegrator: '<S103>/Integrator'
   *  Gain: '<S108>/Proportional Gain'
   */
  op = (rtP.KAp3508 * rtb_Switch1 + localDW->Integrator_DSTATE_i) +
    rtb_FilterCoefficient_d;

  /* Saturate: '<S110>/Saturation' */
  if (op > 16384.0) {
    *rty_Out1 = 16384.0;
  } else if (op < -16384.0) {
    *rty_Out1 = -16384.0;
  } else {
    *rty_Out1 = op;
  }

  /* End of Saturate: '<S110>/Saturation' */

  /* Chart: '<S18>/Chart1' incorporates:
   *  Sum: '<S18>/Sum3'
   */
  Chart1(rtu_yaw_ecd - rtu_yaw_last_ecd, &localDW->u2, &localDW->sf_Chart1);

  /* Sum: '<S18>/Sum4' */
  *rty_yaw_circle_out = localDW->u2 + rtu_yaw_circle;

  /* Chart: '<S18>/Chart2' */
  Chart2(0.0, 0.0, &op, &localDW->sf_Chart2);

  /* Update for DiscreteIntegrator: '<S151>/Integrator' incorporates:
   *  Gain: '<S148>/Integral Gain'
   */
  localDW->Integrator_DSTATE += rtP.Ki3508 * rtb_Filter_kr * 0.001;

  /* Update for DiscreteIntegrator: '<S146>/Filter' */
  localDW->Filter_DSTATE += 0.001 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S98>/Filter' */
  localDW->Filter_DSTATE_n += 0.001 * rtb_FilterCoefficient_d;

  /* Update for DiscreteIntegrator: '<S103>/Integrator' incorporates:
   *  Gain: '<S100>/Integral Gain'
   */
  localDW->Integrator_DSTATE_i += rtP.KAi3508 * rtb_Switch1 * 0.001;
}

/*
 * Output and update for action system:
 *    '<S3>/If Action_speed Subsystem'
 *    '<S4>/If Action_speed Subsystem'
 *    '<S5>/If Action_speed Subsystem'
 *    '<S6>/If Action_speed Subsystem'
 *    '<S7>/If Action_speed Subsystem'
 *    '<S8>/If Action_speed Subsystem'
 *    '<S15>/If Action_speed Subsystem'
 *    '<S16>/If Action_speed Subsystem'
 */
static void IfAction_speedSubsystem_l(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T *rty_Out1, DW_IfAction_speedSubsystem_a *localDW)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_Sum;
  real_T u0;

  /* Sum: '<S319>/Sum' */
  rtb_Sum = rtu_yaw_target - rtu_yaw_speed_rpm;

  /* Gain: '<S356>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S348>/Filter'
   *  Gain: '<S347>/Derivative Gain'
   *  Sum: '<S348>/SumD'
   */
  rtb_FilterCoefficient = (rtP.Kd2006 * rtb_Sum - localDW->Filter_DSTATE) *
    100.0;

  /* Sum: '<S362>/Sum' incorporates:
   *  DiscreteIntegrator: '<S353>/Integrator'
   *  Gain: '<S358>/Proportional Gain'
   */
  u0 = (rtP.Kp2006 * rtb_Sum + localDW->Integrator_DSTATE) +
    rtb_FilterCoefficient;

  /* Saturate: '<S360>/Saturation' */
  if (u0 > 16384.0) {
    *rty_Out1 = 16384.0;
  } else if (u0 < -16384.0) {
    *rty_Out1 = -16384.0;
  } else {
    *rty_Out1 = u0;
  }

  /* End of Saturate: '<S360>/Saturation' */

  /* Update for DiscreteIntegrator: '<S348>/Filter' */
  localDW->Filter_DSTATE += 0.001 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S353>/Integrator' incorporates:
   *  Gain: '<S350>/Integral Gain'
   */
  localDW->Integrator_DSTATE += rtP.Ki2006 * rtb_Sum * 0.001;
}

/*
 * System initialize for action system:
 *    '<S3>/If Action_speed Subsystem1'
 *    '<S4>/If Action_speed Subsystem1'
 *    '<S5>/If Action_speed Subsystem1'
 *    '<S6>/If Action_speed Subsystem1'
 *    '<S7>/If Action_speed Subsystem1'
 *    '<S8>/If Action_speed Subsystem1'
 *    '<S15>/If Action_speed Subsystem1'
 *    '<S16>/If Action_speed Subsystem1'
 */
static void IfAction_speedSubsystem1_m_Init(DW_IfAction_speedSubsystem1_j
  *localDW)
{
  real_T op;

  /* SystemInitialize for Chart: '<S320>/Chart1' */
  Chart1_Init(&localDW->u2);

  /* SystemInitialize for Chart: '<S320>/Chart2' */
  Chart2_Init(&op);
}

/*
 * Output and update for action system:
 *    '<S3>/If Action_speed Subsystem1'
 *    '<S4>/If Action_speed Subsystem1'
 *    '<S5>/If Action_speed Subsystem1'
 *    '<S6>/If Action_speed Subsystem1'
 *    '<S7>/If Action_speed Subsystem1'
 *    '<S8>/If Action_speed Subsystem1'
 *    '<S15>/If Action_speed Subsystem1'
 *    '<S16>/If Action_speed Subsystem1'
 */
static void IfAction_speedSubsystem1_c(real_T rtu_yaw_speed_rpm, real_T
  rtu_yaw_target, real_T rtu_yaw_ecd, real_T rtu_yaw_last_ecd, real_T
  rtu_yaw_circle, real_T *rty_Out1, real_T *rty_yaw_circle_out,
  DW_IfAction_speedSubsystem1_j *localDW)
{
  real_T op;
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_j;
  real_T rtb_Filter_i;
  real_T rtb_Switch1;

  /* Sum: '<S320>/Sum' incorporates:
   *  Gain: '<S320>/Gain'
   *  Sum: '<S320>/Sum2'
   */
  rtb_Filter_i = rtu_yaw_target - (8191.0 * rtu_yaw_circle + rtu_yaw_ecd);

  /* Abs: '<S320>/Abs' */
  rtb_Switch1 = fabs(rtb_Filter_i);

  /* Switch: '<S320>/Switch2' incorporates:
   *  Constant: '<S320>/Constant'
   */
  if (!(rtb_Switch1 > rtP.DeadBand)) {
    rtb_Filter_i = 0.0;
  }

  /* End of Switch: '<S320>/Switch2' */

  /* Gain: '<S456>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S448>/Filter'
   *  Gain: '<S447>/Derivative Gain'
   *  Sum: '<S448>/SumD'
   */
  rtb_FilterCoefficient = (rtP.Kd2006 * rtb_Filter_i - localDW->Filter_DSTATE) *
    100.0;

  /* Switch: '<S320>/Switch1' incorporates:
   *  Constant: '<S320>/Constant'
   *  Gain: '<S320>/Multiply'
   *  Saturate: '<S460>/Saturation'
   *  Sum: '<S320>/Sum1'
   */
  if (rtb_Switch1 > rtP.DeadBand) {
    /* Sum: '<S462>/Sum' incorporates:
     *  DiscreteIntegrator: '<S453>/Integrator'
     *  Gain: '<S458>/Proportional Gain'
     */
    op = (rtP.Kp2006 * rtb_Filter_i + localDW->Integrator_DSTATE) +
      rtb_FilterCoefficient;

    /* Saturate: '<S460>/Saturation' */
    if (op > 16384.0) {
      op = 16384.0;
    } else if (op < -16384.0) {
      op = -16384.0;
    }

    rtb_Switch1 = rtP.trans * op - rtu_yaw_speed_rpm;
  } else {
    rtb_Switch1 = 0.0;
  }

  /* End of Switch: '<S320>/Switch1' */

  /* Gain: '<S408>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S400>/Filter'
   *  Gain: '<S399>/Derivative Gain'
   *  Sum: '<S400>/SumD'
   */
  rtb_FilterCoefficient_j = (rtP.KAd2006 * rtb_Switch1 -
    localDW->Filter_DSTATE_k) * 100.0;

  /* Sum: '<S414>/Sum' incorporates:
   *  DiscreteIntegrator: '<S405>/Integrator'
   *  Gain: '<S410>/Proportional Gain'
   */
  op = (rtP.KAp2006 * rtb_Switch1 + localDW->Integrator_DSTATE_a) +
    rtb_FilterCoefficient_j;

  /* Saturate: '<S412>/Saturation' */
  if (op > 16384.0) {
    *rty_Out1 = 16384.0;
  } else if (op < -16384.0) {
    *rty_Out1 = -16384.0;
  } else {
    *rty_Out1 = op;
  }

  /* End of Saturate: '<S412>/Saturation' */

  /* Chart: '<S320>/Chart1' incorporates:
   *  Sum: '<S320>/Sum3'
   */
  Chart1(rtu_yaw_ecd - rtu_yaw_last_ecd, &localDW->u2, &localDW->sf_Chart1);

  /* Sum: '<S320>/Sum4' */
  *rty_yaw_circle_out = localDW->u2 + rtu_yaw_circle;

  /* Chart: '<S320>/Chart2' */
  Chart2(0.0, 0.0, &op, &localDW->sf_Chart2);

  /* Update for DiscreteIntegrator: '<S453>/Integrator' incorporates:
   *  Gain: '<S450>/Integral Gain'
   */
  localDW->Integrator_DSTATE += rtP.Ki2006 * rtb_Filter_i * 0.001;

  /* Update for DiscreteIntegrator: '<S448>/Filter' */
  localDW->Filter_DSTATE += 0.001 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S400>/Filter' */
  localDW->Filter_DSTATE_k += 0.001 * rtb_FilterCoefficient_j;

  /* Update for DiscreteIntegrator: '<S405>/Integrator' incorporates:
   *  Gain: '<S402>/Integral Gain'
   */
  localDW->Integrator_DSTATE_a += rtP.KAi2006 * rtb_Switch1 * 0.001;
}

/* Model step function */
void pidctl_step(void)
{
  /* SwitchCase: '<S1>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle0'
   *  Inport: '<Root>/yaw_ecd0'
   *  Inport: '<Root>/yaw_last_ecd0'
   *  Inport: '<Root>/yaw_speed_rpm0'
   *  Inport: '<Root>/yaw_status0'
   *  Inport: '<Root>/yaw_target0'
   *  Outport: '<Root>/yaw_ANG_OUT0'
   *  Outport: '<Root>/yaw_SPD_OUT0'
   *  Outport: '<Root>/yaw_circle_out0'
   */
  switch ((int32_T)rtU.yaw_status0) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm0, rtU.yaw_target0,
      &rtY.yaw_SPD_OUT0, &rtDW.IfAction_speedSubsystem_h);

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S18>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm0, rtU.yaw_target0, rtU.yaw_ecd0,
      rtU.yaw_last_ecd0, rtU.yaw_circle0, &rtY.yaw_ANG_OUT0,
      &rtY.yaw_circle_out0, &rtDW.IfAction_speedSubsystem1_o);

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S1>/Switch Case' */

  /* SwitchCase: '<S2>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle1'
   *  Inport: '<Root>/yaw_ecd1'
   *  Inport: '<Root>/yaw_last_ecd1'
   *  Inport: '<Root>/yaw_speed_rpm1'
   *  Inport: '<Root>/yaw_status1'
   *  Inport: '<Root>/yaw_target1'
   *  Outport: '<Root>/yaw_ANG_OUT1'
   *  Outport: '<Root>/yaw_SPD_OUT1'
   *  Outport: '<Root>/yaw_circle_out1'
   */
  switch ((int32_T)rtU.yaw_status1) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S2>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S168>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm1, rtU.yaw_target1,
      &rtY.yaw_SPD_OUT1, &rtDW.IfAction_speedSubsystem_a);

    /* End of Outputs for SubSystem: '<S2>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S2>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S169>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm1, rtU.yaw_target1, rtU.yaw_ecd1,
      rtU.yaw_last_ecd1, rtU.yaw_circle1, &rtY.yaw_ANG_OUT1,
      &rtY.yaw_circle_out1, &rtDW.IfAction_speedSubsystem1_e);

    /* End of Outputs for SubSystem: '<S2>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S2>/Switch Case' */

  /* SwitchCase: '<S9>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle2'
   *  Inport: '<Root>/yaw_ecd2'
   *  Inport: '<Root>/yaw_last_ecd2'
   *  Inport: '<Root>/yaw_speed_rpm2'
   *  Inport: '<Root>/yaw_status2'
   *  Inport: '<Root>/yaw_target2'
   *  Outport: '<Root>/yaw_ANG_OUT2'
   *  Outport: '<Root>/yaw_SPD_OUT2'
   *  Outport: '<Root>/yaw_circle_out2'
   */
  switch ((int32_T)rtU.yaw_status2) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S9>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1225>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm2, rtU.yaw_target2,
      &rtY.yaw_SPD_OUT2, &rtDW.IfAction_speedSubsystem_o);

    /* End of Outputs for SubSystem: '<S9>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S9>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1226>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm2, rtU.yaw_target2, rtU.yaw_ecd2,
      rtU.yaw_last_ecd2, rtU.yaw_circle2, &rtY.yaw_ANG_OUT2,
      &rtY.yaw_circle_out2, &rtDW.IfAction_speedSubsystem1_a);

    /* End of Outputs for SubSystem: '<S9>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S9>/Switch Case' */

  /* SwitchCase: '<S10>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle3'
   *  Inport: '<Root>/yaw_ecd3'
   *  Inport: '<Root>/yaw_last_ecd3'
   *  Inport: '<Root>/yaw_speed_rpm3'
   *  Inport: '<Root>/yaw_status3'
   *  Inport: '<Root>/yaw_target3'
   *  Outport: '<Root>/yaw_ANG_OUT3'
   *  Outport: '<Root>/yaw_SPD_OUT3'
   *  Outport: '<Root>/yaw_circle_out3'
   */
  switch ((int32_T)rtU.yaw_status3) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S10>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1376>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm3, rtU.yaw_target3,
      &rtY.yaw_SPD_OUT3, &rtDW.IfAction_speedSubsystem_e);

    /* End of Outputs for SubSystem: '<S10>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S10>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1377>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm3, rtU.yaw_target3, rtU.yaw_ecd3,
      rtU.yaw_last_ecd3, rtU.yaw_circle3, &rtY.yaw_ANG_OUT3,
      &rtY.yaw_circle_out3, &rtDW.IfAction_speedSubsystem1_fu);

    /* End of Outputs for SubSystem: '<S10>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S10>/Switch Case' */

  /* SwitchCase: '<S12>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle5'
   *  Inport: '<Root>/yaw_ecd5'
   *  Inport: '<Root>/yaw_last_ecd5'
   *  Inport: '<Root>/yaw_speed_rpm5'
   *  Inport: '<Root>/yaw_status5'
   *  Inport: '<Root>/yaw_target5'
   *  Outport: '<Root>/yaw_ANG_OUT5'
   *  Outport: '<Root>/yaw_SPD_OUT5'
   *  Outport: '<Root>/yaw_circle_out5'
   */
  switch ((int32_T)rtU.yaw_status5) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S12>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1678>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm5, rtU.yaw_target5,
      &rtY.yaw_SPD_OUT5, &rtDW.IfAction_speedSubsystem_b);

    /* End of Outputs for SubSystem: '<S12>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S12>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1679>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm5, rtU.yaw_target5, rtU.yaw_ecd5,
      rtU.yaw_last_ecd5, rtU.yaw_circle5, &rtY.yaw_ANG_OUT5,
      &rtY.yaw_circle_out5, &rtDW.IfAction_speedSubsystem1_n);

    /* End of Outputs for SubSystem: '<S12>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S12>/Switch Case' */

  /* SwitchCase: '<S13>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle6'
   *  Inport: '<Root>/yaw_ecd6'
   *  Inport: '<Root>/yaw_last_ecd6'
   *  Inport: '<Root>/yaw_speed_rpm6'
   *  Inport: '<Root>/yaw_status6'
   *  Inport: '<Root>/yaw_target6'
   *  Outport: '<Root>/yaw_ANG_OUT6'
   *  Outport: '<Root>/yaw_SPD_OUT6'
   *  Outport: '<Root>/yaw_circle_out6'
   */
  switch ((int32_T)rtU.yaw_status6) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S13>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1829>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm6, rtU.yaw_target6,
      &rtY.yaw_SPD_OUT6, &rtDW.IfAction_speedSubsystem_n);

    /* End of Outputs for SubSystem: '<S13>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S13>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1830>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm6, rtU.yaw_target6, rtU.yaw_ecd6,
      rtU.yaw_last_ecd6, rtU.yaw_circle6, &rtY.yaw_ANG_OUT6,
      &rtY.yaw_circle_out6, &rtDW.IfAction_speedSubsystem1_ah);

    /* End of Outputs for SubSystem: '<S13>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S13>/Switch Case' */

  /* SwitchCase: '<S14>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle7'
   *  Inport: '<Root>/yaw_ecd7'
   *  Inport: '<Root>/yaw_last_ecd7'
   *  Inport: '<Root>/yaw_speed_rpm7'
   *  Inport: '<Root>/yaw_status7'
   *  Inport: '<Root>/yaw_target7'
   *  Outport: '<Root>/yaw_ANG_OUT7'
   *  Outport: '<Root>/yaw_SPD_OUT7'
   *  Outport: '<Root>/yaw_circle_out7'
   */
  switch ((int32_T)rtU.yaw_status7) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S14>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1980>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm7, rtU.yaw_target7,
      &rtY.yaw_SPD_OUT7, &rtDW.IfAction_speedSubsystem_ln);

    /* End of Outputs for SubSystem: '<S14>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S14>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1981>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm7, rtU.yaw_target7, rtU.yaw_ecd7,
      rtU.yaw_last_ecd7, rtU.yaw_circle7, &rtY.yaw_ANG_OUT7,
      &rtY.yaw_circle_out7, &rtDW.IfAction_speedSubsystem1_nt);

    /* End of Outputs for SubSystem: '<S14>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S14>/Switch Case' */

  /* SwitchCase: '<S15>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle8'
   *  Inport: '<Root>/yaw_ecd8'
   *  Inport: '<Root>/yaw_last_ecd8'
   *  Inport: '<Root>/yaw_speed_rpm8'
   *  Inport: '<Root>/yaw_status8'
   *  Inport: '<Root>/yaw_target8'
   *  Outport: '<Root>/yaw_ANG_OUT8'
   *  Outport: '<Root>/yaw_SPD_OUT8'
   *  Outport: '<Root>/yaw_circle_out8'
   */
  switch ((int32_T)rtU.yaw_status8) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S15>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S2131>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm8, rtU.yaw_target8,
      &rtY.yaw_SPD_OUT8, &rtDW.IfAction_speedSubsystem_o0);

    /* End of Outputs for SubSystem: '<S15>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S15>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S2132>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm8, rtU.yaw_target8, rtU.yaw_ecd8,
      rtU.yaw_last_ecd8, rtU.yaw_circle8, &rtY.yaw_ANG_OUT8,
      &rtY.yaw_circle_out8, &rtDW.IfAction_speedSubsystem1_b);

    /* End of Outputs for SubSystem: '<S15>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S15>/Switch Case' */

  /* SwitchCase: '<S16>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle9'
   *  Inport: '<Root>/yaw_ecd9'
   *  Inport: '<Root>/yaw_last_ecd9'
   *  Inport: '<Root>/yaw_speed_rpm9'
   *  Inport: '<Root>/yaw_status9'
   *  Inport: '<Root>/yaw_target9'
   *  Outport: '<Root>/yaw_ANG_OUT9'
   *  Outport: '<Root>/yaw_SPD_OUT9'
   *  Outport: '<Root>/yaw_circle_out9'
   */
  switch ((int32_T)rtU.yaw_status9) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S16>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S2282>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm9, rtU.yaw_target9,
      &rtY.yaw_SPD_OUT9, &rtDW.IfAction_speedSubsystem_l5);

    /* End of Outputs for SubSystem: '<S16>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S16>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S2283>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm9, rtU.yaw_target9, rtU.yaw_ecd9,
      rtU.yaw_last_ecd9, rtU.yaw_circle9, &rtY.yaw_ANG_OUT9,
      &rtY.yaw_circle_out9, &rtDW.IfAction_speedSubsystem1_fv);

    /* End of Outputs for SubSystem: '<S16>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S16>/Switch Case' */

  /* SwitchCase: '<S3>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle10'
   *  Inport: '<Root>/yaw_ecd10'
   *  Inport: '<Root>/yaw_last_ecd10'
   *  Inport: '<Root>/yaw_speed_rpm10'
   *  Inport: '<Root>/yaw_status10'
   *  Inport: '<Root>/yaw_target10'
   *  Outport: '<Root>/yaw_ANG_OUT10'
   *  Outport: '<Root>/yaw_SPD_OUT10'
   *  Outport: '<Root>/yaw_circle_out10'
   */
  switch ((int32_T)rtU.yaw_status10) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S3>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S319>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm10, rtU.yaw_target10,
      &rtY.yaw_SPD_OUT10, &rtDW.IfAction_speedSubsystem_lx);

    /* End of Outputs for SubSystem: '<S3>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S3>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S320>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm10, rtU.yaw_target10,
      rtU.yaw_ecd10, rtU.yaw_last_ecd10, rtU.yaw_circle10, &rtY.yaw_ANG_OUT10,
      &rtY.yaw_circle_out10, &rtDW.IfAction_speedSubsystem1_cw);

    /* End of Outputs for SubSystem: '<S3>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S3>/Switch Case' */

  /* SwitchCase: '<S4>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle11'
   *  Inport: '<Root>/yaw_ecd11'
   *  Inport: '<Root>/yaw_last_ecd11'
   *  Inport: '<Root>/yaw_speed_rpm11'
   *  Inport: '<Root>/yaw_status11'
   *  Inport: '<Root>/yaw_target11'
   *  Outport: '<Root>/yaw_ANG_OUT11'
   *  Outport: '<Root>/yaw_SPD_OUT11'
   *  Outport: '<Root>/yaw_circle_out11'
   */
  switch ((int32_T)rtU.yaw_status11) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S4>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S470>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm11, rtU.yaw_target11,
      &rtY.yaw_SPD_OUT11, &rtDW.IfAction_speedSubsystem_i);

    /* End of Outputs for SubSystem: '<S4>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S4>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S471>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm11, rtU.yaw_target11,
      rtU.yaw_ecd11, rtU.yaw_last_ecd11, rtU.yaw_circle11, &rtY.yaw_ANG_OUT11,
      &rtY.yaw_circle_out11, &rtDW.IfAction_speedSubsystem1_k);

    /* End of Outputs for SubSystem: '<S4>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S4>/Switch Case' */

  /* SwitchCase: '<S7>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle12'
   *  Inport: '<Root>/yaw_ecd12'
   *  Inport: '<Root>/yaw_last_ecd12'
   *  Inport: '<Root>/yaw_speed_rpm12'
   *  Inport: '<Root>/yaw_status12'
   *  Inport: '<Root>/yaw_target12'
   *  Outport: '<Root>/yaw_ANG_OUT12'
   *  Outport: '<Root>/yaw_SPD_OUT12'
   *  Outport: '<Root>/yaw_circle_out12'
   */
  switch ((int32_T)rtU.yaw_status12) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S7>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S923>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm12, rtU.yaw_target12,
      &rtY.yaw_SPD_OUT12, &rtDW.IfAction_speedSubsystem_c);

    /* End of Outputs for SubSystem: '<S7>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S7>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S924>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm12, rtU.yaw_target12,
      rtU.yaw_ecd12, rtU.yaw_last_ecd12, rtU.yaw_circle12, &rtY.yaw_ANG_OUT12,
      &rtY.yaw_circle_out12, &rtDW.IfAction_speedSubsystem1_g);

    /* End of Outputs for SubSystem: '<S7>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S7>/Switch Case' */

  /* SwitchCase: '<S8>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle13'
   *  Inport: '<Root>/yaw_ecd13'
   *  Inport: '<Root>/yaw_last_ecd13'
   *  Inport: '<Root>/yaw_speed_rpm13'
   *  Inport: '<Root>/yaw_status13'
   *  Inport: '<Root>/yaw_target13'
   *  Outport: '<Root>/yaw_ANG_OUT13'
   *  Outport: '<Root>/yaw_SPD_OUT13'
   *  Outport: '<Root>/yaw_circle_out13'
   */
  switch ((int32_T)rtU.yaw_status13) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S8>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1074>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm13, rtU.yaw_target13,
      &rtY.yaw_SPD_OUT13, &rtDW.IfAction_speedSubsystem_d);

    /* End of Outputs for SubSystem: '<S8>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S8>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1075>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm13, rtU.yaw_target13,
      rtU.yaw_ecd13, rtU.yaw_last_ecd13, rtU.yaw_circle13, &rtY.yaw_ANG_OUT13,
      &rtY.yaw_circle_out13, &rtDW.IfAction_speedSubsystem1_gs);

    /* End of Outputs for SubSystem: '<S8>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S8>/Switch Case' */

  /* SwitchCase: '<S5>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle14'
   *  Inport: '<Root>/yaw_ecd14'
   *  Inport: '<Root>/yaw_last_ecd14'
   *  Inport: '<Root>/yaw_speed_rpm14'
   *  Inport: '<Root>/yaw_status14'
   *  Inport: '<Root>/yaw_target14'
   *  Outport: '<Root>/yaw_ANG_OUT14'
   *  Outport: '<Root>/yaw_SPD_OUT14'
   *  Outport: '<Root>/yaw_circle_out14'
   */
  switch ((int32_T)rtU.yaw_status14) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S5>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S621>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm14, rtU.yaw_target14,
      &rtY.yaw_SPD_OUT14, &rtDW.IfAction_speedSubsystem_j);

    /* End of Outputs for SubSystem: '<S5>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S5>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S622>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm14, rtU.yaw_target14,
      rtU.yaw_ecd14, rtU.yaw_last_ecd14, rtU.yaw_circle14, &rtY.yaw_ANG_OUT14,
      &rtY.yaw_circle_out14, &rtDW.IfAction_speedSubsystem1_h);

    /* End of Outputs for SubSystem: '<S5>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S5>/Switch Case' */

  /* SwitchCase: '<S6>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle15'
   *  Inport: '<Root>/yaw_ecd15'
   *  Inport: '<Root>/yaw_last_ecd15'
   *  Inport: '<Root>/yaw_speed_rpm15'
   *  Inport: '<Root>/yaw_status15'
   *  Inport: '<Root>/yaw_target15'
   *  Outport: '<Root>/yaw_ANG_OUT15'
   *  Outport: '<Root>/yaw_SPD_OUT15'
   *  Outport: '<Root>/yaw_circle_out15'
   */
  switch ((int32_T)rtU.yaw_status15) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S6>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S772>/Action Port'
     */
    IfAction_speedSubsystem_l(rtU.yaw_speed_rpm15, rtU.yaw_target15,
      &rtY.yaw_SPD_OUT15, &rtDW.IfAction_speedSubsystem_f);

    /* End of Outputs for SubSystem: '<S6>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S6>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S773>/Action Port'
     */
    IfAction_speedSubsystem1_c(rtU.yaw_speed_rpm15, rtU.yaw_target15,
      rtU.yaw_ecd15, rtU.yaw_last_ecd15, rtU.yaw_circle15, &rtY.yaw_ANG_OUT15,
      &rtY.yaw_circle_out15, &rtDW.IfAction_speedSubsystem1_f);

    /* End of Outputs for SubSystem: '<S6>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S6>/Switch Case' */

  /* SwitchCase: '<S11>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_circle4'
   *  Inport: '<Root>/yaw_ecd4'
   *  Inport: '<Root>/yaw_last_ecd4'
   *  Inport: '<Root>/yaw_speed_rpm4'
   *  Inport: '<Root>/yaw_status4'
   *  Inport: '<Root>/yaw_target4'
   *  Outport: '<Root>/yaw_ANG_OUT4'
   *  Outport: '<Root>/yaw_SPD_OUT4'
   *  Outport: '<Root>/yaw_circle_out4'
   */
  switch ((int32_T)rtU.yaw_status4) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S11>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1527>/Action Port'
     */
    IfAction_speedSubsystem(rtU.yaw_speed_rpm4, rtU.yaw_target4,
      &rtY.yaw_SPD_OUT4, &rtDW.IfAction_speedSubsystem_jr);

    /* End of Outputs for SubSystem: '<S11>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S11>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1528>/Action Port'
     */
    IfAction_speedSubsystem1(rtU.yaw_speed_rpm4, rtU.yaw_target4, rtU.yaw_ecd4,
      rtU.yaw_last_ecd4, rtU.yaw_circle4, &rtY.yaw_ANG_OUT4,
      &rtY.yaw_circle_out4, &rtDW.IfAction_speedSubsystem1_fd);

    /* End of Outputs for SubSystem: '<S11>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S11>/Switch Case' */
}

/* Model initialize function */
void pidctl_initialize(void)
{
  /* SystemInitialize for IfAction SubSystem: '<S1>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_o);

  /* End of SystemInitialize for SubSystem: '<S1>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_e);

  /* End of SystemInitialize for SubSystem: '<S2>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S9>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_a);

  /* End of SystemInitialize for SubSystem: '<S9>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S10>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_fu);

  /* End of SystemInitialize for SubSystem: '<S10>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S12>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_n);

  /* End of SystemInitialize for SubSystem: '<S12>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S13>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_ah);

  /* End of SystemInitialize for SubSystem: '<S13>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S14>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_nt);

  /* End of SystemInitialize for SubSystem: '<S14>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S15>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_b);

  /* End of SystemInitialize for SubSystem: '<S15>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S16>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_fv);

  /* End of SystemInitialize for SubSystem: '<S16>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_cw);

  /* End of SystemInitialize for SubSystem: '<S3>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S4>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_k);

  /* End of SystemInitialize for SubSystem: '<S4>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S7>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_g);

  /* End of SystemInitialize for SubSystem: '<S7>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S8>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_gs);

  /* End of SystemInitialize for SubSystem: '<S8>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S5>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_h);

  /* End of SystemInitialize for SubSystem: '<S5>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S6>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_m_Init(&rtDW.IfAction_speedSubsystem1_f);

  /* End of SystemInitialize for SubSystem: '<S6>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S11>/If Action_speed Subsystem1' */
  IfAction_speedSubsystem1_Init(&rtDW.IfAction_speedSubsystem1_fd);

  /* End of SystemInitialize for SubSystem: '<S11>/If Action_speed Subsystem1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
