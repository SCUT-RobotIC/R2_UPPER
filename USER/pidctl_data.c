/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pidctl_data.c
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

/* Block parameters (default storage) */
P rtP = {
  /* Variable: DeadBand
   * Referenced by:
   *   '<S18>/Chart2'
   *   '<S18>/Switch1'
   *   '<S18>/Switch2'
   *   '<S169>/Chart2'
   *   '<S169>/Switch1'
   *   '<S169>/Switch2'
   *   '<S320>/Chart2'
   *   '<S320>/Switch1'
   *   '<S320>/Switch2'
   *   '<S471>/Chart2'
   *   '<S471>/Switch1'
   *   '<S471>/Switch2'
   *   '<S622>/Chart2'
   *   '<S622>/Switch1'
   *   '<S622>/Switch2'
   *   '<S773>/Chart2'
   *   '<S773>/Switch1'
   *   '<S773>/Switch2'
   *   '<S924>/Chart2'
   *   '<S924>/Switch1'
   *   '<S924>/Switch2'
   *   '<S1075>/Chart2'
   *   '<S1075>/Switch1'
   *   '<S1075>/Switch2'
   *   '<S1226>/Chart2'
   *   '<S1226>/Switch1'
   *   '<S1226>/Switch2'
   *   '<S1377>/Chart2'
   *   '<S1377>/Switch1'
   *   '<S1377>/Switch2'
   *   '<S1528>/Chart2'
   *   '<S1528>/Switch1'
   *   '<S1528>/Switch2'
   *   '<S1679>/Chart2'
   *   '<S1679>/Switch1'
   *   '<S1679>/Switch2'
   *   '<S1830>/Chart2'
   *   '<S1830>/Switch1'
   *   '<S1830>/Switch2'
   *   '<S1981>/Chart2'
   *   '<S1981>/Switch1'
   *   '<S1981>/Switch2'
   *   '<S2132>/Chart2'
   *   '<S2132>/Switch1'
   *   '<S2132>/Switch2'
   *   '<S2283>/Chart2'
   *   '<S2283>/Switch1'
   *   '<S2283>/Switch2'
   */
  500.0,

  /* Variable: KAd2006
   * Referenced by:
   *   '<S399>/Derivative Gain'
   *   '<S550>/Derivative Gain'
   *   '<S701>/Derivative Gain'
   *   '<S852>/Derivative Gain'
   *   '<S1003>/Derivative Gain'
   *   '<S1154>/Derivative Gain'
   *   '<S2211>/Derivative Gain'
   *   '<S2362>/Derivative Gain'
   */
  0.0,

  /* Variable: KAd3508
   * Referenced by:
   *   '<S97>/Derivative Gain'
   *   '<S248>/Derivative Gain'
   *   '<S1305>/Derivative Gain'
   *   '<S1456>/Derivative Gain'
   *   '<S1607>/Derivative Gain'
   *   '<S1758>/Derivative Gain'
   *   '<S1909>/Derivative Gain'
   *   '<S2060>/Derivative Gain'
   */
  0.0,

  /* Variable: KAi2006
   * Referenced by:
   *   '<S402>/Integral Gain'
   *   '<S553>/Integral Gain'
   *   '<S704>/Integral Gain'
   *   '<S855>/Integral Gain'
   *   '<S1006>/Integral Gain'
   *   '<S1157>/Integral Gain'
   *   '<S2214>/Integral Gain'
   *   '<S2365>/Integral Gain'
   */
  0.0,

  /* Variable: KAi3508
   * Referenced by:
   *   '<S100>/Integral Gain'
   *   '<S251>/Integral Gain'
   *   '<S1308>/Integral Gain'
   *   '<S1459>/Integral Gain'
   *   '<S1610>/Integral Gain'
   *   '<S1761>/Integral Gain'
   *   '<S1912>/Integral Gain'
   *   '<S2063>/Integral Gain'
   */
  0.0,

  /* Variable: KAp2006
   * Referenced by:
   *   '<S410>/Proportional Gain'
   *   '<S561>/Proportional Gain'
   *   '<S712>/Proportional Gain'
   *   '<S863>/Proportional Gain'
   *   '<S1014>/Proportional Gain'
   *   '<S1165>/Proportional Gain'
   *   '<S2222>/Proportional Gain'
   *   '<S2373>/Proportional Gain'
   */
  0.0,

  /* Variable: KAp3508
   * Referenced by:
   *   '<S108>/Proportional Gain'
   *   '<S259>/Proportional Gain'
   *   '<S1316>/Proportional Gain'
   *   '<S1467>/Proportional Gain'
   *   '<S1618>/Proportional Gain'
   *   '<S1769>/Proportional Gain'
   *   '<S1920>/Proportional Gain'
   *   '<S2071>/Proportional Gain'
   */
  0.0,

  /* Variable: Kd2006
   * Referenced by:
   *   '<S347>/Derivative Gain'
   *   '<S447>/Derivative Gain'
   *   '<S498>/Derivative Gain'
   *   '<S598>/Derivative Gain'
   *   '<S649>/Derivative Gain'
   *   '<S749>/Derivative Gain'
   *   '<S800>/Derivative Gain'
   *   '<S900>/Derivative Gain'
   *   '<S951>/Derivative Gain'
   *   '<S1051>/Derivative Gain'
   *   '<S1102>/Derivative Gain'
   *   '<S1202>/Derivative Gain'
   *   '<S2159>/Derivative Gain'
   *   '<S2259>/Derivative Gain'
   *   '<S2310>/Derivative Gain'
   *   '<S2410>/Derivative Gain'
   */
  0.0,

  /* Variable: Kd3508
   * Referenced by:
   *   '<S45>/Derivative Gain'
   *   '<S145>/Derivative Gain'
   *   '<S196>/Derivative Gain'
   *   '<S296>/Derivative Gain'
   *   '<S1253>/Derivative Gain'
   *   '<S1353>/Derivative Gain'
   *   '<S1404>/Derivative Gain'
   *   '<S1504>/Derivative Gain'
   *   '<S1555>/Derivative Gain'
   *   '<S1655>/Derivative Gain'
   *   '<S1706>/Derivative Gain'
   *   '<S1806>/Derivative Gain'
   *   '<S1857>/Derivative Gain'
   *   '<S1957>/Derivative Gain'
   *   '<S2008>/Derivative Gain'
   *   '<S2108>/Derivative Gain'
   */
  0.0,

  /* Variable: Ki2006
   * Referenced by:
   *   '<S350>/Integral Gain'
   *   '<S450>/Integral Gain'
   *   '<S501>/Integral Gain'
   *   '<S601>/Integral Gain'
   *   '<S652>/Integral Gain'
   *   '<S752>/Integral Gain'
   *   '<S803>/Integral Gain'
   *   '<S903>/Integral Gain'
   *   '<S954>/Integral Gain'
   *   '<S1054>/Integral Gain'
   *   '<S1105>/Integral Gain'
   *   '<S1205>/Integral Gain'
   *   '<S2162>/Integral Gain'
   *   '<S2262>/Integral Gain'
   *   '<S2313>/Integral Gain'
   *   '<S2413>/Integral Gain'
   */
  0.0,

  /* Variable: Ki3508
   * Referenced by:
   *   '<S48>/Integral Gain'
   *   '<S148>/Integral Gain'
   *   '<S199>/Integral Gain'
   *   '<S299>/Integral Gain'
   *   '<S1256>/Integral Gain'
   *   '<S1356>/Integral Gain'
   *   '<S1407>/Integral Gain'
   *   '<S1507>/Integral Gain'
   *   '<S1558>/Integral Gain'
   *   '<S1658>/Integral Gain'
   *   '<S1709>/Integral Gain'
   *   '<S1809>/Integral Gain'
   *   '<S1860>/Integral Gain'
   *   '<S1960>/Integral Gain'
   *   '<S2011>/Integral Gain'
   *   '<S2111>/Integral Gain'
   */
  0.0,

  /* Variable: Kp2006
   * Referenced by:
   *   '<S358>/Proportional Gain'
   *   '<S458>/Proportional Gain'
   *   '<S509>/Proportional Gain'
   *   '<S609>/Proportional Gain'
   *   '<S660>/Proportional Gain'
   *   '<S760>/Proportional Gain'
   *   '<S811>/Proportional Gain'
   *   '<S911>/Proportional Gain'
   *   '<S962>/Proportional Gain'
   *   '<S1062>/Proportional Gain'
   *   '<S1113>/Proportional Gain'
   *   '<S1213>/Proportional Gain'
   *   '<S2170>/Proportional Gain'
   *   '<S2270>/Proportional Gain'
   *   '<S2321>/Proportional Gain'
   *   '<S2421>/Proportional Gain'
   */
  0.0,

  /* Variable: Kp3508
   * Referenced by:
   *   '<S56>/Proportional Gain'
   *   '<S156>/Proportional Gain'
   *   '<S207>/Proportional Gain'
   *   '<S307>/Proportional Gain'
   *   '<S1264>/Proportional Gain'
   *   '<S1364>/Proportional Gain'
   *   '<S1415>/Proportional Gain'
   *   '<S1515>/Proportional Gain'
   *   '<S1566>/Proportional Gain'
   *   '<S1666>/Proportional Gain'
   *   '<S1717>/Proportional Gain'
   *   '<S1817>/Proportional Gain'
   *   '<S1868>/Proportional Gain'
   *   '<S1968>/Proportional Gain'
   *   '<S2019>/Proportional Gain'
   *   '<S2119>/Proportional Gain'
   */
  0.0,

  /* Variable: trans
   * Referenced by:
   *   '<S18>/Multiply'
   *   '<S169>/Multiply'
   *   '<S320>/Multiply'
   *   '<S471>/Multiply'
   *   '<S622>/Multiply'
   *   '<S773>/Multiply'
   *   '<S924>/Multiply'
   *   '<S1075>/Multiply'
   *   '<S1226>/Multiply'
   *   '<S1377>/Multiply'
   *   '<S1528>/Multiply'
   *   '<S1679>/Multiply'
   *   '<S1830>/Multiply'
   *   '<S1981>/Multiply'
   *   '<S2132>/Multiply'
   *   '<S2283>/Multiply'
   */
  0.5
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
