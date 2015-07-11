/*
 * STR_Indirect.c
 *
 * Code generation for function 'STR_Indirect'
 *
 * C source code generated on: Tue Apr 29 23:32:21 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "STR_Indirect.h"
#include "../define.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real_T STR_Indirect(real_T Theta[4], real_T uc, real_T y)
{
  real_T u;
  static real_T u_ = 0;
  static real_T uc_ = 0, y_ = 0;

  u = (((-(Theta[3] / Theta[2]) * u_ + (0.999999956715774 / Theta[2]) * uc) + (3.91619188305815e-08 /
        Theta[2]) * uc_) - ((-4.12230724487735e-09 - Theta[0]) / Theta[2]) * y) - ((4.24835425529272e-18 -
    Theta[1]) / Theta[2]) * y_;
  if (u > 90.0) {
    u = 90.0;
  } else {
    if (u < -90.0) {
      u = -90.0;
    }
  }

  uc_ = uc;
  u_ = u;
  y_ = y;

  return (u_);
}

real_T STR_Indirect2(real_T Theta[4], real_T uc, real_T y)
{
  real_T u;
  static real_T u_ = 0;
  static real_T uc_ = 0, y_ = 0;

  u = (((-(Theta[3] / Theta[2]) * u_ + (0.999999956715774 / Theta[2]) * uc) + (3.91619188305815e-08 /
        Theta[2]) * uc_) - ((-4.12230724487735e-09 - Theta[0]) / Theta[2]) * y) - ((4.24835425529272e-18 -
    Theta[1]) / Theta[2]) * y_;
  if (u > 90.0) {
    u = 90.0;
  } else {
    if (u < -90.0) {
      u = -90.0;
    }
  }

  uc_ = uc;
  u_ = u;
  y_ = y;

  return (u_);
}
/* End of code generation (STR_Indirect.c) */
