/*
 * Copyright (c) 2016,2018 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                     Alexandre Boeuf on Fri Aug 19 2016
 */
#include <cmath>

#include "kdtp.h"

static unsigned int	cardano(double p, double q, double (&sol)[3]);
static unsigned int	ferrari(double p, double q, double r, double (&sol)[4]);


/**
 * Computing real roots of a second order polynomial.
 *
 * Input
 *    double a : coefficient of the second order
 *    double b : coefficient of the first order
 *    double c : constant term
 *
 * Output
 *    std::vector<double> : real solutions a*X^2+b*X+c=0
 */
unsigned int
kdtp::poly_root_2(double a, double b, double c, double (&sol)[2])
{
  if (fabs(a) < kdtp::EPSILON) {
    if (fabs(b) < kdtp::EPSILON)
      if (fabs(c) < kdtp::EPSILON)
        sol[0] = 0;
      else
        return 0;
    else
      sol[0] = -c/b;
    return 1;
  }

  const double d = b*b-4*a*c;

  if (d < 0) return 0;

  const double b_2a = b/(2*a);

  if (d < kdtp::EPSILON) {
    sol[0] = - b_2a;
    return 1;
  }

  const double sqrtd_2a = sqrt(d)/(2*a);
  sol[0] = - b_2a - sqrtd_2a;
  sol[1] = - b_2a + sqrtd_2a;;
  return 2;
}


/**
 * Computing real roots of a third order polynomial using Cardano's method.
 *
 * Input
 *    double a : coefficient of the third order
 *    double b : coefficient of the second order
 *    double c : coefficient of the first order
 *    double d : constant term
 *
 * Output
 *    std::vector<double> : real solutions of a*X^3+b*X^2+c*X+d=0
 */
unsigned int
kdtp::poly_root_3(double a, double b, double c, double d, double (&sol)[3])
{
  if (fabs(a) < kdtp::EPSILON)
    return poly_root_2(b, c, d, (double (&)[2])sol);

  unsigned int nsol;

  double aa = b/a;
  double bb = c/a;
  double cc = d/a;

  double p = bb-aa*aa/3.;
  double q = 2*aa*aa*aa/27.-aa*bb/3.+cc;

  nsol = cardano(p, q, sol);
  for(unsigned int k = 0; k < nsol; k++)
    sol[k] = sol[k]-aa/3;

  return nsol;
}


/**
 * Computing real roots of a fourth order polynomial using Ferrari's method.
 *
 * Input
 *    double a : coefficient of the fourth order
 *    double b : coefficient of the third order
 *    double c : coefficient of the second order
 *    double d : coefficient of the first order
 *    double e : constant term
 *
 * Output
 *    std::vector<double> : real solutions of a*X^4+b*X^3+c*X^2+d*X+e=0
 */
unsigned int
kdtp::poly_root_4(double a, double b, double c, double d, double e,
                  double (&sol)[4])
{
  if (fabs(a) < kdtp::EPSILON)
    return poly_root_3(b, c, d, e, (double (&)[3])sol);

  unsigned int n;

  double aa = b/a;
  double bb = c/a;
  double cc = d/a;
  double dd = e/a;
  double p = bb-3*aa*aa/8;
  double q = cc-aa*bb/2+aa*aa*aa/8;
  double r = dd-aa*cc/4+bb*aa*aa/16-3*aa*aa*aa*aa/256;

  n = ferrari(p, q, r, sol);
  for(unsigned int k = 0; k < n; k++)
    sol[k] -= aa/4.;

  return n;
}


/**
 * Implementation of Cardano's method.
 *
 * Input :
 *    double p : coefficient of the first order
 *    double q : constant term
 *
 * Output :
 *    return number of real solutions
 *    sol : array of real solutions of X^3+p*X+q=0
 */
static unsigned int
cardano(double p, double q, double (&sol)[3])
{
  const double delta = -(4*p*p*p+27*q*q);

  if (delta < 0) {
    double v = sqrt(-delta/27)/2;
    double u = -q/2+v;

    u = kdtp::sign(u)*pow(fabs(u),1/3.);
    v = -q/2-v;
    v = kdtp::sign(v)*pow(fabs(v),1/3.);
    sol[0] = u+v;
    return 1;
  }

  if (delta < kdtp::EPSILON) {
    if (fabs(p) < kdtp::EPSILON || fabs(q) < kdtp::EPSILON) {
      sol[0] = 0.;
      return 1;
    }

    sol[0] = 3*q/p;
    sol[1] = -sol[0]/2;
    return 2;
  }

  double u = 1/3. * acos((-q/2)*sqrt(27/(-p*p*p)));
  double v = 2 * sqrt(-p/3);
  for(int k = 0; k < 3; k++)
    sol[k] = v * cos(u+2*(k*M_PI/3));

  return 3;
}

/**
 * Implementation of Ferrari's method.
 *
 * Input
 *    double p : coefficient of the second order
 *    double q : coefficient of the first order
 *    double r : constant term
 *
 * Output
 *    std::vector<double> : real solutions of X^4+p*X^2+q*X+r=0
 */
static unsigned int
ferrari(double p, double q, double r, double (&sol)[4])
{
  if (fabs(q) < kdtp::EPSILON) {
    unsigned int n, ntmp;
    double soltmp[2];

    n = 0;
    ntmp = kdtp::poly_root_2(1, p, r, soltmp);
    for(unsigned k = 0; k < ntmp; k++) {
      if (soltmp[k] >= 0.) {
        sol[n++] = -sqrt(soltmp[k]);
        sol[n++] = -sol[0];
      }
    }
    return n;
  }

  double soltmp[3];
  unsigned int ntmp;

  ntmp = kdtp::poly_root_3(1, -p/2, -r, p*r/2-q*q/8, soltmp);

  double y0 = soltmp[0];
  double delta = 2*y0-p;

  if (delta < 0.)
    return kdtp::poly_root_2(1, 0, y0, (double (&)[2])sol);

  unsigned int n;
  double a0 = sqrt(delta);
  double b0;

  if (a0 < kdtp::EPSILON) {
    if (y0*y0-r>0)
      b0 = sqrt(y0*y0-r);
    else
      b0 = 0.;
  } else
    b0 = -q/(2*a0);

  n = kdtp::poly_root_2(1, a0, y0+b0, (double (&)[2])sol);
  ntmp = kdtp::poly_root_2(1, -a0, y0-b0, (double (&)[2])soltmp);
  for(unsigned int k = 0; k < ntmp; k++)
    sol[n++] = soltmp[k];

  return n;
}
