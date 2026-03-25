/*
 * Copyright (c) 2016,2018-2019 LAAS/CNRS
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
#include <assert.h>
#include <err.h>

#include <algorithm>
#include <cmath>

#include "kdtp.h"

namespace kdtp {

  Spline::Spline(const Dof &dof, double init[3], double end[3])
    : phases_(0)
  {
    vmax_ = dof.getVelocityMax();
    amax_ = dof.getAccelerationMax();
    jmax_ = dof.getJerkMax();
    smax_ = dof.getSnapMax();

    init_[0] = init[0];
    end_[0] = end[0];
    if(end_[0] < dof.getPositionMin())
      end_[0] = dof.getPositionMin();
    if(end_[0] > dof.getPositionMax())
      end_[0] = dof.getPositionMax();

    if(dof.isRotation()) {
      while(fabs(init_[0] - end_[0]) > M_PI)
        end_[0] -= sign(end_[0]) * 2*M_PI;
    }

    init_[1] = init[1];
    end_[1] = end[1];
    if(fabs(end_[1]) > vmax_) end_[1] = copysign(vmax_, end_[1]);

    init_[2] = init[2];
    end_[2] = end[2];
    if(fabs(end_[2]) > amax_) end_[2] = copysign(amax_, end_[2]);

    stay_still_ =
      fabs(init_[0] - end_[0]) < EPSILON &&
      fabs(init_[1]) < EPSILON &&
      fabs(end_[1]) < EPSILON &&
      fabs(init_[2]) < EPSILON &&
      fabs(end_[2]) < EPSILON;

    if (!stay_still_) v_optim();
  }

  /**
   * Returns the value of case_ABC for a given value of aB.
   * Input:
   *    double aB : acceleration during phase B
   * Output:
   *    int : case code
   */
  int
  Spline::case_abc(double aB) const
  {
    double a0 = init_[2];
    int sA = sign(aB-a0);
    int sC = -sign(aB);
    double aj = jmax_*jmax_/smax_;
    int cond_A = fabs(a0-aB)>aj;
    int cond_C = fabs(aB)>aj;
    int case_ABC = 1000*(sA+1)+100*(sC+1)+10*cond_A+cond_C;

    return case_ABC;
  }

  /**
   * Returns the value of case_EGH for a given value of aG.
   * Input:
   *    double aG : acceleration during phase G
   * Output:
   *    int : case code
   */
  int
  Spline::case_egh(double aG) const
  {
    double aF = end_[2];
    int sH = sign(aF-aG);
    int sE = sign(aG);
    double aj = jmax_*jmax_/smax_;
    int cond_H = fabs(aG-aF)>aj;
    int cond_E = fabs(aG)>aj;
    int case_EGH = 1000*(sH+1)+100*(sE+1)+10*cond_H+cond_E;

    return case_EGH;
  }

  /**
   * Sets durations of phases A1, A2, A3, C1, C2, C3 into the _durations
   * attribute and signs of the snap during phases A1 and C1 into the _signs
   * attribute.
   * Input:
   *    double aB : acceleration during phase B
   */
  void
  Spline::durations_and_signs_ac(double aB)
  {
    double a0 = init_[2];
    double aj = jmax_*jmax_/smax_;

    signs_[A] = sign(aB-a0);
    signs_[C] = -sign(aB);

    if (fabs(aB-a0) > aj) {
      durations_[A1] = jmax_/smax_;
      durations_[A2] = fabs(aB-a0)/jmax_ - durations_[A1];
    } else {
      durations_[A1] = sqrt(fabs(a0-aB)/smax_);
      durations_[A2] = 0.;
    }

    if (fabs(aB) > aj) {
      durations_[C1] = jmax_/smax_;
      durations_[C2] = fabs(aB)/jmax_ - durations_[C1];
    } else {
      durations_[C1] = sqrt(fabs(aB)/smax_);
      durations_[C2] = 0.;
    }
  }

  /**
   * Sets durations of phases E1, E2, E3, H1, H2, H3 into the _durations
   * attribute and signs of the snap during phases E1 and H1 into the _signs
   * attribute.
   * Input:
   *    double aG : acceleration during phase G
   */
  void
  Spline::durations_and_signs_eh(double aG)
  {
    double aF = end_[2];
    double aj=jmax_*jmax_/smax_;

    signs_[H] = sign(aF-aG);
    signs_[E] = sign(aG);
    if (fabs(aF-aG) > aj) {
      durations_[H1] = jmax_/smax_;
      durations_[H2] = fabs(aF-aG)/jmax_-jmax_/smax_;
    } else {
      durations_[H1] = sqrt(fabs(aF-aG)/smax_);
      durations_[H2] = 0.;
    }
    if (fabs(aG) > aj) {
      durations_[E1] = jmax_/smax_;
      durations_[E2] = fabs(aG)/jmax_-jmax_/smax_;
    } else {
      durations_[E1] = sqrt(fabs(aG)/smax_);
      durations_[E2] = 0.;
    }
  }

  /**
   * Calculates position at the end of phase C.
   * Inputs:
   *    double aB     : acceleration during phase B
   * Output:
   *    double : position at the end of phase C
   */
  double
  Spline::x_c(double aB)
  {
    double tA1 = durations_[A1];
    double tA2 = durations_[A2];
    double tB = durations_[B];
    double tC1 = durations_[C1];
    double tC2 = durations_[C2];
    double sA = signs_[A];
    double sC = signs_[C];
    double x0 = init_[0];
    double v0 = init_[1];
    double a0 = init_[2];
    double tA1_2 = tA1*tA1;
    double tA1_3 = tA1_2*tA1;
    double tA1_4 = tA1_3*tA1;
    double tA2_2 = tA2*tA2;
    double tA2_3 = tA2_2*tA2;
    double tB_2 = tB*tB;
    double tC1_2 = tC1*tC1;
    double tC1_3 = tC1_2*tC1;
    double tC1_4 = tC1_3*tC1;
    double tC2_2 = tC2*tC2;
    double tC2_3 = tC2_2*tC2;

    return
      (7*sA*smax_*tA1_4)/12+(7*sA*smax_*tA1_3*tA2)/6+sA*smax_*tA1_3*tB
      +2*sA*smax_*tA1_3*tC1+sA*smax_*tA1_3*tC2+(3*sA*smax_*tA1_2*tA2_2)/4
      +(3*sA*smax_*tA1_2*tA2*tB)/2+3*sA*smax_*tA1_2*tA2*tC1
      +(3*sA*smax_*tA1_2*tA2*tC2)/2+(sA*smax_*tA1_2*tB_2)/2
      +2*sA*smax_*tA1_2*tB*tC1+sA*smax_*tA1_2*tB*tC2+2*sA*smax_*tA1_2*tC1_2
      +2*sA*smax_*tA1_2*tC1*tC2+(sA*smax_*tA1_2*tC2_2)/2+2*a0*tA1_2
      +(sA*smax_*tA1*tA2_3)/6+(sA*smax_*tA1*tA2_2*tB)/2
      +sA*smax_*tA1*tA2_2*tC1+(sA*smax_*tA1*tA2_2*tC2)/2
      +(sA*smax_*tA1*tA2*tB_2)/2+2*sA*smax_*tA1*tA2*tB*tC1
      +sA*smax_*tA1*tA2*tB*tC2+2*sA*smax_*tA1*tA2*tC1_2
      +2*sA*smax_*tA1*tA2*tC1*tC2+(sA*smax_*tA1*tA2*tC2_2)/2
      +2*a0*tA1*tA2+2*a0*tA1*tB+4*a0*tA1*tC1+2*a0*tA1*tC2
      +2*v0*tA1+(a0*tA2_2)/2+a0*tA2*tB+2*a0*tA2*tC1+a0*tA2*tC2+v0*tA2
      +(a0*tB_2)/2+2*a0*tB*tC1+a0*tB*tC2+v0*tB+(7*sC*smax_*tC1_4)/12
      +(7*sC*smax_*tC1_3*tC2)/6+(3*sC*smax_*tC1_2*tC2_2)/4+2*a0*tC1_2
      +(sC*smax_*tC1*tC2_3)/6+2*a0*tC1*tC2+2*v0*tC1+(a0*tC2_2)/2+v0*tC2+x0;
  }

  /**
   * Calculates position at the beginning of phase E.
   * Inputs:
   *    double aG     : acceleration during phase G
   * Output:
   *    double : position at the beginning of phase E.
   */
  double
  Spline::x_e(double aG)
  {
    double tE1 = durations_[E1];
    double tE2 = durations_[E2];
    double tG = durations_[G];
    double tH1 = durations_[H1];
    double tH2 = durations_[H2];
    double sE = signs_[E];
    double sH = signs_[H];
    double xF = end_[0];
    double vF = end_[1];
    double aF = end_[2];
    double tE1_2 = tE1*tE1;
    double tE1_3 = tE1_2*tE1;
    double tE1_4 = tE1_3*tE1;
    double tE2_2 = tE2*tE2;
    double tE2_3 = tE2_2*tE2;
    double tG_2 = tG*tG;
    double tH1_2 = tH1*tH1;
    double tH1_3 = tH1_2*tH1;
    double tH1_4 = tH1_3*tH1;
    double tH2_2 = tH2*tH2;
    double tH2_3 = tH2_2*tH2;

    return
      -(7*sE*smax_*tE1_4)/12-(7*sE*smax_*tE1_3*tE2)/6
      -(3*sE*smax_*tE1_2*tE2_2)/4-2*sH*smax_*tE1_2*tH1_2
      -2*sH*smax_*tE1_2*tH1*tH2+2*aF*tE1_2-(sE*smax_*tE1*tE2_3)/6
      -2*sH*smax_*tE1*tE2*tH1_2-2*sH*smax_*tE1*tE2*tH1*tH2+2*aF*tE1*tE2
      -2*sH*smax_*tE1*tG*tH1_2-2*sH*smax_*tE1*tG*tH1*tH2+2*aF*tE1*tG
      -2*sH*smax_*tE1*tH1_3-3*sH*smax_*tE1*tH1_2*tH2-sH*smax_*tE1*tH1*tH2_2
      +4*aF*tE1*tH1+2*aF*tE1*tH2-2*vF*tE1-(sH*smax_*tE2_2*tH1_2)/2
      -(sH*smax_*tE2_2*tH1*tH2)/2+(aF*tE2_2)/2-sH*smax_*tE2*tG*tH1_2
      -sH*smax_*tE2*tG*tH1*tH2+aF*tE2*tG-sH*smax_*tE2*tH1_3
      -(3*sH*smax_*tE2*tH1_2*tH2)/2-(sH*smax_*tE2*tH1*tH2_2)/2+2*aF*tE2*tH1
      +aF*tE2*tH2-vF*tE2-(sH*smax_*tG_2*tH1_2)/2-(sH*smax_*tG_2*tH1*tH2)/2
      +(aF*tG_2)/2-sH*smax_*tG*tH1_3-(3*sH*smax_*tG*tH1_2*tH2)/2
      -(sH*smax_*tG*tH1*tH2_2)/2+2*aF*tG*tH1+aF*tG*tH2-vF*tG
      -(7*sH*smax_*tH1_4)/12-(7*sH*smax_*tH1_3*tH2)/6
      -(3*sH*smax_*tH1_2*tH2_2)/4+2*aF*tH1_2-(sH*smax_*tH1*tH2_3)/6
      +2*aF*tH1*tH2-2*vF*tH1+(aF*tH2_2)/2-vF*tH2+xF;
  }

  /**
   * Calculates velocity at the end of phase C.
   * Inputs:
   *    double aB     : acceleration during phase B
   *    double new_tB : an updated value of the duration of phase B
   * Output:
   *    double : velocity at the end of phase C
   */
  double
  Spline::v_c(double aB, double new_tB)
  {
    durations_[B] = new_tB;
    durations_and_signs_ac(aB);

    double tA1 = durations_[A1];
    double tA2 = durations_[A2];
    double tB = durations_[B];
    double tC1 = durations_[C1];
    double tC2 = durations_[C2];
    double sA = signs_[A];
    double sC = signs_[C];
    double v0 = init_[1];
    double a0 = init_[2];
    double tA1_2 = tA1*tA1;
    double tA1_3 = tA1_2*tA1;
    double tA2_2 = tA2*tA2;
    double tC1_2 = tC1*tC1;
    double tC1_3 = tC1_2*tC1;
    double tC2_2 = tC2*tC2;

    return
      sA*smax_*tA1_3+(3*sA*smax_*tA1_2*tA2)/2+2*sA*smax_*tA1_2*tC1
      +sA*smax_*tA1_2*tC2+sA*smax_*tB*tA1_2+(sA*smax_*tA1*tA2_2)/2
      +2*sA*smax_*tA1*tA2*tC1+sA*smax_*tA1*tA2*tC2+sA*smax_*tB*tA1*tA2
      +2*a0*tA1+a0*tA2+sC*smax_*tC1_3+(3*sC*smax_*tC1_2*tC2)/2
      +(sC*smax_*tC1*tC2_2)/2+2*a0*tC1+a0*tC2+v0+a0*tB;
  }

  /**
   * Calculates velocity at the beginning of phase E.
   * Inputs:
   *    double aG     : acceleration during phase G
   *    double new_tG : an updated value of the duration of phase G
   * Output:
   *    double : velocity at the beginning of phase E.
   */
  double
  Spline::v_e(double aG, double new_tG) {
    durations_[G] = new_tG;
    durations_and_signs_eh(aG);

    double tE1 = durations_[E1];
    double tE2 = durations_[E2];
    double tG = durations_[G];
    double tH1 = durations_[H1];
    double tH2 = durations_[H2];
    double sE = signs_[E];
    double sH = signs_[H];
    double vF = end_[1];
    double aF = end_[2];
    double tE1_2 = tE1*tE1;
    double tE1_3 = tE1_2*tE1;
    double tE2_2 = tE2*tE2;
    double tH1_2 = tH1*tH1;
    double tH1_3 = tH1_2*tH1;
    double tH2_2 = tH2*tH2;

    return
      sE*smax_*tE1_3+(3*sE*smax_*tE1_2*tE2)/2+(sE*smax_*tE1*tE2_2)/2
      +2*sH*smax_*tE1*tH1_2+2*sH*smax_*tE1*tH1*tH2-2*aF*tE1
      +sH*smax_*tE2*tH1_2+sH*smax_*tE2*tH1*tH2-aF*tE2+sH*smax_*tH1_3
      +(3*sH*smax_*tH1_2*tH2)/2+sH*smax_*tG*tH1_2+(sH*smax_*tH1*tH2_2)/2
      +sH*smax_*tG*tH1*tH2-2*aF*tH1-aF*tH2+vF-aF*tG;
  }

  /**
   * Sets the ordered acceleration intervals for phases A to C into _int_a_abc,
   * the corresponding values of velocity into _int_v_abc and the corresponding
   * cases codes into _cases_abc.
   */
  void
  Spline::intervals_ac()
  {
    double aj = jmax_*jmax_/smax_;
    double a0 = init_[2];
    double Ac[7] = { amax_, 0., aj, -aj, a0, a0 + aj, a0 - aj };
    double tmp[8];

    tmp[0] = -amax_;
    int n = 1;
    for(int k = 0; k < 7; k++) {
      if ((Ac[k]*a0 <= 0. || fabs(Ac[k])>= fabs(a0)) && fabs(Ac[k]) <= amax_) {
        int l = 0;
        while (l < n && Ac[k]>tmp[l])
          l++;

        if (l == n || Ac[k] < tmp[l]) {
          for(int i = n; i > l; i--)
            tmp[i] = tmp[i-1];
          tmp[l]=Ac[k];
          n++;
        }
      }
    }

    int_a_abc_.push_back(tmp[0]);
    int_v_abc_.push_back(v_c(tmp[0], 0.));
    for(int k = 1; k < n; k++) {
      double a1 = tmp[k-1];
      double a2 = tmp[k];
      int_a_abc_.push_back(a2);
      int_v_abc_.push_back(v_c(a2, 0.));
      cases_abc_.push_back(case_abc((a1+a2)/2));
    }
  }

  /**
   * Sets the ordered acceleration intervals for phases E to H into _int_a_egh,
   * the corresponding values of velocity into _int_v_egh and the corresponding
   * cases codes into _cases_egh.
   */
  void
  Spline::intervals_eh()
  {
    double aj = jmax_*jmax_/smax_;
    double aF = end_[2];
    double Ac[7] = { amax_, 0., aj, -aj, aF, aF+aj, aF-aj };
    double tmp[8];

    tmp[0] = -amax_;
    int n = 1;
    for(int k = 0; k < 7; k++) {
      if ((Ac[k]*aF <= 0. || fabs(Ac[k]) >= fabs(aF)) && fabs(Ac[k]) <= amax_) {
        int l = 0;
        while (l < n && Ac[k] < tmp[l])
          l++;

        if (l == n || Ac[k] > tmp[l]) {
          for(int i = n; i > l; i--)
            tmp[i] = tmp[i-1];
          tmp[l] = Ac[k];
          n++;
        }
      }
    }

    int_a_egh_.push_back(tmp[0]);
    int_v_egh_.push_back(v_e(tmp[0], 0.));
    for(int k = 1; k < n; k++) {
      double a1 = tmp[k-1];
      double a2 = tmp[k];
      int_a_egh_.push_back(a2);
      int_v_egh_.push_back(v_e(a2, 0.));
      cases_egh_.push_back(case_egh((a1+a2)/2));
    }
  }

  /**
   * Returns acceleration during phase B and sets corresponding duration of
   * phase B into durations_ for a given desired velocity at the end of phase
   * C.
   * Input:
   *    double vC : desired velocity at the end of phase C
   * Output:
   *    double : acceleration during phase B
   */
  double
  Spline::a_b(double vC)
  {
    double v0 = init_[1];
    double a0 = init_[2];
    double a0_2 = a0*a0;
    double a0_3 = a0_2*a0;
    double jmax_2 = jmax_*jmax_;
    double jmax_4 = jmax_2*jmax_2;
    double smax_2 = smax_*smax_;

    assert(!cases_abc_.empty());
    assert(int_v_abc_.size() == cases_abc_.size() + 1);
    assert(int_a_abc_.size() == cases_abc_.size() + 1);

    if (vC < int_v_abc_[0]) {
      durations_[B] = (int_v_abc_[0]-vC)/amax_;
      return -amax_;
    }

    if (vC >= int_v_abc_.at(cases_abc_.size())) {
      durations_[B] = (vC-int_v_abc_.at(cases_abc_.size()))/amax_;
      return amax_;
    }

    durations_[B] = 0;
    unsigned int index_int = 0;
    while (index_int < cases_abc_.size() && vC > int_v_abc_[index_int+1])
      index_int++;
    if (index_int >= cases_abc_.size()) {
      warnx("kdtp::Spline::a_b: vC = %g not found", vC);
      return a0;
    }

    int case_ABC = cases_abc_[index_int];

    double sol[4];
    double a, b, c, d;
    unsigned int nsol, k;

    switch (case_ABC) {
      case 210:
        return -sqr(jmax_-sqrt(
                      4*sqrt(smax_*(a0*jmax_2+a0_2*smax_
                                   +2*jmax_*smax_*(v0-vC)))+jmax_2))/(4*smax_);
      case 211:
        return (jmax_2-sqrt(jmax_4+2*a0_2*smax_2+4*jmax_*smax_2*(v0-vC)
                            +2*a0*jmax_2*smax_))/(2*smax_);
      case 2010:
        return sqr(jmax_-sqrt(
                     4*sqrt(smax_*(a0_2*smax_-a0*jmax_2
                                  +2*jmax_*smax_*(vC-v0)))+jmax_2))/(4*smax_);
      case 2011:
        return (sqrt(jmax_4+2*a0_2*smax_2+4*jmax_*smax_2*(vC-v0)
                     -2*a0*jmax_2*smax_)-jmax_2)/(2*smax_);

      case 200:
        a = 2*sqrt(smax_)*(v0-vC);
        b = -a0_2;
        c = -4*a0*sqrt(smax_)*(v0-vC);
        d = -(smax_*sqr(v0-vC)+a0_3);
        nsol = poly_root_4(a0, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = -sqr(sol[k]) + a0;
        break;

      case 201:
        a = 2*jmax_/sqrt(smax_);
        b = (jmax_2-2*a0*smax_)/smax_;
        c = -4*a0*jmax_/sqrt(smax_);
        d = -(a0*jmax_2-a0_2*smax_+2*jmax_*smax_*(v0-vC))/smax_;
        nsol = poly_root_4(1, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = -sqr(sol[k]) + a0;
        break;

      case 2000:
        a = 2*sqrt(smax_)*(vC-v0);
        b = -a0_2;
        c = 0;
        d = -(smax_*sqr(vC-v0)+a0_3);
        nsol = poly_root_4(a0, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = sqr(sol[k]);
        break;

      case 2001:
        a = 2*jmax_/sqrt(smax_);
        b = (2*a0*smax_+jmax_2)/smax_;
        c = 4*a0*jmax_/sqrt(smax_);
        d = (a0*jmax_2+a0_2*smax_+2*jmax_*smax_*(v0-vC))/smax_;
        nsol = poly_root_4(1, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = sqr(sol[k]) + a0;
        break;

      default:
        warnx("kdtp::Spline::a_b: unknown case %d (vC = %g)", case_ABC, vC);
        assert(!"kdtp::Spline::a_b: unknown case");
    }

    double dV;
    double aB;

    if (nsol > 0) {
      double dVmin = fabs(vC - v_c(sol[0], 0.));
      aB = sol[0];
      for(unsigned int i = 1; i < nsol; i++) {
        dV = fabs(vC - v_c(sol[i], 0.));
        if (dV < dVmin) {
          dVmin = dV;
          aB = sol[i];
        }
      }
      dV = dVmin;
      if (dV < EPSI4) return aB;
    }

    double mina = int_a_abc_[index_int];
    double maxa = int_a_abc_[index_int+1];

    aB = (mina+maxa)/2;
    dV = vC - v_c(aB, 0.);
    while(fabs(dV) > EPSI4 && fabs(mina-maxa) > EPSILON) {
      if (dV>0)
        mina = aB;
      else
        maxa = aB;
      aB = (mina+maxa)/2;
      dV = vC - v_c(aB, 0.);
    }

    return aB;
  }

  /**
   * Returns acceleration during phase G and sets corresponding duration of
   * phase G into durations_ for a given desired velocity at the beginning of
   * phase E.
   * Input:
   *    double vE : desired velocity at the beginning of phase E
   * Output:
   *    double : acceleration during phase G
   */
  double
  Spline::a_g(double vE)
  {
    double vF = end_[1];
    double aF = end_[2];
    double aF_2 = sqr(aF);
    double aF_3 = aF_2*aF;
    double jmax_2 = sqr(jmax_);
    double jmax_4 = sqr(jmax_2);
    double smax_2 = sqr(smax_);

    assert(!cases_egh_.empty());
    assert(int_v_egh_.size() == cases_egh_.size() + 1);
    assert(int_a_egh_.size() == cases_egh_.size() + 1);

    if (vE < int_v_egh_[0]) {
      durations_[G] = (int_v_egh_[0]-vE)/amax_;
      return amax_;
    }

    if (vE >= int_v_egh_[cases_egh_.size()]) {
      durations_[G] = (vE - int_v_egh_[cases_egh_.size()])/amax_;
      return -amax_;
    }

    durations_[G] = 0;
    unsigned int index_int = 0;
    while(index_int < cases_egh_.size() && vE > int_v_egh_[index_int+1])
        index_int++;
    if (index_int >= cases_egh_.size()) {
      warnx("kdtp::Spline::a_g: vE = %g not found", vE);
      return aF;
    }

    int case_EGH = cases_egh_[index_int];
    double sol[4];
    double a, b, c, d;
    unsigned int nsol, k;

    switch (case_EGH) {
      case 210:
        return sqr(jmax_-sqrt(
                     4*sqrt(smax_*(aF_2*smax_-aF*jmax_2
                                  +2*jmax_*smax_*(vF-vE)))+jmax_2))/(4*smax_);
      case 211:
        return (sqrt(jmax_4+2*aF_2*smax_2-2*aF*jmax_2*smax_
                     +4*jmax_*smax_2*(vF-vE))-jmax_2)/(2*smax_);
      case 2010:
        return -sqr(jmax_-sqrt(
                      4*sqrt(smax_*(aF*jmax_2+aF_2*smax_
                                   +2*jmax_*smax_*(vE-vF)))+jmax_2))/(4*smax_);
      case 2011:
        return (jmax_2-sqrt(jmax_4+2*aF_2*smax_2+2*aF*jmax_2*smax_
                            +4*jmax_*smax_2*(vE-vF)))/(2*smax_);

      case 200:
        a = 2*sqrt(smax_)*(vF-vE);
        b = -aF_2;
        c = 0;
        d = -(smax_*sqr(vE-vF)+aF_3);
        nsol = poly_root_4(aF, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = sqr(sol[k]);
        break;

      case 201:
        a = 2*jmax_/sqrt(smax_);
        b = (2*aF*smax_+jmax_2)/smax_;
        c = 4*aF*jmax_/sqrt(smax_);
        d = (aF*jmax_2+aF_2*smax_+2*jmax_*smax_*(vE-vF))/smax_;
        nsol = poly_root_4(1, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = sqr(sol[k])+aF;
        break;

      case 2000:
        a = 2*sqrt(smax_)*(vE-vF);
        b = -aF_2;
        c = 4*aF*sqrt(smax_)*(vF-vE);
        d = -(smax_*sqr(vE-vF)+aF_3);
        nsol = poly_root_4(aF, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = -sqr(sol[k])+aF;
        break;

      case 2001:
        a = 2*jmax_/sqrt(smax_);
        b = (jmax_2-2*aF*smax_)/smax_;
        c = -4*aF*jmax_/sqrt(smax_);
        d = (aF_2*smax_-aF*jmax_2+2*jmax_*smax_*(vF-vE))/smax_;
        nsol = poly_root_4(1, a, b, c, d, sol);
        for(k = 0; k < nsol; k++)
          sol[k] = -sqr(sol[k])+aF;
        break;

      default:
        warnx("kdtp::Spline::a_g: unknown case %d (vC = %g)", case_EGH, vE);
        assert(!"kdtp::Spline::a_g: unknown case");
    }

    double dV;
    double aG;

    if (nsol > 0) {
      double dVmin = fabs(vE - v_e(sol[0], 0));
      aG = sol[0];
      for(unsigned int i = 1; i < nsol; i++) {
        dV = fabs(vE - v_e(sol[i], 0.));
        if (dV < dVmin) {
          dVmin = dV;
          aG = sol[i];
        }
      }
      dV = dVmin;
      if (dV < EPSI4) return aG;
    }

    double mina = int_a_egh_[index_int];
    double maxa = int_a_egh_[index_int+1];

    aG = (mina+maxa)/2;
    dV = vE - v_e(aG, 0.);
    while(fabs(dV) > EPSI4 && fabs(maxa-mina) > EPSILON) {
      if (dV>0)
        mina = aG;
      else
        maxa = aG;
      aG = (mina+maxa)/2;
      dV = vE - v_e(aG, 0.);
    }

    return aG;
  }

  /**
   * Returns difference between position at the beginning of phase E and
   * position at the end of phase C given a desired velocity during phase D.
   * Sets the corresponding durations into durations_ and snap signs into
   * signs_.
   * Input:
   *    double vD : desired velocity during phase D
   * Output:
   *    double : difference between position at the beginning of phase E and
   *             position at the end of phase C
   */
  double
  Spline::d_x(double vD)
  {
    double aB = a_b(vD);
    durations_and_signs_ac(aB);
    double aG = a_g(vD);
    durations_and_signs_eh(aG);
    double xC = x_c(aB);
    double xE = x_e(aG);
    double dX = xE-xC;
    double tD;

    if (sign(vD) == 0)
      tD = 0.;
    else {
      tD = dX/vD;
      if (tD < 0.) tD = 0.;
    }

    durations_[D] = tD;
    durations_[F] =
      2*durations_[A1]
      +durations_[A2]
      +durations_[B]
      +2*durations_[C1]
      +durations_[C2]
      +durations_[D]
      +2*durations_[E1]
      +durations_[E2]
      +durations_[G]
      +2*durations_[H1]
      +durations_[H2];

    return dX;
  }

  /**
   * Calculates optimal velocity during phase D and sets it into v_opt_.
   * Calculates and sets the corresponding durations and snap signs for all
   * phases.
   */
  void
  Spline::v_optim()
  {
    // Setting values of cases, velocities intervals and accelerations
    // intervals
    intervals_ac();
    intervals_eh();

    // Searching a zero of d_x with secant method
    double v1 = 0;
    double dX1 = d_x(v1);
    int s = dX1 >= -0. ? 1 : -1;
    double v2 = s*vmax_;
    double dX2 = d_x(v2);
    double v = v2-(v2-v1)*dX2/(dX2-dX1);
    double dX = d_x(v);
    unsigned int nb_it = 0;

    while(fabs(dX) > EPSI4 && nb_it < NB_IT_MAX) {
      dX1 = dX2;
      dX2 = dX;
      v1 = v2;
      v2 = v;
      v = v2-(v2-v1)*dX2/(dX2-dX1);
      dX = d_x(v);
      nb_it++;
    }

    // vmax is the upper bound
    if (fabs(v)>vmax_ || s*v<0) {
      v = s*vmax_;
      dX = d_x(v);
    }

    // if not found or out of bound, dichotomial search
    if (fabs(dX)>EPSI4 && s*dX<0) {
      v1 = 0;
      v2 = fabs(v);
      v = v2/2;
      dX = d_x(s*v);
      while(fabs(dX)>EPSI4 && fabs(v1-v2)>EPSILON) {
        if (s*dX<0)
          v2 = v;
        else
          v1 = v;
        v = (v1+v2)/2;
        dX = d_x(s*v);
      }
      if (s*dX<0) {
        v = v1;
        dX = d_x(s*v);
      }
      v = s*v;
    }

    v_opt_ = v;

    if (fabs(v)>vmax_ || s*v<0)
      warnx("kdtp::Spline::v_optim(): v = %f s = %d vmax = %f\n", v, s, vmax_);
  }

  void
  Spline::setValues()
  {
    static const durations_index phase_index[15] = {
      A1, A2, A1, B, C1, C2, C1, D, E1, E2, E1, G, H1, H2, H1
    };

    double snapsigns_[15] = {
      signs_[A], 0., -signs_[A], 0.,
      signs_[C], 0., -signs_[C], 0.,
      signs_[E], 0., -signs_[E], 0.,
      signs_[H], 0., -signs_[H]
    };

    double nextq[5];
    double time = 0.;

    times_[0] = 0.;
    positions_[0] = init_[0];
    velocities_[0] = init_[1];
    accelerations_[0] = init_[2];
    jerks_[0] = 0.;
    phases_ = 0;

    for(int k = 0; k<15; k++) {
      if (durations_[phase_index[k]] > EPSILON) {
        time += durations_[phase_index[k]];
        snaps_[phases_] = snapsigns_[k] * smax_;

        getAllAt(phases_, time, nextq);

        phases_++;
        times_[phases_] = time;
        positions_[phases_] = nextq[0];
        velocities_[phases_] = nextq[1];
        accelerations_[phases_] = nextq[2];
        jerks_[phases_] = nextq[3];
      }
    }
    snaps_[phases_] = 0.;
    phases_++;
  }

  unsigned int
  Spline::getIndexAt(double time) const
  {
    static unsigned int index_cache = 0;

    if (index_cache >= phases_ || time < times_[index_cache])
      index_cache = 0;

    while(index_cache < phases_-1 && time > times_[index_cache + 1])
      index_cache++;

    return index_cache;
  }

  double
  Spline::getPositionAtLocal(unsigned int index, double local) const
  {
    return
      positions_[index]
      + (velocities_[index]
         + (accelerations_[index] /2
            + (jerks_[index] /6
               + snaps_[index] * local/24) * local) * local) * local;
  }

  double
  Spline::getPositionAt(unsigned int index, double time) const
  {
    return getPositionAtLocal(index, time - times_[index]);
  }

  double
  Spline::getVelocityAtLocal(unsigned int index, double local) const
  {
    return
      velocities_[index]
      + (accelerations_[index]
         + (jerks_[index] /2
            + snaps_[index] * local/6) * local) * local;
  }

  double
  Spline::getVelocityAt(unsigned int index, double time) const
  {
    return this->getVelocityAtLocal(index, time - times_[index]);
  }

  double
  Spline::getAccelerationAtLocal(unsigned int index, double local) const
  {
    return
      accelerations_[index]
      + (jerks_[index]
         + snaps_[index] * local/2) * local;
  }

  double
  Spline::getAccelerationAt(unsigned int index, double time) const
  {
    return getAccelerationAtLocal(index, time - times_[index]);
  }

  double
  Spline::getJerkAtLocal(unsigned int index, double local) const
  {
    return
      jerks_[index]
      + snaps_[index] * local;
  }

  double
  Spline::getJerkAt(unsigned int index, double time) const
  {
    return getJerkAtLocal(index, time - times_[index]);
  }

  void
  Spline::getAllAt(unsigned int index, double time, double (&q)[5]) const
  {
    double dt = time - times_[index];
    double dt2_2 = dt * dt/2.;
    double dt3_6 = dt2_2 * dt/3.;
    double dt4_24 = dt3_6 * dt/4.;

    double s = snaps_[index];
    double j = jerks_[index];
    double a = accelerations_[index];
    double v = velocities_[index];
    double p = positions_[index];

    q[4] = s;
    q[3] = j + s * dt;
    q[2] = a + j * dt + s * dt2_2;
    q[1] = v + a * dt + j * dt2_2 + s * dt3_6;
    q[0] = p + v * dt + a * dt2_2 + j * dt3_6 + s * dt4_24;
  }

  double
  Spline::getDuration() const
  {
    return stay_still_ ? 0 : durations_[F];
  }

  /**
   * Synchronizes spline with given duration by reducing velocity dusing phase D
   * Input:
   *    double tF_des : desired total duration
   */
  void
  Spline::synchronize(double tF_des)
  {
    if (stay_still_) return;

    if (durations_[F] < tF_des && fabs(durations_[F]-tF_des)>EPSI4)
    {
      double s = sign(v_opt_);
      double minv = 0;
      double maxv = fabs(v_opt_);
      double vD = maxv/2;
      d_x(s*vD);
      while(fabs(durations_[F]-tF_des)>EPSI4 && fabs(minv-maxv)>EPSILON) {
        if (durations_[F]<tF_des)
          maxv = vD;
        else
          minv = vD;
        vD = (minv+maxv)/2;
        d_x(s*vD);
      }
      vD = s*vD;
    }
    setValues();
  }

  double
  Spline::getPositionAt(double time) const
  {
    if (stay_still_ || time <EPSILON) return init_[0];
    if (time > durations_[F] - EPSILON) return end_[0];

    return getPositionAt(getIndexAt(time), time);
  }

  double
  Spline::getVelocityAt(double time) const
  {
    if (stay_still_) return 0.;
    if (time < EPSILON) return init_[1];
    if (time > durations_[F] - EPSILON) return end_[1];

    return getVelocityAt(getIndexAt(time), time);
  }

  double
  Spline::getAccelerationAt(double time) const
  {
    if (stay_still_) return 0.;
    if (time < EPSILON) return init_[2];
    if (time > durations_[F] - EPSILON) return end_[2];

    return getAccelerationAt(getIndexAt(time), time);
  }

  double
  Spline::getJerkAt(double time) const
  {
    if (stay_still_ || time < EPSILON) return 0.;
    if (time > durations_[F] - EPSILON) return 0.;

    return getJerkAt(getIndexAt(time), time);
  }

  double
  Spline::getSnapAt(double time) const
  {
    if (stay_still_) return 0.;
    if (time < EPSILON) return signs_[A] * smax_;
    if (time > durations_[F] - EPSILON) return 0.;

    return snaps_[getIndexAt(time)];
  }

  void
  Spline::getAllAt(double time, double (&q)[5]) const
  {
    if (stay_still_) {
      q[0] = init_[0];
      q[1] = q[2] = q[3] = q[4] = 0.;
      return;
    }

    if (time < EPSILON) {
      q[0] = init_[0];
      q[1] = init_[1];
      q[2] = init_[2];
      q[3] = 0.;
      q[4] = signs_[A] * smax_;
      return;
    }

    if (time > durations_[F] - EPSILON) {
      q[0] = end_[0];
      q[1] = end_[1];
      q[2] = end_[2];
      q[3] = 0.;
      q[4] = 0.;
      return;
    }

    getAllAt(getIndexAt(time), time, q);
  }
}
