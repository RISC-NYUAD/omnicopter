/*
 * Copyright (c) 2015-2016,2019,2021-2023 LAAS/CNRS
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
 *                                      Anthony Mallet on Wed Aug 26 2015
 */
#include "ukf.h"

#include <iostream>
#include <Eigen/Geometry>

using namespace ukf;


/* --- state_s::transition ------------------------------------------------- */

state_s::prediction_model state_s::pmodel = state_s::constant_acceleration;

state_s
state_s::transition(const state_s &in, double dt) {
  state_s out;

  /* acc, vel, pos */
  switch (pmodel) {
    case constant_velocity:
      out.a() << 0., 0., 0.;
      break;

    case constant_acceleration:
      out.a() = in.a();
      break;
  }
  out.v() = in.v() + out.a() * dt;
  out.p() = in.p() + in.v() * dt + 0.5 * out.a() * dt * dt;

  /* ang vel, att */
  out.w() = in.w();

  Eigen::Quaternion<double> omega_q;
  double a2 = dt * dt * in.w().squaredNorm();
  if (a2 < 1e-1) {
    omega_q.w() = 1 - a2/8 /*std::cos(a/2)*/;
    omega_q.vec() = (0.5 - a2/48 /*std::sin(a/2)/a*/) * dt * in.w();
  } else {
    double a = std::sqrt(a2);
    omega_q.w() = std::cos(a/2);
    omega_q.vec() = std::sin(a/2)/a * dt * in.w();
  }
  out.q() = (omega_q * Eigen::Quaternion<double>(in.q())).coeffs();
  return out;
}


/* --- state_s::process_noise ---------------------------------------------- */

double state_s::max_dadt = 200.;
double state_s::max_dwdt = 50.;

state_s::dcov_s
state_s::process_noise(double dt) const
{
  double astddev = dt*max_dadt/3.;
  double wstddev = dt*max_dwdt/3.;

  double avar = astddev * astddev;
  double wvar = wstddev * wstddev;

  double avar_dt = avar * dt;
  double avar_dt2 = avar_dt * dt;
  double avar_dt3 = avar_dt2 * dt;
  double avar_dt4 = avar_dt3 * dt;

  double wvar_dt = wvar * dt;
  double wvar_dt2 = wvar_dt * dt;

  dcov_s n = dcov_s::Zero();

  n.block<3,3>(0,0).diagonal() <<
    0.25 * avar_dt4, 0.25 * avar_dt4, 0.25 * avar_dt4;
  n.block<3,3>(6,0).diagonal() <<
    0.5 * avar_dt3, 0.5 * avar_dt3, 0.5 * avar_dt3;
  n.block<3,3>(0,6).diagonal() <<
    0.5 * avar_dt3, 0.5 * avar_dt3, 0.5 * avar_dt3;
  n.block<3,3>(12,0).diagonal() <<
    0.5 * avar_dt2, 0.5 * avar_dt2, 0.5 * avar_dt2;
  n.block<3,3>(0,12).diagonal() <<
    0.5 * avar_dt2, 0.5 * avar_dt2, 0.5 * avar_dt2;
  n.block<3,3>(6,6).diagonal() <<
    avar_dt2, avar_dt2, avar_dt2;
  n.block<3,3>(12,6).diagonal() <<
    avar_dt, avar_dt, avar_dt;
  n.block<3,3>(6,12).diagonal() <<
    avar_dt, avar_dt, avar_dt;
  n.block<3,3>(12,12).diagonal() <<
    avar, avar, avar;

  n.block<3,3>(3,3).diagonal() <<
    wvar_dt2, wvar_dt2, wvar_dt2;
  n.block<3,3>(9,3).diagonal() <<
    wvar_dt, wvar_dt, wvar_dt;
  n.block<3,3>(3,9).diagonal() <<
    wvar_dt, wvar_dt, wvar_dt;
  n.block<3,3>(9,9).diagonal() <<
    wvar, wvar, wvar;

  return n;
}


/* --- state_s::add_delta -------------------------------------------------- */

state_s
state_s::add_delta(const state_s &s, const delta_s &ds)
{
  state_s sds;

  Eigen::Matrix<double, 3, 1> dr = ds.segment<3>(3);
  Eigen::Quaternion<double> dq;
  Eigen::Quaternion<double> q = Eigen::Quaternion<double>(s.q());

  /* compute the quaternion from the error quaternion */
  double r2 = dr.squaredNorm();
  dq.w() = (16. - r2) / (16. + r2);
  dq.vec() = (1 + dq.w()) * dr / 4.;
  sds.q() = (dq * Eigen::Quaternion<double>(s.q())).coeffs();

  sds.p() = s.p() + ds.segment<3>(0);
  sds.v() = s.v() + ds.segment<3>(6);
  sds.w() = s.w() + ds.segment<3>(9);
  sds.a() = s.a() + ds.segment<3>(12);

  return sds;
}


/* --- state_s::delta ------------------------------------------------------ */

state_s::delta_s
state_s::delta(const state_s &s2, const state_s &s1)
{
  delta_s ds;
  Eigen::Quaternion<double> q(s1.q());

  Eigen::Quaternion<double> dq =
    Eigen::Quaternion<double>(s2.q()) *
    Eigen::Quaternion<double>(s1.q()).conjugate();

  /* compute the error quaternion. There is a singularity at 2π: to avoid it,
   * just use the opposite quaternion for any angle above π. In practice,
   * updates with angles above π happen only during initialization. */
  if (dq.w() >= 0)
    ds.segment<3>(3) = 4. * dq.vec()/(1.+dq.w());
  else
    ds.segment<3>(3) = 4. * -dq.vec()/(1.-dq.w());

  ds.segment<3>(0) = s2.p() - s1.p();
  ds.segment<3>(6) = s2.v() - s1.v();
  ds.segment<3>(9) = s2.w() - s1.w();
  ds.segment<3>(12) = s2.a() - s1.a();

  return ds;
}


/* --- state_s::rpy -------------------------------------------------------- */

const Eigen::Matrix<double, 3, 1>
state_s::rpy() const
{
  Eigen::Matrix<double, 3, 1> rpy;
  double qx = q()(0), qy = q()(1), qz = q()(2), qw = q()(3);
  double a, b;

  /* With (R)oll, (P)itch, (Y)aw defined as:
   *     | qw   | cos(R/2).cos(P/2).cos(Y/2) + sin(R/2).sin(P/2).sin(Y/2)
   *     | qx   | sin(R/2).cos(P/2).cos(Y/2) - cos(R/2).sin(P/2).sin(Y/2)
   *     | qy   | cos(R/2).sin(P/2).cos(Y/2) + sin(R/2).cos(P/2).sin(Y/2)
   * q = | qz = | cos(R/2).cos(P/2).sin(Y/2) - sin(R/2).sin(P/2).cos(Y/2)
   *
   * and α = (R + Y)/2, ß = (R - Y)/2
   * or equivalently R = α + ß, Y = α - ß
   *
   * it follows:
   * qx + qz = (cos(P/2) - sin(P/2)).sin(α)
   * qx - qz = (cos(P/2) + sin(P/2)).sin(ß)
   * qw + qy = (cos(P/2) + sin(P/2)).cos(ß)
   * qw - qy = (cos(P/2) - sin(P/2)).cos(α)
   *
   * tan(α) = (qx + qz)/(qw - qy)
   * tan(ß) = (qx - qz)/(qw + qy)
   *
   * P is deduced from the matrix representation of q:
   * q.matrix(2,2) = 2(qx.qw - qw.qy) = -sin(P).
   */

  a = atan2(qx + qz, qw - qy);
  b = atan2(qx - qz, qw + qy);

  rpy << a + b, asin(2 * (qw*qy - qz*qx)), a - b;

  if (rpy(0) > M_PI)
    rpy(0) -= 2*M_PI;
  else if (rpy(0) < -M_PI)
    rpy(0) += 2*M_PI;

  if (rpy(2) > M_PI)
    rpy(2) -= 2*M_PI;
  else if (rpy(2) < -M_PI)
    rpy(2) += 2*M_PI;

  return rpy;
}


/* --- state_s::q_cov ------------------------------------------------------ */

const Eigen::Matrix<double, 7, 7>
state_s::pq_cov(const dcov_s &cov) const
{
  /* Jacobian is (dq/dmrp)(q) = (dq/dΔq)(q).(dΔq/dmrp)(0)
   *
   * dq/dΔq = QR(q) (with QR(q) s.t Δq × q = QR(q).Δq)
   * dΔq/dmrp -> [ I₃₃/2 ] when mrp -> 0
   *             [ 0₁₃   ]
   *
   *      |  qw  qz -qy  qx
   *      | -qz  qw  qx  qy
   * QR = |  qy -qx  qw  qz
   *      | -qx -qy -qz  qw
   *
   * Joan Solà. Quaternion kinematics for the error-state KF. 2015.
   * (section 4.3.1, eq 218c).
   * https://hal.archives-ouvertes.fr/hal-01122406v3/document
   */
  double hqx = q()(0)/2., hqy = q()(1)/2., hqz = q()(2)/2., hqw = q()(3)/2.;
  Eigen::Matrix<double, 7, 6> J;

  J.block<3, 3>(0, 0).setIdentity();  J.block<3, 3>(0, 3).setZero();
  J.block<4, 3>(3, 0).setZero();      J.block<4, 3>(3, 3) <<
                                         hqw,  hqz, -hqy,
                                        -hqz,  hqw,  hqx,
                                         hqy, -hqx,  hqw,
                                        -hqx, -hqy, -hqz;

  return J * cov.block<6, 6>(0, 0) * J.transpose();
}


/* --- state_s::rpy_cov ---------------------------------------------------- */

const Eigen::Matrix<double, 3, 3>
state_s::rpy_cov(const dcov_s &cov) const
{
  double qx = q()(0), qy = q()(1), qz = q()(2), qw = q()(3);
  Eigen::Matrix<double, 3, 4> J;

  double wmy = qw - qy, wpy = qw + qy;
  double xmz = qx - qz, xpz = qx + qz;

  double da = 1./(wmy * wmy + xpz * xpz);
  double db = 1./(wpy * wpy + xmz * xmz);

  double wmyda = wmy * da, wpydb = wpy * db;
  double xpzda = xpz * da, xmzdb = xmz * db;

  double dp = 1./std::sqrt(0.25 - (qw*qy - qz*qx)*(qw*qy - qz*qx));

  /* Derivative of rpy() equations wrt. qx, qy, qz, qw */
  J <<
    wmyda + wpydb, xpzda - xmzdb, wmyda - wpydb, -xpzda - xmzdb,
    -qz*dp, qw*dp, -qx*dp, qy*dp,
    wmyda - wpydb, xpzda + xmzdb, wmyda + wpydb, -xpzda + xmzdb;

  return J * pq_cov(cov).block<4, 4>(3, 3) * J.transpose();
}


/* --- measure_s::vector --------------------------------------------------- */

/* default to Toulouse */
Eigen::Matrix<double, 3, 1> measure_s::magdir =
  Eigen::Matrix<double, 3, 1>(23.816e-6, -0.410e-6, -39.829e-6).normalized();

measure_s::vector_s
measure_s::vector() const
{
  vector_s m(sizem);
  int j = 0;

  if (pm.present) { m.segment<3>(j) = pm.data; j+=3; }
  if (qm.present) { m.segment<4>(j) = qm.data; j+=4; }
  if (vm.present) { m.segment<3>(j) = vm.data; j+=3; }
  if (wm.present) { m.segment<3>(j) = wm.data; j+=3; }
  if (am.present) { m.segment<3>(j) = am.data; j+=3; }
  if (ipzm.present) { m.segment<1>(j) = ipzm.data; j+=1; }
  if (imm.present) { m.segment<3>(j) = imm.data; j+=3; }
  if (ivm.present) { m.segment<3>(j) = ivm.data; j+=3; }
  if (iwm.present) { m.segment<3>(j) = iwm.data; j+=3; }
  if (iam.present) { m.segment<3>(j) = iam.data; j+=3; }
  if (iawgm.present) { m.segment<3>(j) = iawgm.data; j+=3; }

  return m;
}


/* --- measure_s::noise -------------------------------------------------- */

measure_s::cov_s
measure_s::noise() const
{
  cov_s n(dsizem, dsizem);
  int j = 0;

  n.setZero();
  if (pm.present) { n.block<3, 3>(j, j) = pm.cov; j+=3; }
  if (qm.present) {
    /* Jacobian is (dmrp/dq)(q) = (dmrp/dΔq)(0).(dΔq/dq)(q)
     *
     * dmrp/dΔq -> [ 2.I₃₃ | 0₃₁ ] when dΔq -> 0
     * dΔq/dq = QR(q*) (with QR s.t Δq * q = QR(q).Δq)
     *
     *         |  qw  qz -qy  qx
     *         | -qz  qw  qx  qy
     * QR(q) = |  qy -qx  qw  qz
     *         | -qx -qy -qz  qw
     */
    double tqx = qm.data(0)*2., tqy = qm.data(1)*2., tqz = qm.data(2)*2.;
    double tqw = qm.data(3)*2.;
    Eigen::Matrix<double, 3, 4> J;

    J <<
       tqw, -tqz,  tqy, -tqx,
       tqz,  tqw, -tqx, -tqy,
      -tqy,  tqx,  tqw, -tqz;

    n.block<3, 3>(j, j) = J * qm.cov * J.transpose();
    j += 3;
  }
  if (vm.present) { n.block<3, 3>(j, j) = vm.cov; j+=3; }
  if (wm.present) { n.block<3, 3>(j, j) = wm.cov; j+=3; }
  if (am.present) { n.block<3, 3>(j, j) = am.cov; j+=3; }
  if (ipzm.present) { n.block<1, 1>(j, j) = ipzm.cov; j+=1; }
  if (imm.present) { n.block<3, 3>(j, j) = imm.cov; j+=3; }
  if (ivm.present) { n.block<3, 3>(j, j) = ivm.cov; j+=3; }
  if (iwm.present) { n.block<3, 3>(j, j) = iwm.cov; j+=3; }
  if (iam.present) { n.block<3, 3>(j, j) = iam.cov; j+=3; }
  if (iawgm.present) { n.block<3, 3>(j, j) = iawgm.cov; j+=3; }

  return n;
}


/* --- measure_s::observe -------------------------------------------------- */

measure_s::vector_s
measure_s::observe(const state_s &in) const
{

  Eigen::Quaternion<double> q(in.q());
  vector_s o(sizem);
  int j = 0;

  if (pm.present) {
    o.segment<3>(j) = in.p() + q._transformVector(offset.p);
    j+=3;
  }
  if (qm.present) { o.segment<4>(j) = (q * offset.q.normalized()).coeffs(); j+=4;}
  //if (qm.present) { o.segment<4>(j) = q.coeffs(); j+=4;}

  if (vm.present) {
    o.segment<3>(j) = in.v() + in.w().cross(q._transformVector(offset.p));
    j+=3;
  }
  if (wm.present) { o.segment<3>(j) = in.w(); j+=3; }

  if (am.present) {
    o.segment<3>(j) =
      offset.q.conjugate()._transformVector(
        in.a() + in.w().cross(in.w().cross(q._transformVector(offset.p))));
    j+=3;
  }


  if (ipzm.present) {
    /* Z coordinate of body frame Z axis */
    double z3 = (q * offset.q).conjugate().matrix()(2, 2);
    if (std::fabs(z3) < 1e-50) z3 = std::copysign(1e-50, z3);

    o(j) = (in.p()(2) + q._transformVector(offset.p)(2)) / z3;
    j+=1;
  }

  if (imm.present) {
    o.segment<3>(j) =
      (q * offset.q).conjugate()._transformVector(magdir) * imm.data.norm();
    j+=3;
  }

  if (ivm.present) {
    o.segment<3>(j) =
      (q * offset.q).conjugate()._transformVector(
        in.v() + in.w().cross(q._transformVector(offset.p)));
    j+=3;
  }
  if (iwm.present) {
    o.segment<3>(j) = (q * offset.q).conjugate()._transformVector(in.w());
    j+=3;
  }

  if (iam.present) {
    o.segment<3>(j) =
      (q * offset.q).conjugate()._transformVector(
        in.a() + in.w().cross(in.w().cross(q._transformVector(offset.p))));
    j+=3;
  }
  if (iawgm.present) {
    o.segment<3>(j) =
      (q * offset.q).conjugate()._transformVector(
        in.a() + in.w().cross(in.w().cross(q._transformVector(offset.p)))+
        Eigen::Matrix<double, 3, 1>(0, 0, 9.81));
    j+=3;
  }

  return o;
}


/* --- measure_s::delta ---------------------------------------------------- */

measure_s::vector_s
measure_s::delta(const vector_s &s2, const vector_s &s1) const
{
  vector_s d(dsizem);
  int j = 0, k = 0;

  if (pm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (qm.present) {
    Eigen::Quaternion<double> dq =
      Eigen::Quaternion<double>(s2.segment<4>(k)) *
    Eigen::Quaternion<double>(s1.segment<4>(k)).conjugate();

    if (dq.w() >= 0)
      d.segment<3>(j) = 4. * dq.vec()/(1.+dq.w());
    else
      d.segment<3>(j) = 4. * -dq.vec()/(1.-dq.w());

    j+=3; k+=4;
  }
  if (vm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (wm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (am.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (ipzm.present) {
    d.segment<1>(j) = s2.segment<1>(k) - s1.segment<1>(k);
    j+=1; k+= 1;
  }
  if (imm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (ivm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (iwm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (iam.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }
  if (iawgm.present) {
    d.segment<3>(j) = s2.segment<3>(k) - s1.segment<3>(k);
    j+=3; k+= 3;
  }

  return d;
}
