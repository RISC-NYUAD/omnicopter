/*                                                              -*-c++-*-
 * Copyright (c) 2015-2016,2018-2019,2021-2023 LAAS/CNRS
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
#ifndef H_UKF
#define H_UKF

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ukf {

  /* state interface */
  struct state_s : public Eigen::Matrix<double, 3+4 + 3+3 + 3, 1>
  {
    enum {
      dof = RowsAtCompileTime-1,
      dim = RowsAtCompileTime
    };

    typedef Eigen::Matrix<double, dim, 1> vector_s;
    typedef Eigen::Matrix<double, dim, dim> cov_s;

    typedef Eigen::Matrix<double, dof, 1> delta_s;
    typedef Eigen::Matrix<double, dof, dof> dcov_s;

    state_s():
      vector_s() {}
    state_s(double v):
      vector_s(vector_s::Constant(v)) {}
    template <typename M> state_s(const Eigen::EigenBase<M> &in):
      vector_s(in) {}

    void normalize() {
      q() = Eigen::Quaternion<double>(q()).normalized().coeffs();
    }

    static const state_s Identity() {
      state_s i(0.);
      i.q() = Eigen::Quaternion<double>::Identity().coeffs();
      return i;
    }

    enum prediction_model {
      constant_velocity,
      constant_acceleration
    };
    static prediction_model pmodel;
    static double max_dadt;
    static double max_dwdt;

    static state_s transition(const state_s &in, double dt);
    dcov_s process_noise(double dt) const;
    static state_s add_delta(const state_s &s, const delta_s &ds);
    static delta_s delta(const state_s &s1, const state_s &s2);

    const Eigen::Matrix<double, 3, 1> p() const { return segment<3>(0); }
    Eigen::VectorBlock<vector_s, 3> p() { return segment<3>(0); }

    const Eigen::Matrix<double, 4, 1> q() const { return segment<4>(3); }
    Eigen::VectorBlock<vector_s, 4> q() { return segment<4>(3); }

    const Eigen::Matrix<double, 3, 1> rpy() const;

    const Eigen::Matrix<double, 3, 1> v() const { return segment<3>(7); }
    Eigen::VectorBlock<vector_s, 3> v() { return segment<3>(7); }

    const Eigen::Matrix<double, 3, 1> w() const { return segment<3>(10); }
    Eigen::VectorBlock<vector_s, 3> w() { return segment<3>(10); }

    const Eigen::Matrix<double, 3, 1> a() const { return segment<3>(13); }
    Eigen::VectorBlock<vector_s, 3> a() { return segment<3>(13); }

    const Eigen::Matrix<double, 7, 7> pq_cov(const dcov_s &cov) const;
    const Eigen::Matrix<double, 3, 3> rpy_cov(const dcov_s &cov) const;
    const Eigen::Matrix<double, 6, 6> vw_cov(const dcov_s &cov) const {
      return cov.block<6, 6>(6, 6);
    }
    const Eigen::Matrix<double, 3, 3> a_cov(const dcov_s &cov) const {
      return cov.block<3, 3>(12, 12);
    }
  };


  /* measurement interface */
  class measure_s {
    template<int D> struct data_s {
      bool present;
      Eigen::Matrix<double, D, 1> data;
      Eigen::Matrix<double, D, D> cov;
    };

    data_s<3> pm, vm, wm, am;
    data_s<1> ipzm;
    data_s<3> imm, ivm, iwm, iam, iawgm;
    data_s<4> qm;
    int sizem, dsizem;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    enum { dim = 3+4 + 3 + 3+3+3+3 + 3+3+3 };

    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, dim, 1> vector_s;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0,
                          dim, dim> cov_s;

    static Eigen::Matrix<double, 3, 1> magdir;

    struct {
      Eigen::Matrix<double, 3, 1> p;
      Eigen::Quaternion<double> q;
    } offset;

    measure_s() { reset(); }

    void reset() {
      pm.present = vm.present = wm.present = am.present = false;
      ipzm.present = imm.present = false;
      ivm.present = iwm.present = iam.present = iawgm.present = false;
      qm.present = false;
    }
    void collect() {
      sizem = dsizem = 0;
      if (pm.present) { sizem += 3; dsizem += 3; }
      if (qm.present) { sizem += 4; dsizem += 3; }
      if (vm.present) { sizem += 3; dsizem += 3; }
      if (wm.present) { sizem += 3; dsizem += 3; }
      if (am.present) { sizem += 3; dsizem += 3; }
      if (ipzm.present) { sizem += 1; dsizem += 1; }
      if (imm.present) { sizem += 3; dsizem += 3; }
      if (ivm.present) { sizem += 3; dsizem += 3; }
      if (iwm.present) { sizem += 3; dsizem += 3; }
      if (iam.present) { sizem += 3; dsizem += 3; }
      if (iawgm.present) { sizem += 3; dsizem += 3; }
    }

    int size() const { return sizem; }
    int delta_size() const { return dsizem; }
    vector_s vector() const;
    cov_s noise() const;
    vector_s observe(const state_s &in) const;
    vector_s delta(const vector_s &s2, const vector_s &s1) const;

    Eigen::Matrix<double, 3,1> &p() { pm.present = true; return pm.data; }
    Eigen::Matrix<double, 3,3> &p_cov() { return pm.cov; }

    Eigen::Matrix<double, 4,1> &q() { qm.present = true; return qm.data; }
    Eigen::Matrix<double, 4,4> &q_cov() { return qm.cov; }

    Eigen::Matrix<double, 3,1> &v() { vm.present = true; return vm.data; }
    Eigen::Matrix<double, 3,3> &v_cov() { return vm.cov; }

    Eigen::Matrix<double, 3,1> &w() { wm.present = true; return wm.data; }
    Eigen::Matrix<double, 3,3> &w_cov() { return wm.cov; }

    Eigen::Matrix<double, 3,1> &a() { am.present = true; return am.data; }
    Eigen::Matrix<double, 3,3> &a_cov() { return am.cov; }

    Eigen::Matrix<double, 1,1> &ipz() { ipzm.present = true; return ipzm.data; }
    Eigen::Matrix<double, 1,1> &ipz_cov() { return ipzm.cov; }

    Eigen::Matrix<double, 3,1> &im() { imm.present = true; return imm.data; }
    Eigen::Matrix<double, 3,3> &im_cov() { return imm.cov; }

    Eigen::Matrix<double, 3,1> &iv() { ivm.present = true; return ivm.data; }
    Eigen::Matrix<double, 3,3> &iv_cov() { return ivm.cov; }

    Eigen::Matrix<double, 3,1> &iw() { iwm.present = true; return iwm.data; }
    Eigen::Matrix<double, 3,3> &iw_cov() { return iwm.cov; }

    Eigen::Matrix<double, 3,1> &ia() { iam.present = true; return iam.data; }
    Eigen::Matrix<double, 3,3> &ia_cov() { return iam.cov; }

    Eigen::Matrix<double, 3,1> &iawg() {
      iawgm.present = true; return iawgm.data;
    }
    Eigen::Matrix<double, 3,3> &iawg_cov() { return iawgm.cov; }
  };


  /* unscented kalman filter */
  class filter_s
  {
    class impl_s;
    impl_s *impl;

   public:
    measure_s measure;

    filter_s();
    ~filter_s();

    void set(const state_s &state, state_s::dcov_s &cov);
    void predict(double dt);
    void update(state_s &state, state_s::dcov_s &cov);
  };
}

#endif /* H_UKF */
