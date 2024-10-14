/*
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
#include <err.h>

#include <sstream>
#include <string>
#include <iostream>

#include "ukf.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace ukf;


/* --- impl_s -------------------------------------------------------------- */

class filter_s::impl_s
{
  static const double alpha;
  static const double beta;
  static const double kappa;

  static inline double lambda() {
    return alpha * alpha * (state_s::dof + kappa) - state_s::dof;
  }
  static inline double weight_m0() {
    return lambda() / (state_s::dof + lambda());
  }
  static inline double weight_c0() {
    return weight_m0() + (1 - alpha * alpha + beta);
  }
  static inline double weight_i() {
    return 1./(2. * (state_s::dof + lambda()));
  }

  /* sigma points */
  typedef Eigen::Matrix<double, state_s::dim, 2*state_s::dof+1> sigma_s;
  typedef Eigen::Matrix<double, state_s::dof, 2*state_s::dof+1> dsigma_s;

  sigma_s sigma_points;

  /* prediction */
  state_s prediction;
  state_s::dcov_s prediction_cov;
  dsigma_s prediction_deviation;

  /* measurements */
  typedef Eigen::Matrix<double, Eigen::Dynamic, 2*state_s::dof+1,
                        0, measure_s::dim> obs_sigma_s;

  obs_sigma_s obs_deviation;
  measure_s::vector_s obs;

  measure_s::vector_s innovation;
  measure_s::cov_s innovation_cov;

  /* kalman */
  typedef Eigen::Matrix<double, state_s::dof, Eigen::Dynamic,
                        0, state_s::dof, state_s::dof> kalman_gain_s;

  kalman_gain_s cross_cov, kalman_gain;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void compute_sigma_points(const state_s &state, state_s::dcov_s &cov) {
    Eigen::LLT<state_s::dcov_s> llt((state_s::dof + lambda()) * cov);

    if (llt.info() != Eigen::Success) {
      warnx("state covariance is not positive definite ... fixing");

      Eigen::EigenSolver<state_s::dcov_s> e(cov);
      state_s::dcov_s d = e.eigenvalues().real().asDiagonal();
      state_s::dcov_s v = e.eigenvectors().real();

      {
        std::ostringstream msg;
        msg << "diagonal was [" << d.diagonal().transpose() << "]'";
        warnx("%s", msg.str().c_str());
      }
      d.diagonal() =
        d.diagonal().cwiseMax(state_s::delta_s::Constant(1e-10));
      cov = v * d * v.inverse();

      llt.compute((state_s::dof + lambda()) * cov);
      if (llt.info() != Eigen::Success) {
        warnx("state covariance cannot be fixed ... resetting");
        cov = state_s::dcov_s::Identity();
        llt.compute((state_s::dof + lambda()) * cov);
      }
    }

    state_s::dcov_s sqrtcov = llt.matrixL();

    sigma_points.col(0) = state;
    for(int i = 0; i < state_s::dof; i++) {
      sigma_points.col(2*i+1) = state_s::add_delta(state, sqrtcov.col(i));
      sigma_points.col(2*i+2) = state_s::add_delta(state, -sqrtcov.col(i));
    }

    /* store current state as current prediction, in case predict() is
     * called with a tiny dt */
    prediction = state;
    prediction_cov = cov;
  }


  void predict(double dt) {
    /* don't bother updating the state when dt is 0 */
    if (std::fabs(dt) > 0.) {
      for(int i = 0; i < 2*state_s::dof+1; i++)
        sigma_points.col(i) = state_s::transition(sigma_points.col(i), dt);

      prediction.noalias() =
        weight_m0() * sigma_points.col(0) +
        weight_i() * sigma_points.rightCols<2*state_s::dof>().rowwise().sum();
      prediction.normalize();
    }

    for(int i = 0; i < 2*state_s::dof+1; i++)
      prediction_deviation.col(i) =
        state_s::delta(sigma_points.col(i), prediction);
     
    /* don't update cov for tiny dt: in constant velocity model, the sigma
     * point spread on the acceleration is 0 and the process noise approaches
     * 0 when dt is too small, which leads to numerical unstability. */
    if (std::fabs(dt) < 1e-5 /* 10 µs */) return;

    prediction_cov.noalias() =
      weight_c0() * (
        prediction_deviation.col(0) * prediction_deviation.col(0).transpose()
        );
    for(int i = 1; i < 2*state_s::dof+1; i++)
      prediction_cov.noalias() +=
        weight_i() * (
          prediction_deviation.col(i) * prediction_deviation.col(i).transpose()
          );

    prediction_cov += prediction.process_noise(dt);
  }


  void update(const measure_s &measure, state_s &state, state_s::dcov_s &cov) {
    int dmdim;

    dmdim = measure.delta_size();
    if (!dmdim) {
      state = prediction;
      cov = prediction_cov;
      return;
    }

    obs_deviation.resize(dmdim, 2*state_s::dof+1);
    innovation.resize(dmdim, 1);
    innovation_cov.resize(dmdim, dmdim);
    cross_cov.resize(state_s::dof, dmdim);
    kalman_gain.resize(state_s::dof, dmdim);
    //std::cout << prediction.transpose() << std::endl;
    obs = measure.observe(prediction);
    for(int i = 0; i < 2*state_s::dof+1; i++)
      obs_deviation.col(i) =
        measure.delta(measure.observe(sigma_points.col(i)), obs);
    
    //std::cout << "obs: " << obs.segment<4>(3).transpose() << ", P: " << measure.vector().segment<4>(3).transpose() << std::endl; 
    //std::cout << (Eigen::Quaternion<double>(measure.vector().segment<4>(3)) *
    //Eigen::Quaternion<double>(obs.segment<4>(3)).conjugate()).vec().transpose() << std::endl;

    innovation = measure.delta(measure.vector(), obs);
    //std::cout << "I: " << innovation.segment<3>(3).transpose() << ", P: " << prediction.segment<4>(3).transpose() << std::endl; 
    
    innovation_cov.noalias() =
      weight_c0() * (
        obs_deviation.col(0) * obs_deviation.col(0).transpose()
        );
    for(int i = 1; i < 2*state_s::dof+1; i++)
      innovation_cov.noalias() +=
        weight_i() * (
          obs_deviation.col(i) * obs_deviation.col(i).transpose()
          );
    innovation_cov += measure.noise();

    cross_cov.noalias() =
      weight_c0() * (
        prediction_deviation.col(0) * obs_deviation.col(0).transpose()
        );
    for(int i = 1; i < 2*state_s::dof+1; i++) {
      cross_cov.noalias() +=
        weight_i() * (
          prediction_deviation.col(i) * obs_deviation.col(i).transpose()
          );
    }
    kalman_gain.noalias() = cross_cov * innovation_cov.inverse();

    state = state_s::add_delta(prediction, kalman_gain * innovation);

    cov = prediction_cov;
    /* simplified kalman_gain * innovation_cov * kalman_gain.transpose() */
    cov.noalias() -= (cross_cov * kalman_gain.transpose());
  }

};

/* --- filter_s ------------------------------------------------------------ */

const double filter_s::impl_s::alpha = 0.5;
const double filter_s::impl_s::beta = 2.;
const double filter_s::impl_s::kappa = 0.;


/* --- filter_s::filter_s -------------------------------------------------- */

filter_s::filter_s()
{
  impl = new filter_s::impl_s;
}


/* --- filter_s::~filter_s ------------------------------------------------- */

filter_s::~filter_s()
{
  delete impl;
}


/* --- filter_s::set ------------------------------------------------------- */

void
filter_s::set(const state_s &state, state_s::dcov_s &cov)
{
  impl->compute_sigma_points(state, cov);
}




/* --- filter_s::predict --------------------------------------------------- */

void
filter_s::predict(double dt)
{
  impl->predict(dt);
}


/* --- filter_s::update ---------------------------------------------------- */

void
filter_s::update(state_s &state, state_s::dcov_s &cov)
{
  measure.collect();
  impl->update(measure, state, cov);
  measure.reset();
}
