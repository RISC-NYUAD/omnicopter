/*
 * Copyright (c) 2016,2020 LAAS/CNRS
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
#include <algorithm>
#include <cmath>

#include "kdtp.h"

namespace kdtp {

  Dof::Dof(double xmin, double xmax,
           double vmax, double amax, double jmax, double smax,
           bool rotation) :
    xmin_(xmin), xmax_(xmax),
    vmax_(fabs(vmax)), amax_(fabs(amax)), jmax_(fabs(jmax)), smax_(fabs(smax)),
    rotation_(rotation)
  {
    if (xmax_ < xmin_) std::swap(xmax_, xmin_);
  }

  void Dof::setPositionMin(double xmin)
  {
    setPositionMinMax(xmin, xmax_);
  }

  void Dof::setPositionMax(double xmax)
  {
    setPositionMinMax(xmin_, xmax);
  }

  void Dof::setPositionMinMax(double xmin, double xmax)
  {
    xmin_ = xmin;
    xmax_ = xmax;
    if (xmax_ < xmin_) std::swap(xmax_, xmin_);
  }

  void Dof::setVelocityMax(double vmax)
  {
    vmax_ = fabs(vmax);
  }

  void Dof::setAccelerationMax(double amax)
  {
    amax_ = fabs(amax);
  }

  void Dof::setJerkMax(double jmax)
  {
    jmax_ = fabs(jmax);
  }

  void Dof::setSnapMax(double smax)
  {
    smax_ = fabs(smax);
  }
}
