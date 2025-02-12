/*
 * Copyright (c) 2016-2019 LAAS/CNRS
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
#include "kdtp.h"

namespace kdtp {

  LocalPath::LocalPath(const Robot &robot,
                       const State &init, const State &end, double duration):
    robot_(robot)
  {
    unsigned int nbDof = robot_.getNbDof();
    double init_[3], end_[3];
    double d;

    duration_ = duration;
    splines_.reserve(nbDof);

    for(unsigned int i=0; i<nbDof; i++) {
      init_[0] = init.position()[i];
      init_[1] = init.velocity()[i];
      init_[2] = init.acceleration()[i];

      end_[0] = end.position()[i];
      end_[1] = end.velocity()[i];
      end_[2] = end.acceleration()[i];

      splines_.push_back(Spline(robot_.getDof(i), init_, end_));

      /* get maximum duration */
      d = splines_[i].getDuration();
      if (duration_ < d) duration_ = d;
    }

    /* synchronize to same (max) duration */
    for(unsigned int i = 0; i < splines_.size(); i++)
      splines_[i].synchronize(duration_);

    /* get the actual max duration (due to dichotomy in synchronize(), all
     * duration won't be strictly equal) */
    for(unsigned int i = 0; i < splines_.size(); i++) {
      d = splines_[i].getDuration();
      if (duration_ < d) duration_ = d;
    }
  }

  void
  LocalPath::getPositionAt(double time, double *q) const
  {
    for(unsigned int i = 0; i < splines_.size(); i++)
      q[i] = splines_[i].getPositionAt(time);
  }

  void
  LocalPath::getVelocityAt(double time, double *q) const
  {
    for(unsigned int i = 0; i < splines_.size(); i++)
      q[i] = splines_[i].getVelocityAt(time);
  }

  void
  LocalPath::getAccelerationAt(double time, double *q) const
  {
    for(unsigned int i = 0; i< splines_.size(); i++)
      q[i] = splines_[i].getAccelerationAt(time);
  }

  void
  LocalPath::getJerkAt(double time, double *q) const
  {
    for(unsigned int i = 0; i < splines_.size(); i++)
      q[i] = splines_[i].getJerkAt(time);
  }

  void
  LocalPath::getSnapAt(double time, double *q) const
  {
    for(unsigned int i = 0; i < splines_.size(); i++)
      q[i] = splines_[i].getSnapAt(time);
  }

  void
  LocalPath::getAllAt(double time, double (*q)[5]) const
  {
    for(unsigned int i = 0; i < splines_.size(); i++)
      splines_[i].getAllAt(time, q[i]);
  }

  State
  LocalPath::getStateAt(double time) const
  {
    State state(robot_);

    for(unsigned int i=0; i < splines_.size(); i++) {
      state.position()[i] = splines_[i].getPositionAt(time);
      state.velocity()[i] = splines_[i].getVelocityAt(time);
      state.acceleration()[i] = splines_[i].getAccelerationAt(time);
    }

    return state;
  }

  void
  LocalPath::setDuration(double duration)
  {
    duration_ = duration;
    for(unsigned int i = 0; i < splines_.size(); i++)
      splines_[i].synchronize(duration_);
  }
}
