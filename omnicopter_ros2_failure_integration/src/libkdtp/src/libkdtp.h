/*
 * Copyright (c) 2016-2018,2020 LAAS/CNRS
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

/* Alexandre Boeuf, Juan Cortes, Rachid Alami, Thierry Simeon.  Enhancing
 * sampling-based kinodynamic motion planning for quadrotors.  IEEE/RSJ
 * International Conference on Intelligent Robots and Systems, Sep 2015,
 * Hamburg, Germany. 2015. */

#ifndef H_LIBKDTP
#define H_LIBKDTP

#include <string>
#include <vector>

namespace kdtp {

  class Dof {
   public:
    Dof(double xmin, double xmax,
        double vmax, double amax, double jmax, double smax,
        bool rotation);

    double getPositionMin() const { return xmin_; }
    double getPositionMax() const { return xmax_; }
    double getVelocityMax() const { return vmax_; }
    double getAccelerationMax() const { return amax_; }
    double getJerkMax() const { return jmax_; }
    double getSnapMax() const { return smax_; }
    bool isRotation() const { return rotation_; }

    void setPositionMin(double xmin);
    void setPositionMax(double xmax);
    void setPositionMinMax(double xmin, double xmax);
    void setVelocityMax(double vmax);
    void setAccelerationMax(double amax);
    void setJerkMax(double jmax);
    void setSnapMax(double smax);

   private:
    double xmin_, xmax_;
    double vmax_, amax_, jmax_, smax_;
    bool rotation_;
  };

  class Robot {
   public:

    Robot(const char *name) : name_(name) {}
    Robot(std::string name) : name_(name) {}

    std::string getName() const { return name_; }
    unsigned int getNbDof() const { return dofs_.size(); }
    const Dof &getDof(unsigned int i) const { return dofs_[i]; }
    Dof &getDof(unsigned int i) { return dofs_[i]; }

    void setName(char *name) { name_ = std::string(name); }
    void setName(std::string name) { name_ = name; }
    void addDof(const Dof &dof) { dofs_.push_back(dof); }

   private:
    std::vector<Dof> dofs_;
    std::string name_;
  };

  class State {
   public:

    State(const Robot &robot):
      position_(robot.getNbDof(), 0.),
      velocity_(robot.getNbDof(), 0.),
      acceleration_(robot.getNbDof(), 0.) {}

    const std::vector<double> &position() const { return position_; }
    std::vector<double> &position() { return position_; }

    const std::vector<double> &velocity() const { return velocity_; }
    std::vector<double> &velocity() { return velocity_; }

    const std::vector<double> &acceleration() const { return acceleration_; }
    std::vector<double> &acceleration() { return acceleration_; }

   private:
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> acceleration_;
  };


  class Spline {
   public:
    Spline(const Dof &dof, double init[3], double end[3]);

    double getDuration() const;
    void synchronize(double tF_des);

    double getPositionAt(double time) const;
    double getVelocityAt(double time) const;
    double getAccelerationAt(double time) const;
    double getJerkAt(double time) const;
    double getSnapAt(double time) const;
    void getAllAt(double time, double (&q)[5]) const;

    std::vector<double> getInit();
    std::vector<double> getEnd();

   private:
    int case_abc(double aB) const;
    int case_egh(double aG) const;
    void durations_and_signs_ac(double aB);
    void durations_and_signs_eh(double aG);
    double x_c(double aB);
    double x_e(double aG);
    double v_c(double aB, double new_tB);
    double v_e(double aG, double new_tG);
    void intervals_ac();
    void intervals_eh();
    double a_b(double vC);
    double a_g(double vE);
    double d_x(double vD);
    void v_optim();

    void setValues();

    unsigned int getIndexAt(double time) const;
    double getPositionAtLocal(unsigned int index, double local) const;
    double getPositionAt(unsigned int index, double time) const;
    double getVelocityAtLocal(unsigned int index, double local) const;
    double getVelocityAt(unsigned int index, double time) const;
    double getAccelerationAtLocal(unsigned int index, double local) const;
    double getAccelerationAt(unsigned int index, double time) const;
    double getJerkAtLocal(unsigned int index, double local) const;
    double getJerkAt(unsigned int index, double time) const;
    void getAllAt(unsigned int index, double time, double (&q)[5]) const;

    enum durations_index {
      A1, A2, B, C1, C2, D, E1, E2, G, H1, H2, F
    };

    enum signs_index {
      A, C, E, H
    };

    double vmax_, amax_, jmax_, smax_;
    double init_[3], end_[3];

    unsigned int phases_;
    double times_[16];
    double positions_[16];
    double velocities_[16];
    double accelerations_[16];
    double jerks_[16];
    double snaps_[16];

    double signs_[4];
    double durations_[12];

    std::vector<int> cases_abc_;
    std::vector<double> int_v_abc_;
    std::vector<double> int_a_abc_;

    std::vector<int> cases_egh_;
    std::vector<double> int_v_egh_;
    std::vector<double> int_a_egh_;

    double v_opt_;

    bool stay_still_;
  };

  class LocalPath {
   public:
    LocalPath(const Robot &robot, const State &init, const State &end,
              double duration = 0.);

    double duration() const { return duration_; }
    unsigned int getNbSplines() const { return splines_.size(); }
    void getPositionAt(double time, double *q) const;
    void getVelocityAt(double time, double *q) const;
    void getAccelerationAt(double time, double *q) const;
    void getJerkAt(double time, double *q) const;
    void getSnapAt(double time, double *q) const;
    void getAllAt(double time, double (*q)[5]) const;
    State getStateAt(double time) const;
    void setDuration(double duration);

   private:
    const Robot &robot_;
    std::vector<Spline> splines_;
    double duration_;
  };

}

#endif /* H_LIBKDTP */
