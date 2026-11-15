#pragma once
#include "feasible_allocator.hpp"
#include <array>
#include <limits>

namespace omni {

struct RecoverySettings {
  double reserve = 0.5;          // N: planning reserve, not a new command limit
  double force_weight = 100.0;  // squared force-error weight, 1/N^2
  double moment_weight = 10.0;  // squared moment-error weight, 1/(Nm)^2
  double warm_budget = 0.001;   // seconds per warm governor solve
  double seed_budget = 0.002;   // seconds per initialization solve
  int max_evaluations = 120;
};

struct RecoveryResult : Result {
  bool governed = false;
  bool feasible_attitude_found = false;
  bool allocation_converged = false;
  bool command_reused = false;
  Eigen::Matrix3d attitude_target = Eigen::Matrix3d::Identity();
  double governor_seconds = 0;
  double static_force_residual = 0;
  double static_moment_residual = 0;
  double force_tracking_error = 0;
  double moment_tracking_error = 0;
};

// Experimental alternative to Eqs. (18)-(20): plan a statically feasible
// attitude, then allocate CURRENT force and full attitude-feedback torque.
// The original FeasibleAllocator::step remains available without changes.
class GovernedAllocator : public FeasibleAllocator {
  friend class ManuscriptAllocator; // Reuse static projection without changing governed dispatch.
 public:
  bool enabled = false;
  RecoverySettings recovery;

  explicit GovernedAllocator(const A68& allocation, Options opts = Options(),
                             RecoverySettings settings = RecoverySettings())
      : FeasibleAllocator(allocation, opts), recovery(settings) {
    if (!(recovery.reserve >= 0 && recovery.force_weight > 0 &&
          recovery.moment_weight > 0 && recovery.warm_budget > 0 &&
          recovery.seed_budget > 0) ||
        !std::isfinite(recovery.reserve + recovery.force_weight +
                       recovery.moment_weight + recovery.warm_budget +
                       recovery.seed_budget) || recovery.max_evaluations < 1)
      throw std::invalid_argument("Invalid governed-allocation settings");
  }

  // Exact feasibility of A*u=w, lower<=u<=upper, using the nominal 8x2 null
  // basis. A nonempty bounded 2D polygon has a vertex, including degenerate
  // line/point cases. Test origin, edge projections and line intersections.
  bool wrenchFeasible(const V6& w, const V8& lower, const V8& upper, V8& u) const {
    const V8 base = P*w;
    Eigen::Matrix<double,16,2> G;
    Eigen::Matrix<double,16,1> h;
    G.topRows<8>() = B; G.bottomRows<8>() = -B;
    h.head<8>() = upper-base; h.tail<8>() = base-lower;
    double best = HUGE_VAL;
    Eigen::Vector2d chosen = Eigen::Vector2d::Zero();
    auto consider = [&](const Eigen::Vector2d& x) {
      if (x.allFinite() && (G*x-h).maxCoeff() <= 1e-8 && x.squaredNorm() < best) {
        chosen=x; best=x.squaredNorm();
      }
    };
    consider(Eigen::Vector2d::Zero());
    for (int i=0;i<16;i++) {
      if (G.row(i).squaredNorm() > 1e-14)
        consider(G.row(i).transpose()*h(i)/G.row(i).squaredNorm());
      for (int j=0;j<i;j++) {
        Eigen::Matrix2d lines;
        lines.row(0)=G.row(i); lines.row(1)=G.row(j);
        if (std::abs(lines.determinant()) > 1e-12)
          consider(lines.inverse()*Eigen::Vector2d(h(i),h(j)));
      }
    }
    if (!std::isfinite(best)) return false;
    u=base+B*chosen;
    return (A*u-w).norm() < 1e-6;
  }

 private:
  struct Handle {
    nlopt_opt p;
    explicit Handle(unsigned n):p(nlopt_create(NLOPT_LD_SLSQP,n)) {
      if (!p) throw std::runtime_error("Could not create allocation solver");
    }
    ~Handle() { nlopt_destroy(p); }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
  };
  static void require(nlopt_result code) {
    if (code < 0) throw std::runtime_error("Invalid governed solver setup");
  }
  mutable V8 static_u_ = V8::Zero();
  mutable V8 last_lower_ = V8::Constant(-1), last_upper_ = V8::Constant(-1);
  mutable bool static_valid_ = false;
  mutable Eigen::Matrix3d last_target_ = Eigen::Matrix3d::Identity();

  struct StaticProblem {
    A68 A;
    Eigen::Vector3d direction;
    double force;
    bool sphere=false, minimum=false;
    static double objective(unsigned, const double* x, double* gradient, void* data) {
      auto& m=*static_cast<StaticProblem*>(data);
      Eigen::Map<const V8> u(x);
      V8 g; double value;
      if (m.minimum) { value=0.5*u.squaredNorm(); g=u; }
      else {
        value=-m.direction.dot(m.A.topRows<3>()*u)/m.force
              +0.5e-4*u.squaredNorm()/(m.force*m.force);
        g=-m.A.topRows<3>().transpose()*m.direction/m.force
          +1e-4*u/(m.force*m.force);
      }
      if (gradient) { Eigen::Map<V8> out(gradient); out=g; }
      return value;
    }
    static void equalities(unsigned count, double* result, unsigned,
                           const double* x, double* gradient, void* data) {
      auto& m=*static_cast<StaticProblem*>(data);
      Eigen::Map<const V8> u(x);
      Eigen::Map<Eigen::VectorXd> c(result,count);
      c.head<3>()=m.A.bottomRows<3>()*u;
      const Eigen::Vector3d f=m.A.topRows<3>()*u;
      if (m.sphere) c(3)=f.squaredNorm()/(m.force*m.force)-1;
      if (gradient) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,8,Eigen::RowMajor>> D(gradient,count,8);
        D.topRows(3)=m.A.bottomRows<3>();
        if (m.sphere) D.row(3)=2*f.transpose()*m.A.topRows<3>()/(m.force*m.force);
      }
    }
  };

  bool solveStatic(StaticProblem& problem, V8& u, const V8& lower,
                   const V8& upper, double budget) const {
    Handle solver(8);
    require(nlopt_set_lower_bounds(solver.p,lower.data()));
    require(nlopt_set_upper_bounds(solver.p,upper.data()));
    require(nlopt_set_min_objective(solver.p,StaticProblem::objective,&problem));
    std::vector<double> tolerances(problem.sphere?4:3,1e-8);
    require(nlopt_add_equality_mconstraint(solver.p,tolerances.size(),
            StaticProblem::equalities,&problem,tolerances.data()));
    require(nlopt_set_ftol_abs(solver.p,1e-10));
    require(nlopt_set_xtol_abs1(solver.p,1e-8));
    require(nlopt_set_maxeval(solver.p,recovery.max_evaluations));
    require(nlopt_set_maxtime(solver.p,budget));
    double value;
    nlopt_optimize(solver.p,u.data(),&value);
    return u.allFinite() && (u-lower).minCoeff() >= -1e-7 &&
           (upper-u).minCoeff() >= -1e-7 &&
           (A.bottomRows<3>()*u).norm() < 1e-5 &&
           (!problem.sphere || std::abs((A.topRows<3>()*u).norm()-problem.force) < 1e-4);
  }

  Eigen::Matrix3d feasibleTarget(const Eigen::Vector3d& force_world,
      const Eigen::Matrix3d& reference, const Eigen::Matrix3d& current,
      const V8& lower, const V8& upper, RecoveryResult& result) const {
    const double F=force_world.norm();
    if (F <= 1e-8) {
      // Direction is undefined; retain ordinary reference attitude feedback.
      result.feasible_attitude_found=lower.isZero(1e-12);
      return reference;
    }
    V8 low=lower, high=upper;
    for (int i=0;i<8;i++) if (upper(i)>lower(i)+1e-8) {
      const double margin=std::min(recovery.reserve,0.2*(upper(i)-lower(i)));
      low(i)+=margin; high(i)-=margin;
    }
    const Eigen::Vector3d requested_direction=reference.transpose()*force_world/F;
    V6 wrench; wrench << F*requested_direction,Eigen::Vector3d::Zero();
    V8 u;
    if (wrenchFeasible(wrench,low,high,u)) {
      static_u_=u; static_valid_=true;
      last_lower_=lower; last_upper_=upper; last_target_=reference;
      result.feasible_attitude_found=true;
    } else {
      StaticProblem problem;
      problem.A=A; problem.force=F; problem.direction=requested_direction; problem.sphere=true;
      const bool changed=!lower.isApprox(last_lower_,1e-12)||!upper.isApprox(last_upper_,1e-12);
      bool found=false; double best_score=HUGE_VAL; V8 best=V8::Zero();
      if (static_valid_ && !changed) {
        u=static_u_.cwiseMax(low).cwiseMin(high);
        if (solveStatic(problem,u,low,high,recovery.warm_budget)) {
          best=u; best_score=StaticProblem::objective(8,u.data(),nullptr,&problem); found=true;
        }
      }
      if (!found && (high-low).norm()>1e-12) {
        // A low-thrust zero-moment point and support points give feasible
        // sphere starts by line intersection; initialize only when needed.
        StaticProblem minimum=problem;
        minimum.minimum=true; minimum.sphere=false;
        V8 origin=low;
        if (solveStatic(minimum,origin,low,high,recovery.seed_budget) &&
            (A.topRows<3>()*origin).norm()<F) {
          const std::array<Eigen::Vector3d,7> directions={{requested_direction,
            Eigen::Vector3d::UnitX(),-Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY(),-Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ(),-Eigen::Vector3d::UnitZ()}};
          for (const auto& direction:directions) {
            StaticProblem support=problem;
            support.sphere=false; support.direction=direction;
            V8 vertex=origin;
            if (!solveStatic(support,vertex,low,high,recovery.seed_budget)) continue;
            const Eigen::Vector3d f0=A.topRows<3>()*origin;
            const Eigen::Vector3d df=A.topRows<3>()*(vertex-origin);
            if ((f0+df).norm()<F || df.squaredNorm()<1e-8) continue;
            const double a=df.squaredNorm(),b=2*f0.dot(df),c=f0.squaredNorm()-F*F;
            const double fraction=(-b+std::sqrt(std::max(0.0,b*b-4*a*c)))/(2*a);
            if (fraction<0 || fraction>1) continue;
            u=origin+fraction*(vertex-origin);
            V8 trial=u;
            if (solveStatic(problem,trial,low,high,recovery.seed_budget)) u=trial;
            const double score=StaticProblem::objective(8,u.data(),nullptr,&problem);
            if (score<best_score) {best_score=score;best=u;found=true;}
          }
        }
      }
      last_lower_=lower; last_upper_=upper;
      result.feasible_attitude_found=found;
      if (found) {
        static_u_=best; static_valid_=true;
        const Eigen::Vector3d body_direction=(A.topRows<3>()*best).normalized();
        // Closest reference rotation satisfying R_target*body_direction=fhat.
        last_target_=Eigen::Quaterniond::FromTwoVectors(
          reference*body_direction,force_world/F).toRotationMatrix()*reference;
      } else if (!static_valid_) last_target_=current;
    }
    if (result.feasible_attitude_found) {
      result.static_force_residual=std::abs((A.topRows<3>()*static_u_).norm()-F);
      result.static_moment_residual=(A.bottomRows<3>()*static_u_).norm();
    } else {
      result.static_force_residual=HUGE_VAL;
      result.static_moment_residual=HUGE_VAL;
    }
    return last_target_;
  }

  struct WrenchProblem {
    A68 A;
    V6 target;
    V8 previous;
    double force_weight,moment_weight,beta,gamma;
    static double objective(unsigned, const double* x, double* gradient, void* data) {
      auto& m=*static_cast<WrenchProblem*>(data);
      Eigen::Map<const V8> u(x);
      const V6 residual=m.A*u-m.target;
      V6 weighted=residual;
      weighted.head<3>()*=m.force_weight; weighted.tail<3>()*=m.moment_weight;
      if (gradient) {
        Eigen::Map<V8> g(gradient);
        g=m.A.transpose()*weighted+m.beta*u+m.gamma*(u-m.previous);
      }
      return 0.5*(residual.dot(weighted)+m.beta*u.squaredNorm()
                  +m.gamma*(u-m.previous).squaredNorm());
    }
  };

 public:
  RecoveryResult step(const Eigen::Vector3d& force_world, const Eigen::Matrix3d& R,
      const Eigen::Matrix3d& reference, const Eigen::Vector3d& omega,
      const Eigen::Matrix3d& J, const Eigen::Vector3d& kp, const Eigen::Vector3d& kd,
      const V8& previous, const V8& lower, const V8& upper) const {
    RecoveryResult result;
    result.attitude_target=reference;
    if (!enabled) {
      static_cast<Result&>(result)=FeasibleAllocator::step(
          force_world,R,reference,omega,J,kp,kd,previous,lower,upper);
      result.allocation_converged=result.status>0 && result.status<5;
      return result;
    }
    validateBounds(lower,upper);
    if (!force_world.allFinite() || !R.allFinite() || !reference.allFinite() ||
        !omega.allFinite() || !J.allFinite() || !kp.allFinite() || !kd.allFinite() ||
        !previous.allFinite()) throw std::invalid_argument("Nonfinite recovery input");
    result.governed=true;
    const auto start=std::chrono::steady_clock::now();
    result.attitude_target=feasibleTarget(force_world,reference,R,lower,upper,result);
    result.governor_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    const Eigen::Matrix3d E=0.5*(result.attitude_target.transpose()*R-R.transpose()*result.attitude_target);
    const Eigen::Vector3d error(E(2,1),E(0,2),E(1,0));
    WrenchProblem problem;
    problem.A=A;
    problem.target << R.transpose()*force_world,
        omega.cross(J*omega)-kp.cwiseProduct(error)-kd.cwiseProduct(omega);
    problem.previous=previous;
    problem.force_weight=recovery.force_weight; problem.moment_weight=recovery.moment_weight;
    problem.beta=options.beta; problem.gamma=options.gamma;
    const V8 initial=previous.cwiseMax(lower).cwiseMin(upper);
    V8 candidate=initial;
    const auto allocation_start=std::chrono::steady_clock::now();
    if ((upper-lower).norm()>1e-12) {
      Handle solver(8);
      require(nlopt_set_lower_bounds(solver.p,lower.data()));
      require(nlopt_set_upper_bounds(solver.p,upper.data()));
      require(nlopt_set_min_objective(solver.p,WrenchProblem::objective,&problem));
      require(nlopt_set_ftol_abs(solver.p,1e-9));
      require(nlopt_set_xtol_abs1(solver.p,1e-8));
      require(nlopt_set_maxeval(solver.p,recovery.max_evaluations));
      require(nlopt_set_maxtime(solver.p,options.max_seconds));
      double value;
      result.status=static_cast<int>(nlopt_optimize(solver.p,candidate.data(),&value));
    } else result.status=1;
    result.solver_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-allocation_start).count();
    result.allocation_converged=result.status>0 && result.status<5;
    result.motor_bound_violation=candidate.allFinite()?
      std::max(0.0,std::max((candidate-upper).maxCoeff(),(lower-candidate).maxCoeff())):HUGE_VAL;
    result.violation=result.motor_bound_violation;
    // Bounds are hard. Retain an improving bounded iterate at an iteration or
    // time limit; never switch to another force/attitude objective.
    bool usable=candidate.allFinite();
    if (usable) candidate=candidate.cwiseMax(lower).cwiseMin(upper);
    if (usable) usable=WrenchProblem::objective(8,candidate.data(),nullptr,&problem)
        <= WrenchProblem::objective(8,initial.data(),nullptr,&problem)+1e-10;
    result.u=usable?candidate:initial;
    result.command_reused=!usable;
    result.accepted=usable;
    result.fallback=!usable;
    result.rejected_by_constraints=!candidate.allFinite();
    const V6 residual=A*result.u-problem.target;
    result.force_tracking_error=residual.head<3>().norm();
    result.moment_tracking_error=residual.tail<3>().norm();
    result.wrench_error=residual.norm();
    result.elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    return result;
  }
};
} // namespace omni
