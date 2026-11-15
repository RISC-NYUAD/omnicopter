#pragma once
#include <Eigen/Dense>
#include <nlopt.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace omni {
using V8 = Eigen::Matrix<double,8,1>;
using A68 = Eigen::Matrix<double,6,8>;
using V6 = Eigen::Matrix<double,6,1>;
inline Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
  Eigen::Matrix3d s; s << 0,-a.z(),a.y(),a.z(),0,-a.x(),-a.y(),a.x(),0; return s;
}

struct Options {
  double alpha=1.0, beta=1e-5, gamma=0.0, max_v=0.1;
  double tolerance=1e-6, max_seconds=0.002;
  int max_evaluations=80;
  bool clip_reference=false; // true: supplied MATLAB; false: paper Eq. (19)
};
struct Result {
  V8 u=V8::Zero();
  Eigen::Vector2d v=Eigen::Vector2d::Zero();
  Eigen::Vector3d direction=Eigen::Vector3d::UnitZ();
  bool accepted=false, fallback=true;
  int status=0;
  double violation=0, elapsed=0, wrench_error=0, radius=0;
  // Diagnostics only. Motor violation is in N; direction violation is squared
  // tangent-coordinate norm. solver_seconds includes setup and validation.
  double motor_bound_violation=0, direction_violation=0;
  double candidate_step_norm=0, solver_seconds=0;
  bool rejected_by_status=false, rejected_by_constraints=false, rejected_by_time=false;
};

// u is individual rotor thrust [N], A*u=[body force N; body moment Nm].
// Constant nominal A is retained for failures; u_max[i]=0 represents a stopped
// motor. This avoids assuming full rank of a failed-motor reduced matrix.
class FeasibleAllocator {
 public:
  A68 A;
  Eigen::Matrix<double,8,6> P;
  Eigen::Matrix<double,8,2> B;
  Options options;
  explicit FeasibleAllocator(const A68& allocation, Options opts=Options())
      : A(allocation), options(opts) {
    if (!A.allFinite() || !(options.max_v>0 && options.max_v<0.5) ||
        !(options.alpha>0 && options.beta>0 && options.gamma>=0) ||
        !std::isfinite(options.alpha+options.beta+options.gamma) ||
        !(options.tolerance>0 && options.max_seconds>0) ||
        !std::isfinite(options.tolerance+options.max_seconds) || options.max_evaluations<1)
      throw std::invalid_argument("Invalid allocator configuration");
    Eigen::JacobiSVD<A68> svd(A,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if(svd.singularValues()(0)<=1e-12 || svd.singularValues()(5)<=1e-8*svd.singularValues()(0))
      throw std::invalid_argument("Nominal allocation must have row rank six");
    P=svd.solve(Eigen::Matrix<double,6,6>::Identity());
    B=svd.matrixV().rightCols<2>();
  }
  struct Map {
    Eigen::Matrix<double,8,3> Pf;
    Eigen::Matrix<double,8,2> M, B;
    Eigen::Matrix<double,3,2> C;
    Eigen::Vector3d d;
    V8 base, previous, lower, upper;
    Eigen::Vector2d reference;
    double force=0, radius=0;
    bool moving=false;
    Options opts;
    int size() const {return moving?4:2;}
    void evaluate(const Eigen::VectorXd& z,V8& u,Eigen::MatrixXd& du) const {
      Eigen::Vector2d v=Eigen::Vector2d::Zero();
      if(moving) v=z.head<2>();
      // Component box limits keep every SLSQP trial in ||v|| < 1,
      // including infeasible trial points outside the step disk.
      const double s=std::sqrt(1-v.squaredNorm());
      u=Pf*(force*(C*v+d*s))+base+M*v+B*z.tail<2>();
      du.resize(8,size());
      if(moving) du.leftCols(2)=Pf*force*(C-d*v.transpose()/s)+M;
      du.rightCols(2)=B;
    }
    double objective(const Eigen::VectorXd& z,Eigen::VectorXd* gradient=nullptr) const {
      V8 u; Eigen::MatrixXd du; evaluate(z,u,du);
      Eigen::Vector2d v=Eigen::Vector2d::Zero(); if(moving) v=z.head<2>();
      const Eigen::Vector2d x=z.tail<2>();
      if(gradient) {
        *gradient=opts.gamma*du.transpose()*(u-previous);
        if(moving) gradient->head<2>()+=opts.alpha*(v-reference);
        gradient->tail<2>()+=opts.beta*x;
      }
      return 0.5*(opts.alpha*(v-reference).squaredNorm()+opts.beta*x.squaredNorm()+
                  opts.gamma*(u-previous).squaredNorm());
    }
    void constraints(const Eigen::VectorXd& z,Eigen::VectorXd& c,Eigen::MatrixXd& dc) const {
      V8 u; Eigen::MatrixXd du; evaluate(z,u,du);
      c.resize(moving?17:16); dc.setZero(c.size(),size());
      c.head<8>()=u-upper; c.segment<8>(8)=lower-u;
      dc.topRows(8)=du; dc.middleRows(8,8)=-du;
      if(moving) {
        c(16)=z.head<2>().squaredNorm()-radius*radius;
        dc.block(16,0,1,2)=2*z.head<2>().transpose();
      }
    }
  };
  static double objectiveCallback(unsigned n,const double* z,double* g,void* p) {
    auto& m=*static_cast<Map*>(p);
    Eigen::Map<const Eigen::VectorXd> x(z,n); Eigen::VectorXd grad;
    const double value=m.objective(x,g?&grad:nullptr);
    if(g) Eigen::Map<Eigen::VectorXd>(g,n)=grad;
    return value;
  }
  static void constraintCallback(unsigned count,double* values,unsigned n,const double* z,
                                 double* gradient,void* p) {
    auto& m=*static_cast<Map*>(p); Eigen::VectorXd c; Eigen::MatrixXd dc;
    m.constraints(Eigen::Map<const Eigen::VectorXd>(z,n),c,dc);
    Eigen::Map<Eigen::VectorXd>(values,count)=c;
    if(gradient) Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
        (gradient,count,n)=dc;
  }
  Map makeMap(const Eigen::Vector3d& force_world,const Eigen::Matrix3d& R,
              const Eigen::Matrix3d& reference,const Eigen::Vector3d& omega,
              const Eigen::Matrix3d& J,const Eigen::Vector3d& kp,const Eigen::Vector3d& kd,
              const V8& previous,const V8& lower,const V8& upper) const {
    Map m; m.opts=options; m.Pf=P.leftCols<3>(); m.B=B;
    m.force=force_world.norm(); m.previous=previous; m.lower=lower; m.upper=upper;
    m.d=R.transpose()*force_world/m.force;
    Eigen::Vector3d axis=Eigen::Vector3d::UnitZ();
    if(std::abs(m.d.z())>0.9) axis=Eigen::Vector3d::UnitX();
    m.C.col(0)=m.d.cross(axis).normalized();
    m.C.col(1)=m.d.cross(m.C.col(0)).normalized();
    const Eigen::Vector3d dr=reference.transpose()*force_world/m.force;
    m.reference=m.C.transpose()*dr;
    m.radius=std::min(m.reference.norm(),options.max_v);
    if(options.clip_reference && m.reference.norm()>options.max_v)
      m.reference*=options.max_v/m.reference.norm();
    m.moving=m.radius>1e-8;
    const Eigen::Matrix3d E=0.5*(reference.transpose()*R-R.transpose()*reference);
    // Current-minus-reference attitude error. Standard Euler compensation,
    // +omega x J*omega, intentionally corrects the MATLAB sign discrepancy.
    Eigen::Vector3d moment=omega.cross(J*omega)-kd.cwiseProduct(omega);
    moment.z()-=kp.z()*E(1,0);
    m.base=P.rightCols<3>()*moment;
    Eigen::Matrix3d H=Eigen::Matrix3d::Identity(); H(2,2)=0;
    m.M=-P.rightCols<3>()*kp.asDiagonal()*H*skew(m.d)*m.C;
    return m;
  }
  static void validateBounds(const V8& lower,const V8& upper) {
    if(!lower.allFinite() || !upper.allFinite() || (lower.array()<0).any() ||
       (upper.array()<lower.array()).any()) throw std::invalid_argument("Invalid thrust bounds");
  }
  // Bounded least squares fallback: position-force residual has a greater
  // weight than moment residual. It does NOT preserve the requested wrench.
  V8 bestEffort(const V6& desired,const V8& lower,const V8& upper,const V8& previous) const {
    A68 weighted=A; weighted.topRows<3>()*=10;
    V6 target=desired; target.head<3>()*=10;
    const Eigen::Matrix<double,8,8> H=weighted.transpose()*weighted+
        1e-7*Eigen::Matrix<double,8,8>::Identity();
    const V8 b=weighted.transpose()*target+1e-7*previous;
    const double L=H.cwiseAbs().rowwise().sum().maxCoeff();
    V8 u=previous.cwiseMax(lower).cwiseMin(upper), y=u;
    double t=1;
    for(int i=0;i<300;i++) {
      const V8 next=(y-(H*y-b)/L).cwiseMax(lower).cwiseMin(upper);
      const double tn=0.5*(1+std::sqrt(1+4*t*t));
      y=next+(t-1)/tn*(next-u); u=next; t=tn;
    }
    return u;
  }
  Result step(const Eigen::Vector3d& force_world,const Eigen::Matrix3d& R,
              const Eigen::Matrix3d& reference,const Eigen::Vector3d& omega,
              const Eigen::Matrix3d& J,const Eigen::Vector3d& kp,const Eigen::Vector3d& kd,
              const V8& previous,const V8& lower,const V8& upper) const {
    validateBounds(lower,upper);
    Result out; auto start=std::chrono::steady_clock::now();
    if(!force_world.allFinite() || !R.allFinite() || !reference.allFinite() ||
       !omega.allFinite() || !J.allFinite() || !kp.allFinite() || !kd.allFinite() || !previous.allFinite())
      throw std::invalid_argument("Nonfinite controller input");
    V6 desired; desired.head<3>()=R.transpose()*force_world;
    const Eigen::Matrix3d E=0.5*(reference.transpose()*R-R.transpose()*reference);
    Eigen::Vector3d error(E(2,1),E(0,2),E(1,0));
    desired.tail<3>()=omega.cross(J*omega)-kd.cwiseProduct(omega)-kp.cwiseProduct(error);
    if(force_world.norm()>1e-8) {
      Map m=makeMap(force_world,R,reference,omega,J,kp,kd,previous,lower,upper);
      out.radius=m.radius;
      Eigen::VectorXd guess=Eigen::VectorXd::Zero(m.size());
      guess.tail<2>()=B.transpose()*(previous-P*desired);
      std::vector<double> z(guess.data(),guess.data()+guess.size());
      struct Handle {
        nlopt_opt p;
        explicit Handle(unsigned n):p(nlopt_create(NLOPT_LD_SLSQP,n)) {}
        ~Handle(){if(p) nlopt_destroy(p);}
      } solver(m.size());
      if(!solver.p) throw std::runtime_error("Could not allocate nonlinear solver");
      std::vector<double> lb(m.size(),-HUGE_VAL),ub(m.size(),HUGE_VAL);
      if(m.moving) for(int i=0;i<2;i++){lb[i]=-m.radius; ub[i]=m.radius;}
      std::vector<double> tolerances(m.moving?17:16,options.tolerance);
      auto require=[](nlopt_result code) {
        if(code<0) throw std::runtime_error("Invalid nonlinear solver setup");
      };
      require(nlopt_set_lower_bounds(solver.p,lb.data()));
      require(nlopt_set_upper_bounds(solver.p,ub.data()));
      require(nlopt_set_min_objective(solver.p,objectiveCallback,&m));
      require(nlopt_add_inequality_mconstraint(solver.p,tolerances.size(),constraintCallback,&m,tolerances.data()));
      require(nlopt_set_xtol_abs1(solver.p,1e-8)); require(nlopt_set_ftol_abs(solver.p,1e-10));
      require(nlopt_set_maxeval(solver.p,options.max_evaluations));
      require(nlopt_set_maxtime(solver.p,options.max_seconds));
      double cost; out.status=static_cast<int>(nlopt_optimize(solver.p,z.data(),&cost));
      Eigen::Map<const Eigen::VectorXd> solution(z.data(),z.size());
      Eigen::MatrixXd du,dc; Eigen::VectorXd c;
      m.evaluate(solution,out.u,du); m.constraints(solution,c,dc);
      out.violation=c.allFinite()?std::max(0.0,c.maxCoeff()):HUGE_VAL;
      // Resource limits, exceptions and failed verification enter fallback.
      // NLopt's time limit is cooperative, not a hard real-time guarantee.
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      out.accepted=out.status>0 && out.status<5 && out.u.allFinite() && c.allFinite() &&
          out.violation<=options.tolerance && elapsed<=options.max_seconds;
      // Gather diagnostics AFTER the original acceptance decision, without
      // changing its tolerances, status policy, or elapsed-time criterion.
      out.solver_seconds=elapsed;
      out.motor_bound_violation=c.head(16).allFinite()?std::max(0.0,c.head(16).maxCoeff()):HUGE_VAL;
      if(m.moving) {
        out.direction_violation=std::isfinite(c(16))?std::max(0.0,c(16)):HUGE_VAL;
        out.candidate_step_norm=solution.head<2>().norm();
      }
      out.rejected_by_status=!(out.status>0 && out.status<5);
      out.rejected_by_constraints=!(out.u.allFinite() && c.allFinite() && out.violation<=options.tolerance);
      out.rejected_by_time=!(elapsed<=options.max_seconds);
      if(out.accepted) {
        if(m.moving) out.v=solution.head<2>();
        out.direction=m.C*out.v+m.d*std::sqrt(1-out.v.squaredNorm());
        // Roundoff cleanup only, never a repair of an infeasible iterate.
        out.u=out.u.cwiseMax(lower).cwiseMin(upper);
      }
    }
    if(!out.accepted) out.u=bestEffort(desired,lower,upper,previous);
    out.fallback=!out.accepted;
    out.wrench_error=(A*out.u-desired).norm();
    out.elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    return out;
  }
};
} // namespace omni
