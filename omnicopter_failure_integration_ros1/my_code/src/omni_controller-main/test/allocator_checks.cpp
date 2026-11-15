#include "model_fixture.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
void check(bool ok,const char* what) {if(!ok) throw std::runtime_error(what);}
int main() {
 try {
  omni::Options options; options.max_seconds=0.1; options.max_evaluations=300;
  options.gamma=0.002;
  omni::FeasibleAllocator a(fixture::allocation(),options);
  check((a.A*a.P-Eigen::Matrix<double,6,6>::Identity()).norm()<1e-10,"A*P identity");
  check((a.A*a.B).norm()<1e-10,"null wrench");
  check((a.B.transpose()*a.B-Eigen::Matrix2d::Identity()).norm()<1e-10,"orthonormal null basis");
  const auto I=Eigen::Matrix3d::Identity(); const auto J=fixture::inertia();
  const Eigen::Vector3d force(0,0,fixture::mass*9.81),zero=Eigen::Vector3d::Zero(),
      kp=Eigen::Vector3d::Constant(8.5),kd=Eigen::Vector3d::Constant(6.5);
  omni::V8 lo=omni::V8::Zero(),hi=omni::V8::Constant(fixture::thrust_max),prev=omni::V8::Constant(8);
  Eigen::Matrix3d ref=Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto m=a.makeMap(force,I,ref,zero,J,kp,kd,prev,lo,hi);
  check((m.C.transpose()*m.d).norm()<1e-12,"tangent orthogonality");
  check((m.C.transpose()*m.C-Eigen::Matrix2d::Identity()).norm()<1e-12,"tangent basis");
  Eigen::VectorXd z(4); z << 0.02,-0.01,0.3,-0.5;
  Eigen::VectorXd g; m.objective(z,&g); Eigen::VectorXd c; Eigen::MatrixXd dc;
  m.constraints(z,c,dc);
  for(int k=0;k<4;k++) {
    Eigen::VectorXd p=z,n=z; p(k)+=1e-6; n(k)-=1e-6;
    double numeric=(m.objective(p)-m.objective(n))/2e-6;
    check(std::abs(numeric-g(k))<1e-6,"objective gradient");
    Eigen::VectorXd cp,cn; Eigen::MatrixXd tmp;
    m.constraints(p,cp,tmp); m.constraints(n,cn,tmp);
    check(((cp-cn)/2e-6-dc.col(k)).norm()<1e-6,"constraint gradients");
  }
  // Trial box corners are outside the disk but remain in sqrt's valid domain.
  z.head<2>().setConstant(options.max_v); omni::V8 trial; Eigen::MatrixXd du;
  m.evaluate(z,trial,du); check(trial.allFinite() && du.allFinite(),"finite trial domain");
  auto r=a.step(force,I,I,zero,J,kp,kd,prev,lo,hi);
  check(r.accepted,"nominal hover accepted");
  check((a.P*(omni::V6()<<force,zero).finished()).minCoeff()<0,"hover requires null-space redistribution");
  check((r.u-a.P*(omni::V6()<<force,zero).finished()).norm()>1,"null-space redistribution used");
  check(r.radius==0 && r.v.norm()==0,"zero-radius reduction");
  check((a.A*r.u-(omni::V6()<<force,zero).finished()).norm()<1e-5,"hover wrench preserved");
  auto moving=a.step(force,I,ref,zero,J,kp,kd,r.u,lo,hi);
  check(moving.accepted,"nonzero attitude step accepted");
  check(moving.v.norm()<=moving.radius+1e-6,"step disk");
  check(std::abs(moving.direction.norm()-1)<1e-12,"unit direction");
  int fault_accepted=0;
  for(int motor=0;motor<8;motor++) {
    auto limit=hi; limit(motor)=0;
    auto fault=a.step(force,I,ref,zero,J,kp,kd,r.u,lo,limit);
    check(fault.u.allFinite(),"fault finite");
    check((fault.u.array()>=0).all() && (fault.u.array()<=limit.array()).all(),"fault bounds");
    check(fault.u(motor)==0,"failed motor zero");
    if(fault.accepted) fault_accepted++;
  }
  auto impossible=a.step(force,I,I,zero,J,kp,kd,r.u,lo,lo);
  check(impossible.fallback && impossible.u.norm()==0,"all motors failed fallback");
  check(impossible.wrench_error>30,"infeasible force reported");
  auto noforce=a.step(zero,I,I,zero,J,kp,kd,r.u,lo,hi);
  check(noforce.fallback && noforce.u.allFinite(),"near zero force guarded");
  auto bad=hi; bad(0)=-1; bool rejected=false;
  try {a.step(force,I,I,zero,J,kp,kd,prev,lo,bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"invalid bounds rejected");
  rejected=false;
  try {omni::FeasibleAllocator rank_bad(omni::A68::Zero());}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"rank deficient allocation rejected");
  omni::Options matlab=options; matlab.clip_reference=true;
  omni::FeasibleAllocator parity(fixture::allocation(),matlab);
  auto clipped=parity.makeMap(force,I,ref,zero,J,kp,kd,prev,lo,hi);
  check(std::abs(clipped.reference.norm()-matlab.max_v)<1e-12,"MATLAB clipped reference mode");
  a.options.max_seconds=1e-12;
  auto deadline=a.step(force,I,ref,zero,J,kp,kd,r.u,lo,hi);
  check(deadline.fallback,"time limit fallback");
  a.options.max_seconds=options.max_seconds;
  a.options.max_evaluations=1;
  auto resource=a.step(force,I,ref,zero,J,kp,kd,r.u,lo,hi);
  check(resource.fallback,"evaluation limit fallback");
  int major,minor,patch; nlopt_version(&major,&minor,&patch);
  std::cout << "NLopt " << major << '.' << minor << '.' << patch << "\n";
  std::cout << "PASS: allocation identities, tangent basis, analytic derivatives, trial domain, hover, "
               "step limit, eight single faults, impossible/zero force, bad bounds/rank, clipped-reference mode, time/evaluation limits.\n";
  std::cout << "Single-fault requests accepted: " << fault_accepted << "/8 (remaining requests use bounded fallback).\n";
  return 0;
 }catch(const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n'; return 1;}
}
