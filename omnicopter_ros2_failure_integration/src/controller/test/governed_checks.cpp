#include "governed_allocator.hpp"
#include "model_fixture.hpp"
#include <iostream>
#include <stdexcept>
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 omni::Options options;options.max_seconds=.1;
 omni::RecoverySettings settings;settings.reserve=1.5;settings.warm_budget=.1;settings.seed_budget=.1;
 omni::GovernedAllocator allocator(fixture::allocation(),options,settings);allocator.enabled=true;
 const Eigen::Matrix3d I=Eigen::Matrix3d::Identity(),J=fixture::inertia();
 const Eigen::Vector3d force(0,0,fixture::mass*9.81),zero=Eigen::Vector3d::Zero(),kp=Eigen::Vector3d::Constant(8.5),kd=Eigen::Vector3d::Constant(6.5);
 omni::V8 lower=omni::V8::Zero(),upper=omni::V8::Constant(fixture::thrust_max),previous=lower;
 auto healthy=allocator.step(force,I,I,zero,J,kp,kd,previous,lower,upper);
 check(healthy.feasible_attitude_found,"healthy target");
 check(healthy.attitude_target.isApprox(I,1e-10),"healthy original attitude retained");
 check((allocator.A*healthy.u-(omni::V6()<<force,zero).finished()).norm()<.001,"healthy hover wrench");

 // S1 uses motor 4, S2 motor 1 (one-based paper numbering).
 for(int motor:{3,0}){
  auto limits=upper;limits[motor]=0;
  auto result=allocator.step(force,I,I,zero,J,kp,kd,healthy.u,lower,limits);
  check(result.feasible_attitude_found,"S1/S2 feasible target");
  check(result.u[motor]==0,"S1/S2 failed motor command zero");
  check(result.static_force_residual<1e-4&&result.static_moment_residual<1e-5,"S1/S2 static certificate");
  if(motor==3)check(result.attitude_target.isApprox(I,1e-10),"S1 keeps level target");
  if(motor==0)check(Eigen::AngleAxisd(result.attitude_target).angle()>1e-3,"S2 adapts target");
 }
 for(int motor:{5,6}){
  auto limits=upper;limits[motor]=motor==5?fixture::thrust_max*.333:0;
  auto result=allocator.step(force,I,I,zero,J,kp,kd,healthy.u,lower,limits);
  check(result.feasible_attitude_found,"fault target found");
  check(result.u.allFinite()&&(result.u.array()>=lower.array()).all()&&(result.u.array()<=limits.array()).all(),"hard command limits");
  check(result.static_force_residual<1e-4&&result.static_moment_residual<1e-5,"static feasibility verified");
  check(Eigen::AngleAxisd(result.attitude_target).angle()>.1,"infeasible level attitude relaxed");
  check(std::abs(result.attitude_target.determinant()-1)<1e-10,"target proper rotation");
  if(motor==6)check(result.u[6]==0,"failed motor exactly zero");
 }
 auto restored=allocator.step(force,I,I,zero,J,kp,kd,healthy.u,lower,upper);
 check(restored.feasible_attitude_found&&restored.attitude_target.isApprox(I,1e-10),"restoration returns original reference");
 // Validate exact null-space box feasibility on known feasible wrenches.
 for(int i=0;i<8;i++){
  auto limits=upper;limits[i]=0;auto known=omni::V8::Constant(5).eval();known[i]=0;omni::V8 found;
  check(allocator.wrenchFeasible(allocator.A*known,lower,limits,found),"known feasible wrench found");
  check((allocator.A*(found-known)).norm()<1e-6,"null-space wrench preserved");
  check((found-lower).minCoeff()>-1e-7&&(limits-found).minCoeff()>-1e-7,"null-space box respected");
 }
 auto zero_force=allocator.step(zero,I,I,zero,J,kp,kd,lower,lower,upper);
 check(zero_force.u.allFinite(),"zero-force guard");
 auto all_failed=allocator.step(force,I,I,zero,J,kp,kd,healthy.u,lower,lower);
 check(!all_failed.feasible_attitude_found&&all_failed.u.isZero(0),"all-failed bounded response");
 // Original paper mode must dispatch to the original controller exactly.
 allocator.enabled=false;omni::FeasibleAllocator original(fixture::allocation(),options);
 auto before=original.step(force,I,I,zero,J,kp,kd,previous,lower,upper);
 auto after=allocator.step(force,I,I,zero,J,kp,kd,previous,lower,upper);
 check(before.status==after.status&&before.accepted==after.accepted&&(before.u-after.u).norm()<1e-9,"paper mode parity");
 bool rejected=false;auto invalid=upper;invalid[0]=-1;
 try{allocator.enabled=true;allocator.step(force,I,I,zero,J,kp,kd,previous,lower,invalid);}catch(const std::invalid_argument&){rejected=true;}
 check(rejected,"invalid bounds rejected");
 std::cout<<"PASS: nominal hover, exact reference retention, two fault targets, static residuals, hard bounds, zero failed motor, restoration, eight known-wrench feasibility cases, zero force, total loss, original paper-mode parity and bad bounds.\n";
 return 0;
}catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
