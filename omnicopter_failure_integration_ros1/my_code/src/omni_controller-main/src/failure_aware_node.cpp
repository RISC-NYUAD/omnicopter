#include "manuscript_allocator.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <omni_firmware/FullPose.h>
#include <controller/SetMotorLimits.h>
#include <controller/AllocatorStatus.h>
#include <memory>
#include <string>

class FailureAwareNode {
  ros::NodeHandle nh, priv;
  ros::Subscriber state_sub, ref_sub, arm_sub;
  ros::Publisher thrust_pub, status_pub, limits_pub;
  ros::ServiceServer service;
  std::unique_ptr<omni::ManuscriptAllocator> allocator;
  std::string allocation_mode;
  omni::V8 lower, upper, nominal_upper, previous=omni::V8::Zero();
  Eigen::Vector3d p=Eigen::Vector3d::Zero(),vel=p,omega=p,pr=p,vr=p,ar=p, integral=p;
  Eigen::Vector3d kp,kv,ki,kr,kw;
  Eigen::Matrix3d R=Eigen::Matrix3d::Identity(), Rr=R, J;
  ros::Time state_time;
  bool armed=false, state_valid=false, reference_valid=false;
  double mass, ep_limit, ev_limit, integral_limit, stale_seconds, frequency;
  std::vector<double> vectorParam(const std::string& key,size_t size) {
    std::vector<double> a;
    if(!priv.getParam(key,a) || a.size()!=size) throw std::runtime_error("Missing/invalid "+key);
    for(double v:a) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite "+key);
    return a;
  }
  Eigen::Vector3d vec3(const std::string& key) {
    const auto a=vectorParam(key,3); return Eigen::Vector3d(a[0],a[1],a[2]);
  }
  static bool rotation(const geometry_msgs::Quaternion& msg,Eigen::Matrix3d& rotation) {
    Eigen::Quaterniond q(msg.w,msg.x,msg.y,msg.z);
    if(!q.coeffs().allFinite() || q.norm()<1e-8) return false;
    rotation=q.normalized().toRotationMatrix(); return true;
  }
  void state(const omni_firmware::FullPose::ConstPtr& s) {
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos(s->pose.position.x,s->pose.position.y,s->pose.position.z);
    Eigen::Vector3d v(s->vel.linear.x,s->vel.linear.y,s->vel.linear.z);
    Eigen::Vector3d w(s->vel.angular.x,s->vel.angular.y,s->vel.angular.z);
    if(!pos.allFinite() || !v.allFinite() || !w.allFinite() || !rotation(s->pose.orientation,rot)) return;
    p=pos; vel=v; omega=w; R=rot;
    state_time=s->header.stamp;
    if(state_time.isZero()) state_time=ros::Time::now();
    state_valid=true;
  }
  void reference(const omni_firmware::FullPose::ConstPtr& s) {
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos(s->pose.position.x,s->pose.position.y,s->pose.position.z);
    Eigen::Vector3d v(s->vel.linear.x,s->vel.linear.y,s->vel.linear.z);
    Eigen::Vector3d a(s->acc.linear.x,s->acc.linear.y,s->acc.linear.z);
    if(!pos.allFinite() || !v.allFinite() || !a.allFinite() || !rotation(s->pose.orientation,rot)) return;
    pr=pos; vr=v; ar=a; Rr=rot; reference_valid=true;
  }
  void arm(const std_msgs::Bool::ConstPtr& s) {
    armed=s->data;
    if(!armed) {allocator->resetTransition(); integral.setZero(); previous.setZero(); reference_valid=false;}
  }
  void publishLimits() {
    std_msgs::Float64MultiArray msg;
    msg.data.insert(msg.data.end(),lower.data(),lower.data()+8);
    msg.data.insert(msg.data.end(),upper.data(),upper.data()+8);
    limits_pub.publish(msg);
  }
  bool setLimits(controller::SetMotorLimits::Request& req,controller::SetMotorLimits::Response& res) {
    omni::V8 lo,hi;
    for(int i=0;i<8;i++){lo(i)=req.lower[i]; hi(i)=req.upper[i];}
    try {
      omni::FeasibleAllocator::validateBounds(lo,hi);
      if((hi.array()>nominal_upper.array()).any()) throw std::invalid_argument("Exceeds configured nominal bounds");
      lower=lo; upper=hi; publishLimits(); res.success=true; res.message="Thrust limits updated";
    } catch(const std::exception& e) {res.success=false; res.message=e.what();}
    return true;
  }
 public:
  FailureAwareNode():priv("~") {
    auto values=vectorParam("allocation",48);
    omni::A68 A=Eigen::Map<const Eigen::Matrix<double,6,8,Eigen::RowMajor>>(values.data());
    auto inertia=vectorParam("inertia",9);
    J=Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(inertia.data());
    if(!J.isApprox(J.transpose(),1e-9) || Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(J).eigenvalues().minCoeff()<=0)
      throw std::runtime_error("Inertia must be symmetric positive definite");
    omni::Options o;
    priv.param("alpha",o.alpha,1.0); priv.param("beta",o.beta,1e-5); priv.param("gamma",o.gamma,0.0);
    priv.param("max_v",o.max_v,0.1); priv.param("clip_reference",o.clip_reference,false);
    priv.param("max_solve_seconds",o.max_seconds,0.002); priv.param("max_evaluations",o.max_evaluations,80);
    priv.param("constraint_tolerance",o.tolerance,1e-6);
    priv.param<std::string>("allocation_mode",allocation_mode,"paper");
    if(allocation_mode!="paper" && allocation_mode!="governed" && allocation_mode!="manuscript")
      throw std::runtime_error("allocation_mode must be paper, governed or manuscript");
    omni::RecoverySettings recovery;
    priv.param("actuator_reserve",recovery.reserve,0.5);
    priv.param("allocation_force_weight",recovery.force_weight,100.0);
    priv.param("allocation_moment_weight",recovery.moment_weight,10.0);
    priv.param("governor_warm_budget",recovery.warm_budget,0.001);
    priv.param("governor_seed_budget",recovery.seed_budget,0.002);
    priv.param("recovery_max_evaluations",recovery.max_evaluations,120);
    omni::ManuscriptSettings joint;
    priv.param("slack_force_weight",joint.force_slack_weight,1000.0);
    priv.param("slack_moment_weight",joint.moment_slack_weight,100.0);
    priv.param("reference_max_rate",joint.reference_max_rate,0.35);
    priv.param("reference_max_acceleration",joint.reference_max_acceleration,0.7);
    priv.param("reference_gain",joint.reference_gain,2.0);
    priv.param("joint_max_evaluations",joint.max_evaluations,120);
    allocator.reset(new omni::ManuscriptAllocator(A,o,recovery,joint));
    allocator->manuscript_enabled=allocation_mode=="manuscript";
    allocator->enabled=allocation_mode=="governed";
    ROS_INFO("Allocation mode: %s",allocation_mode.c_str());
    kp=vec3("position_kp"); kv=vec3("position_kd"); ki=vec3("position_ki");
    kr=vec3("attitude_kp"); kw=vec3("attitude_kd");
    auto lo=vectorParam("thrust_min",8),hi=vectorParam("thrust_max",8);
    lower=Eigen::Map<omni::V8>(lo.data()); upper=Eigen::Map<omni::V8>(hi.data());
    omni::FeasibleAllocator::validateBounds(lower,upper); nominal_upper=upper;
    priv.param("mass",mass,3.44); priv.param("frequency",frequency,250.0);
    priv.param("position_error_limit",ep_limit,0.5); priv.param("velocity_error_limit",ev_limit,1.0);
    priv.param("integral_limit",integral_limit,1.0); priv.param("state_timeout",stale_seconds,0.1);
    if(!(mass>0 && frequency>0 && ep_limit>0 && ev_limit>0 && integral_limit>0 && stale_seconds>0) ||
       !std::isfinite(mass+frequency+ep_limit+ev_limit+integral_limit+stale_seconds))
      throw std::runtime_error("Invalid feedback/timing configuration");
    state_sub=nh.subscribe("pose_full",1,&FailureAwareNode::state,this);
    ref_sub=nh.subscribe("pose_d",1,&FailureAwareNode::reference,this);
    arm_sub=nh.subscribe("/toggle_topic",1,&FailureAwareNode::arm,this);
    thrust_pub=nh.advertise<std_msgs::Float64MultiArray>("/omnicopter/motor_thrust",1);
    status_pub=priv.advertise<controller::AllocatorStatus>("status",1);
    limits_pub=nh.advertise<std_msgs::Float64MultiArray>("/omnicopter/motor_limits",1,true);
    service=priv.advertiseService("set_motor_limits",&FailureAwareNode::setLimits,this);
    publishLimits();
  }
  void run() {
    ros::Rate rate(frequency); ros::Time last=ros::Time::now();
    while(ros::ok()) {
      ros::spinOnce(); const ros::Time now=ros::Time::now();
      const double dt=(now-last).toSec(); last=now;
      std_msgs::Float64MultiArray thrust; thrust.data.resize(8,0);
      const double age=(now-state_time).toSec();
      if(armed && state_valid && reference_valid && age>=0 && age<stale_seconds && dt>0 && dt<stale_seconds) {
        const Eigen::Vector3d ep=(pr-p).cwiseMax(-ep_limit).cwiseMin(ep_limit);
        const Eigen::Vector3d ev=(vr-vel).cwiseMax(-ev_limit).cwiseMin(ev_limit);
        const Eigen::Vector3d proposed=(integral+ep*dt).cwiseMax(-integral_limit).cwiseMin(integral_limit);
        Eigen::Vector3d force=mass*(ar+9.81*Eigen::Vector3d::UnitZ())+
            kp.cwiseProduct(ep)+kv.cwiseProduct(ev)+ki.cwiseProduct(proposed);
        try {
          auto result=allocator->step(force,R,Rr,omega,J,kr,kw,previous,lower,upper,dt);
          if(result.accepted) integral=proposed; // freeze on infeasible solve
          previous=result.u;
          for(int i=0;i<8;i++) thrust.data[i]=result.u(i);
          controller::AllocatorStatus status; status.header.stamp=now;
          status.accepted=result.accepted; status.fallback=result.fallback; status.solver_status=result.status;
          status.solve_seconds=result.elapsed; status.candidate_constraint_violation=result.violation;
          status.issued_wrench_error=result.wrench_error; status.step_radius=result.radius;
          status.allocation_mode=allocation_mode;
          const Eigen::Quaterniond transition_q(result.transition_reference);
          status.transition_reference_xyzw[0]=transition_q.x();
          status.transition_reference_xyzw[1]=transition_q.y();
          status.transition_reference_xyzw[2]=transition_q.z();
          status.transition_reference_xyzw[3]=transition_q.w();
          for(int i=0;i<6;i++)status.wrench_slack[i]=result.wrench_slack[i];
          for(int i=0;i<3;i++) {
            status.reference_rate_world[i]=result.reference_rate_world[i];
            status.local_direction_body[i]=result.direction[i];
          }
          status.joint_objective=result.joint_objective;
          status.feasible_attitude_found=result.feasible_attitude_found;
          status.allocation_converged=result.allocation_converged;
          status.command_reused=result.command_reused;
          status.governor_seconds=result.governor_seconds;
          status.static_force_residual=result.static_force_residual;
          status.static_moment_residual=result.static_moment_residual;
          status.force_tracking_error=result.force_tracking_error;
          status.moment_tracking_error=result.moment_tracking_error;
          const Eigen::Quaterniond target_q(result.attitude_target);
          status.feasible_attitude_xyzw[0]=target_q.x();
          status.feasible_attitude_xyzw[1]=target_q.y();
          status.feasible_attitude_xyzw[2]=target_q.z();
          status.feasible_attitude_xyzw[3]=target_q.w();
          status.motor_bound_violation=result.motor_bound_violation;
          status.direction_violation=result.direction_violation;
          status.candidate_step_norm=result.candidate_step_norm;
          status.solver_seconds=result.solver_seconds;
          status.rejected_by_status=result.rejected_by_status;
          status.rejected_by_constraints=result.rejected_by_constraints;
          status.rejected_by_time=result.rejected_by_time;
          const Eigen::Vector3d allocated_force=R*(allocator->A.topRows<3>()*result.u);
          for(int i=0;i<3;i++) {
            status.desired_force_world[i]=force(i);
            status.allocated_force_world[i]=allocated_force(i);
          }
          for(int i=0;i<2;i++) status.tangent_step[i]=result.v(i);
          for(int i=0;i<8;i++) status.thrust[i]=result.u(i);
          status_pub.publish(status);
          if(result.governed && !result.feasible_attitude_found && force.norm()>1e-8)
            ROS_WARN_THROTTLE(1.0,"No feasible attitude target found; retaining the last target with bounded allocation");
          if(result.governed && result.command_reused)
            ROS_WARN_THROTTLE(1.0,"Recovery allocator reused the bounded previous command: status=%d",result.status);
          if(!result.governed && result.fallback) ROS_WARN_THROTTLE(1.0,
              "Allocator fallback: status=%d, motor_N=%.6g, direction=%.6g, v=%.6g, r=%.6g, solver_ms=%.3f, reject[status,constraint,time]=[%d,%d,%d], residual=%.3g",
              result.status,result.motor_bound_violation,result.direction_violation,
              result.candidate_step_norm,result.radius,1000.0*result.solver_seconds,
              static_cast<int>(result.rejected_by_status),static_cast<int>(result.rejected_by_constraints),
              static_cast<int>(result.rejected_by_time),result.wrench_error);
        } catch(const std::exception& e) {ROS_ERROR_THROTTLE(1.0,"Allocator: %s",e.what()); previous.setZero();}
      } else {allocator->resetTransition(); previous.setZero(); integral.setZero();}
      thrust_pub.publish(thrust); rate.sleep();
    }
  }
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"failure_controller");
  try {FailureAwareNode node; node.run();}
  catch(const std::exception& e) {ROS_FATAL("%s",e.what()); return 1;}
}
