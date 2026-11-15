#include "controller/governed_allocator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <maneuver/msg/full_pose.hpp>
#include <controller/msg/allocator_status.hpp>
#include <controller/srv/set_motor_limits.hpp>
#include <memory>

class FailureAwareNode : public rclcpp::Node {
  using Pose=maneuver::msg::FullPose;
  using Array=std_msgs::msg::Float64MultiArray;
  using Limits=controller::srv::SetMotorLimits;
  std::unique_ptr<omni::GovernedAllocator> allocator;
  omni::V8 lower,upper,nominal_upper,previous=omni::V8::Zero();
  Eigen::Vector3d p=Eigen::Vector3d::Zero(),vel=p,omega=p,pr=p,vr=p,ar=p,integral=p;
  Eigen::Vector3d kp,kv,ki,kr,kw;
  Eigen::Matrix3d R=Eigen::Matrix3d::Identity(),Rr=R,J;
  rclcpp::Time state_time{0,0,RCL_ROS_TIME},last{0,0,RCL_ROS_TIME};
  bool armed=false,state_valid=false,reference_valid=false;
  double mass,ep_limit,ev_limit,integral_limit,stale_seconds,frequency,gravity;
  rclcpp::Subscription<Pose>::SharedPtr state_sub,ref_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub;
  rclcpp::Publisher<Array>::SharedPtr thrust_pub,limits_pub;
  rclcpp::Publisher<controller::msg::AllocatorStatus>::SharedPtr status_pub;
  rclcpp::Service<Limits>::SharedPtr service;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<double> vectorParam(const std::string& key,size_t count){
    auto a=declare_parameter<std::vector<double>>(key,std::vector<double>{});
    if(a.size()!=count)throw std::runtime_error("Invalid/missing "+key);
    for(double v:a)if(!std::isfinite(v))throw std::runtime_error("Nonfinite "+key);
    return a;
  }
  Eigen::Vector3d vec3(const std::string& key){auto a=vectorParam(key,3);return {a[0],a[1],a[2]};}
  static bool rotation(const geometry_msgs::msg::Quaternion& msg,Eigen::Matrix3d& r){
    Eigen::Quaterniond q(msg.w,msg.x,msg.y,msg.z);
    if(!q.coeffs().allFinite()||q.norm()<1e-8)return false;
    r=q.normalized().toRotationMatrix();return true;
  }
  void state(const Pose::SharedPtr s){
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos(s->pose.position.x,s->pose.position.y,s->pose.position.z);
    Eigen::Vector3d v(s->vel.linear.x,s->vel.linear.y,s->vel.linear.z);
    Eigen::Vector3d w(s->vel.angular.x,s->vel.angular.y,s->vel.angular.z);
    if(!pos.allFinite()||!v.allFinite()||!w.allFinite()||!rotation(s->pose.orientation,rot))return;
    p=pos;vel=v;omega=w;R=rot;state_time=rclcpp::Time(s->header.stamp,RCL_ROS_TIME);state_valid=true;
  }
  void reference(const Pose::SharedPtr s){
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos(s->pose.position.x,s->pose.position.y,s->pose.position.z);
    Eigen::Vector3d v(s->vel.linear.x,s->vel.linear.y,s->vel.linear.z);
    Eigen::Vector3d a(s->acc.linear.x,s->acc.linear.y,s->acc.linear.z);
    if(!pos.allFinite()||!v.allFinite()||!a.allFinite()||!rotation(s->pose.orientation,rot))return;
    pr=pos;vr=v;ar=a;Rr=rot;reference_valid=true;
  }
  void publishLimits(){Array msg;msg.data.insert(msg.data.end(),lower.data(),lower.data()+8);msg.data.insert(msg.data.end(),upper.data(),upper.data()+8);limits_pub->publish(msg);}
  void setLimits(const std::shared_ptr<Limits::Request> req,std::shared_ptr<Limits::Response> res){
    omni::V8 lo,hi;for(int i=0;i<8;i++){lo[i]=req->lower[i];hi[i]=req->upper[i];}
    try{
      omni::FeasibleAllocator::validateBounds(lo,hi);
      if((hi.array()>nominal_upper.array()).any())throw std::invalid_argument("Exceeds nominal limits");
      lower=lo;upper=hi;publishLimits();res->success=true;res->message="Thrust limits updated";
    }catch(const std::exception& e){res->success=false;res->message=e.what();}
  }
  void update(){
    const auto stamp=now();const double dt=(stamp-last).seconds();last=stamp;
    if(dt==0.)return; // Do not issue zero commands for a duplicate simulated timestamp.
    Array thrust;thrust.data.resize(8,0.);
    const double age=(stamp-state_time).seconds();
    if(armed&&state_valid&&reference_valid&&age>=0&&age<stale_seconds&&dt>0&&dt<stale_seconds){
      const Eigen::Vector3d ep=(pr-p).cwiseMax(-ep_limit).cwiseMin(ep_limit);
      const Eigen::Vector3d ev=(vr-vel).cwiseMax(-ev_limit).cwiseMin(ev_limit);
      const Eigen::Vector3d proposed=(integral+ep*dt).cwiseMax(-integral_limit).cwiseMin(integral_limit);
      const Eigen::Vector3d force=mass*(ar+gravity*Eigen::Vector3d::UnitZ())+kp.cwiseProduct(ep)+kv.cwiseProduct(ev)+ki.cwiseProduct(proposed);
      try{
        const auto result=allocator->step(force,R,Rr,omega,J,kr,kw,previous,lower,upper);
        if(result.accepted)integral=proposed;
        previous=result.u;
        for(int i=0;i<8;i++)thrust.data[i]=result.u[i];
        controller::msg::AllocatorStatus status;status.header.stamp=stamp;
        status.allocation_mode="governed";status.accepted=result.accepted;status.fallback=result.fallback;
        status.solver_status=result.status;status.solve_seconds=result.elapsed;
        status.candidate_constraint_violation=result.violation;status.issued_wrench_error=result.wrench_error;
        status.step_radius=result.radius;status.feasible_attitude_found=result.feasible_attitude_found;
        status.allocation_converged=result.allocation_converged;status.command_reused=result.command_reused;
        status.governor_seconds=result.governor_seconds;status.static_force_residual=result.static_force_residual;
        status.static_moment_residual=result.static_moment_residual;status.force_tracking_error=result.force_tracking_error;
        status.moment_tracking_error=result.moment_tracking_error;status.motor_bound_violation=result.motor_bound_violation;
        status.direction_violation=result.direction_violation;status.candidate_step_norm=result.candidate_step_norm;
        status.solver_seconds=result.solver_seconds;status.rejected_by_status=result.rejected_by_status;
        status.rejected_by_constraints=result.rejected_by_constraints;status.rejected_by_time=result.rejected_by_time;
        Eigen::Quaterniond q(result.attitude_target);
        status.feasible_attitude_xyzw={q.x(),q.y(),q.z(),q.w()};status.transition_reference_xyzw=status.feasible_attitude_xyzw;
        const Eigen::Vector3d allocated=R*(allocator->A.topRows<3>()*result.u);
        const Eigen::Matrix3d E=.5*(result.attitude_target.transpose()*R-R.transpose()*result.attitude_target);
        const Eigen::Vector3d er(E(2,1),E(0,2),E(1,0));
        const Eigen::Vector3d moment=omega.cross(J*omega)-kr.cwiseProduct(er)-kw.cwiseProduct(omega);
        const Eigen::Vector3d ef=allocator->A.topRows<3>()*result.u-R.transpose()*force;
        const Eigen::Vector3d em=allocator->A.bottomRows<3>()*result.u-moment;
        for(int i=0;i<3;i++){
          status.desired_force_world[i]=force[i];status.allocated_force_world[i]=allocated[i];
          status.wrench_slack[i]=ef[i];status.wrench_slack[i+3]=em[i];
          status.desired_moment_body[i]=moment[i];status.force_residual_body[i]=ef[i];status.moment_residual_body[i]=em[i];
        }
        for(int i=0;i<8;i++)status.thrust[i]=result.u[i];
        status_pub->publish(status);
        if(!result.feasible_attitude_found&&force.norm()>1e-8)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"No feasible target found; retaining previous target");
        if(result.command_reused)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Allocator reused bounded previous command");
      }catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Allocator: %s",e.what());previous.setZero();}
    }else{previous.setZero();integral.setZero();}
    thrust_pub->publish(thrust);
  }
 public:
  FailureAwareNode():Node("failure_controller"){
    auto a=vectorParam("allocation",48);omni::A68 A=Eigen::Map<const Eigen::Matrix<double,6,8,Eigen::RowMajor>>(a.data());
    auto j=vectorParam("inertia",9);J=Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(j.data());
    if(!J.isApprox(J.transpose(),1e-9)||Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(J).eigenvalues().minCoeff()<=0)throw std::runtime_error("Invalid inertia");
    omni::Options o;o.beta=declare_parameter("beta",1e-5);o.gamma=declare_parameter("gamma",.1);
    o.max_seconds=declare_parameter("max_solve_seconds",.002);o.max_evaluations=declare_parameter("max_evaluations",80);o.tolerance=declare_parameter("constraint_tolerance",1e-6);
    omni::RecoverySettings recovery;recovery.reserve=declare_parameter("actuator_reserve",1.5);
    recovery.force_weight=declare_parameter("allocation_force_weight",100.);recovery.moment_weight=declare_parameter("allocation_moment_weight",10.);
    recovery.warm_budget=declare_parameter("governor_warm_budget",.001);recovery.seed_budget=declare_parameter("governor_seed_budget",.002);recovery.max_evaluations=declare_parameter("recovery_max_evaluations",120);
    if(declare_parameter<std::string>("allocation_mode","governed")!="governed")throw std::runtime_error("This ROS 2 node implements the adopted governed mode only");
    allocator=std::make_unique<omni::GovernedAllocator>(A,o,recovery);allocator->enabled=true;
    kp=vec3("position_kp");kv=vec3("position_kd");ki=vec3("position_ki");kr=vec3("attitude_kp");kw=vec3("attitude_kd");
    auto lo=vectorParam("thrust_min",8),hi=vectorParam("thrust_max",8);
    lower=Eigen::Map<omni::V8>(lo.data());upper=Eigen::Map<omni::V8>(hi.data());nominal_upper=upper;omni::FeasibleAllocator::validateBounds(lower,upper);
    mass=declare_parameter("mass",3.44);frequency=declare_parameter("frequency",250.);gravity=declare_parameter("gravity",9.81);
    ep_limit=declare_parameter("position_error_limit",.5);ev_limit=declare_parameter("velocity_error_limit",1.);integral_limit=declare_parameter("integral_limit",1.);stale_seconds=declare_parameter("state_timeout",.1);
    if(!(mass>0&&frequency>0&&gravity>0&&ep_limit>0&&ev_limit>0&&integral_limit>0&&stale_seconds>0)||!std::isfinite(mass+frequency+gravity+ep_limit+ev_limit+integral_limit+stale_seconds))throw std::runtime_error("Invalid feedback settings");
    state_sub=create_subscription<Pose>("/pose_full",rclcpp::QoS(10),std::bind(&FailureAwareNode::state,this,std::placeholders::_1));
    ref_sub=create_subscription<Pose>("/pose_d",rclcpp::QoS(10),std::bind(&FailureAwareNode::reference,this,std::placeholders::_1));
    arm_sub=create_subscription<std_msgs::msg::Bool>("/toggle_topic",10,[this](std_msgs::msg::Bool::SharedPtr s){armed=s->data;if(!armed){integral.setZero();previous.setZero();reference_valid=false;}});
    thrust_pub=create_publisher<Array>("/omnicopter/motor_thrust",10);
    limits_pub=create_publisher<Array>("/omnicopter/motor_limits",rclcpp::QoS(1).reliable().transient_local());
    status_pub=create_publisher<controller::msg::AllocatorStatus>("~/status",10);
    service=create_service<Limits>("~/set_motor_limits",std::bind(&FailureAwareNode::setLimits,this,std::placeholders::_1,std::placeholders::_2));
    publishLimits();last=now();
    timer=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(1./frequency),std::bind(&FailureAwareNode::update,this));
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<FailureAwareNode>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("failure_controller"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
