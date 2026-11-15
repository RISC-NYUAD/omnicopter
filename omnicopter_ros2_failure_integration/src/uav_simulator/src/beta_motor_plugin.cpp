#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "thrust_dynamics.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

class BetaMotorPlugin : public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
  using Array=std_msgs::msg::Float64MultiArray;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<Array>::SharedPtr command_sub,limits_sub;
  rclcpp::Publisher<Array>::SharedPtr actual_pub;
  gz::sim::Entity model_entity=gz::sim::kNullEntity;
  std::array<double,8> command{},actual{},lower{},upper{},sign{};
  double k=4.4086e-6,drag=.001,rpm_max=30000.,slowdown=40.,tau_up=.15,tau_down=.21,timeout=.2;
  double last_command=-1.;
  bool direct=false,have_limits=false;
  double sim_seconds=0.;
 public:
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element>&sdf,
                 gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    node=rclcpp::Node::make_shared("beta_motor_plugin");model_entity=entity;
    auto read=[&](const char*key,double&v){if(sdf->HasElement(key))v=sdf->Get<double>(key);};
    read("thrust_const",k);read("drag_const",drag);read("throttle",rpm_max);read("slowdown_sim",slowdown);
    read("thrust_tau_up",tau_up);read("thrust_tau_down",tau_down);read("command_timeout",timeout);
    if(sdf->HasElement("direct_thrust"))direct=sdf->Get<bool>("direct_thrust");
    if(sdf->HasElement("prop_num")&&sdf->Get<int>("prop_num")!=8)throw std::runtime_error("Expected eight rotors");
    if(!(k>0&&drag>=0&&rpm_max>0&&slowdown>0&&tau_up>0&&tau_down>0&&timeout>0)||
       !std::isfinite(k+drag+rpm_max+slowdown+tau_up+tau_down+timeout))throw std::runtime_error("Invalid motor dynamics");
    std::istringstream signs(sdf->Get<std::string>("turning_directions"));
    const double physical_max=k*std::pow(rpm_max*.10472,2);
    for(int i=0;i<8;i++){int s;if(!(signs>>s))throw std::runtime_error("Invalid rotor signs");sign[i]=s>0?1.:-1.;upper[i]=direct?0.:physical_max;}
    limits_sub=node->create_subscription<Array>("/omnicopter/motor_limits",rclcpp::QoS(1).reliable().transient_local(),
      [this,physical_max](Array::SharedPtr msg){
        if(msg->data.size()!=16)return;
        for(int i=0;i<8;i++)if(!std::isfinite(msg->data[i])||!std::isfinite(msg->data[i+8])||
          msg->data[i]<0||msg->data[i+8]<msg->data[i]||msg->data[i+8]>physical_max+1e-6)return;
        for(int i=0;i<8;i++){lower[i]=msg->data[i];upper[i]=msg->data[i+8];}have_limits=true;
      });
    command_sub=node->create_subscription<Array>(direct?"/omnicopter/motor_thrust":"/prop_cmd",10,
      [this](Array::SharedPtr msg){
        if(msg->data.size()!=8)return;
        for(double v:msg->data)if(!std::isfinite(v))return;
        for(int i=0;i<8;i++)command[i]=direct?std::max(0.,msg->data[i]):
          k*std::pow(std::clamp(msg->data[i],0.,1.)*rpm_max*.10472,2);
        last_command=sim_seconds;
      });
    actual_pub=node->create_publisher<Array>("/omnicopter/motor_thrust_actual",10);
    RCLCPP_INFO(node->get_logger(),"Motor dynamics tau_up=%.3f, tau_down=%.3f, direct thrust=%d",tau_up,tau_down,direct);
  }
  void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
    sim_seconds=std::chrono::duration<double>(info.simTime).count();
    rclcpp::spin_some(node);
    if(info.paused)return;
    const double dt=std::chrono::duration<double>(info.dt).count();
    if(dt<0){actual.fill(0.);command.fill(0.);last_command=-1.;return;}
    gz::sim::Model model(model_entity);
    const bool fresh=last_command>=0&&sim_seconds-last_command>=0&&sim_seconds-last_command<timeout;
    for(int i=0;i<8;i++){
      const double target=fresh&&(!direct||have_limits)?std::clamp(command[i],lower[i],upper[i]):0.;
      actual[i]=omni_sim::advanceThrust(actual[i],target,dt,tau_up,tau_down);
      const auto link_entity=model.LinkByName(ecm,"rotor_"+std::to_string(i));
      const auto joint_entity=model.JointByName(ecm,"rotor_"+std::to_string(i)+"_joint");
      if(link_entity==gz::sim::kNullEntity||joint_entity==gz::sim::kNullEntity)continue;
      const double speed=sign[i]*std::sqrt(actual[i]/k)/slowdown;
      auto*component=ecm.Component<gz::sim::components::JointVelocityCmd>(joint_entity);
      if(!component)ecm.CreateComponent(joint_entity,gz::sim::components::JointVelocityCmd({speed}));
      else component->Data()={speed};
      const auto rotation=gz::sim::worldPose(link_entity,ecm).Rot();
      const auto force=rotation.RotateVector(gz::math::Vector3d(0,0,actual[i]));
      const auto torque=rotation.RotateVector(gz::math::Vector3d(0,0,-sign[i]*drag*actual[i]));
      gz::sim::Link(link_entity).AddWorldWrench(ecm,force,torque);
    }
    Array msg;msg.data.assign(actual.begin(),actual.end());actual_pub->publish(msg);
  }
};
GZ_ADD_PLUGIN(BetaMotorPlugin,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
