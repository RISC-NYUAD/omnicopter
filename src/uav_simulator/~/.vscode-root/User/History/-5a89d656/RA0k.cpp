#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#define RPM2RPS 0.10472

struct MotorControls {
  std::string namespc;
  int num_of_props;
  double slow_down;
  double norm_to_rpm;
  double thr_const;
  double thr_to_drag;
  std::string directions_string;
  Eigen::VectorXd turning_dir;

  MotorControls() : namespc("m100"), num_of_props(4), slow_down(10), norm_to_rpm(500), thr_const(0.0027), thr_to_drag(0.017),
                    directions_string("1 0 1 0"), turning_dir(Eigen::VectorXd::Zero(4)) { }
};

namespace gz {
namespace sim {

class BetaMotorPlugin : public System, public ISystemConfigure, public ISystemPreUpdate {
public: 
  rclcpp::Node::SharedPtr ros_node;
  MotorControls c_vars;
  Eigen::VectorXd latest_msg;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_sub;
  Entity entity;

  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &, EventManager &) override {
    this->ros_node = rclcpp::Node::make_shared("beta_motor_plugin");
    this->entity = entity;

    if (_sdf->HasElement("nameSpace"))
      c_vars.namespc = _sdf->Get<std::string>("nameSpace");
    if (_sdf->HasElement("prop_num"))
      c_vars.num_of_props = _sdf->Get<int>("prop_num");
    if (_sdf->HasElement("slowdown_sim"))
      c_vars.slow_down = _sdf->Get<double>("slowdown_sim");
    if (_sdf->HasElement("throttle"))
      c_vars.norm_to_rpm = _sdf->Get<double>("throttle");
    if (_sdf->HasElement("thrust_const"))
      c_vars.thr_const = _sdf->Get<double>("thrust_const");
    if (_sdf->HasElement("drag_const"))
      c_vars.thr_to_drag = _sdf->Get<double>("drag_const");
    if (_sdf->HasElement("turning_directions"))
      c_vars.directions_string = _sdf->Get<std::string>("turning_directions");

    std::stringstream ss(c_vars.directions_string);
    std::string temp_str;
    std::vector<std::string> words;
    while (std::getline(ss, temp_str, ' ')) {
      words.push_back(temp_str);
    }
    c_vars.turning_dir = Eigen::VectorXd::Zero(c_vars.num_of_props);
    for (int i = 0; i < c_vars.num_of_props; i++) {
      int val = std::stoi(words.at(i));
      c_vars.turning_dir(i) = (val > 0) ? 1 : -1;
    }

    latest_msg = Eigen::VectorXd::Zero(c_vars.num_of_props);
    std::string topicName = c_vars.namespc + "/command_motor_speed";
    cmd_sub = ros_node->create_subscription<geometry_msgs::msg::Vector3>(topicName, 1, std::bind(&BetaMotorPlugin::cmd_callback, this, std::placeholders::_1));
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override {
    if (!rclcpp::ok())
      return;

    std::string preface = c_vars.namespc + "::rotor_";
    std::string postface = "_joint";

    for (int i = 0; i < c_vars.num_of_props; i++) {
      auto jointVelCmd = components::JointVelocityCmd();
      jointVelCmd.Data()[0] = c_vars.turning_dir(i) * (latest_msg(i) * RPM2RPS / c_vars.slow_down);
      ecm.CreateComponent(this->entity, jointVelCmd);
    }
  }

  void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    for (int i = 0; i < c_vars.num_of_props; i++) {
      double vel = msg->z;
      vel = std::max(vel, 0.0);
      vel = std::min(vel, 1.0);
      vel *= c_vars.norm_to_rpm;
      latest_msg(i) = 0.6 * latest_msg(i) + 0.4 * vel;
    }
  }
};

GZ_ADD_PLUGIN(BetaMotorPlugin, System, ISystemConfigure, ISystemPreUpdate)

}  // namespace sim
}  // namespace gz
