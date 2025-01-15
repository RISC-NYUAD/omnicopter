#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>           // Use Pose instead of WorldPose
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>                  // For 3D vectors
#include <gz/math/Quaternion.hh>              // For quaternion rotations
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <cmath>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Link.hh>

#define RPM2RPS 0.10472  //RPM to rad/s

struct MotorControls {
  std::string namespc;
  int num_of_props;
  double slow_down;
  double norm_to_rpm;
  double thr_const;
  double thr_to_drag;
  std::string directions_string;
  Eigen::VectorXd turning_dir;

  MotorControls() 
    : namespc("omnicopter"), num_of_props(8), slow_down(10), norm_to_rpm(30000), thr_const(4.4086e-06), thr_to_drag(0.001),
      directions_string("0 1 0 1 0 1 0 1"), turning_dir(Eigen::VectorXd::Zero(8)) {}
};

using namespace gz;
using namespace sim;
using namespace systems;

class BetaMotorPlugin : public System, public ISystemConfigure, public ISystemPreUpdate {
public: 
  rclcpp::Node::SharedPtr ros_node;
  MotorControls c_vars;
  Eigen::VectorXd latest_msg;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub;
  Entity entity;
  Entity modelEntity;     // For the model (original entity)

  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &, EventManager &) override {
    this->ros_node = rclcpp::Node::make_shared("beta_motor_plugin");
    this->entity = entity;
    this->modelEntity = entity;

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
    std::string topicName = "prop_cmd";

    cmd_sub = ros_node->create_subscription<std_msgs::msg::Float64MultiArray>(topicName, 10, std::bind(&BetaMotorPlugin::cmd_callback, this, std::placeholders::_1));
  }

void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
        return;

    rclcpp::spin_some(ros_node); // Process ROS2 callbacks

    std::string preface = "rotor_";
    std::string postface = "_joint";
    auto poseComp_model = ecm.Component<components::Pose>(modelEntity);

    for (int i = 0; i < c_vars.num_of_props; ++i) {
        // Joint name and entity
        std::string joint_name = preface + std::to_string(i) + postface;
        auto joint_entity = ecm.EntityByComponents(components::Name(joint_name));
        if (joint_entity == gz::sim::kNullEntity) {
            std::cerr << "Joint entity not found: " << joint_name << std::endl;
            continue;
        }

        // Calculate RPM and velocity
        double rpm = latest_msg(i) * RPM2RPS / c_vars.slow_down;
        double velocity = c_vars.turning_dir(i) * rpm;

        // Set joint velocity
        auto jointVelComp = ecm.Component<components::JointVelocityCmd>(joint_entity);
        if (!jointVelComp) {
            ecm.CreateComponent(joint_entity, components::JointVelocityCmd({velocity}));
        } else {
            jointVelComp->Data()[0] = velocity;
        }

        // Link entity and pose
        std::string link_name = preface + std::to_string(i);
        auto link_entity = ecm.EntityByComponents(components::Name(link_name), components::Link());
        if (link_entity == gz::sim::kNullEntity) {
            std::cerr << "Link entity not found: " << link_name << std::endl;
            continue;
        }

        auto poseComp = ecm.Component<components::Pose>(link_entity);
        if (!poseComp) {
            std::cerr << "Pose component not found for link: " << link_name << std::endl;
            continue;
        }
        auto modelPose = poseComp_model->Data();

        auto linkPose = poseComp->Data();
        // Compute thrust force
        double force = std::pow(latest_msg(i) * RPM2RPS, 2) * c_vars.thr_const;
        gz::math::Vector3d thrust_force(0, 0, force); // Thrust in local Z-axis

        auto forceInWorldFrame = modelPose.Rot().RotateVector(linkPose.Rot().RotateVector(thrust_force));
        
        // Compute drag torque
        double torque = force * c_vars.thr_to_drag;
        gz::math::Vector3d drag_torque(0, 0, -c_vars.turning_dir(i) * torque);
        auto torqueInWorldFrame = modelPose.Rot().RotateVector(linkPose.Rot().RotateVector(drag_torque));

        // Create a Link object
        gz::sim::Link link(link_entity);

        // Apply force and torque in the world frame
        link.AddWorldWrench(ecm, forceInWorldFrame, torqueInWorldFrame);

        //std::cerr << "Propeller " << i << ": Force " << forceInWorldFrame 
        //          << ", Torque " << torqueInWorldFrame << std::endl;
    }
}


void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    for (int i = 0; i < c_vars.num_of_props; i++) {
        double vel = msg->data[i]; // Get the velocity for the current propeller
        vel = std::max(vel, 0.0); // Ensure the velocity is not negative
        vel = std::min(vel, 1.0); // Ensure the velocity does not exceed 1.0
        vel *= c_vars.norm_to_rpm; // Scale the normalized velocity to RPM
        latest_msg(i) = 0.6 * latest_msg(i) + 0.4 * vel; // Smoothly update the velocity
      //latest_msg(i) = vel;
    }
  }
};

GZ_ADD_PLUGIN(BetaMotorPlugin, System, ISystemConfigure, ISystemPreUpdate)
