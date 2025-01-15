#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <string>
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>
#include "maneuver/msg/full_pose.hpp"

using namespace gz;
using namespace sim;
using namespace systems;

class OdometrySensor : public System, public ISystemConfigure, public ISystemPreUpdate {
public: 
  rclcpp::Node::SharedPtr ros_node;
  //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
  rclcpp::Publisher<maneuver::msg::FullPose>::SharedPtr odometry_pub;
  std::string namespc;
  uint32_t sequence;
  Entity entity;
  Entity modelEntity;     // For the model (original entity)

  // Destructor to ensure proper ROS 2 shutdown
  ~OdometrySensor() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &ecm, EventManager &) override {
    // Suppress unused parameter warning
    (void)_sdf;

    // Ensure ROS 2 context is initialized
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    this->ros_node = rclcpp::Node::make_shared("odometry_sensor");
    this->entity = entity;
    this->modelEntity = entity;

    if (_sdf->HasElement("nameSpace")) {
      namespc = _sdf->Get<std::string>("nameSpace");
    }
    // Find the base_link entity
    Entity baseLinkEntity = kNullEntity;
    ecm.Each<components::Name>(
        [&](const Entity &_entity, const components::Name *_name) -> bool {
            if (_name->Data() == "base_link") {
                baseLinkEntity = _entity;
                return false; // Stop iteration once found
            }
            return true;
        });

    if (baseLinkEntity != kNullEntity) {
        std::cerr << "Found base_link entity: " << baseLinkEntity << std::endl;

        // Create velocity components if not already present
        if (!ecm.Component<components::LinearVelocity>(baseLinkEntity)) {
            ecm.CreateComponent(baseLinkEntity, components::LinearVelocity(gz::math::Vector3d::Zero));
        }

        if (!ecm.Component<components::AngularVelocity>(baseLinkEntity)) {
            ecm.CreateComponent(baseLinkEntity, components::AngularVelocity(gz::math::Vector3d::Zero));
        }

        if (!ecm.Component<components::LinearAcceleration>(baseLinkEntity)) {
            ecm.CreateComponent(baseLinkEntity, components::LinearAcceleration(gz::math::Vector3d::Zero));
        }

        if (!ecm.Component<components::AngularAcceleration>(baseLinkEntity)) {
            ecm.CreateComponent(baseLinkEntity, components::AngularAcceleration(gz::math::Vector3d::Zero));
        }
        // Update the entity to base_link for future operations
        this->entity = baseLinkEntity;
    } else {
        std::cerr << "base_link not found under model entity: " << entity << std::endl;
    }

    sequence = 0;
    //std::string topicName = namespc + "/odometry";
    std::string topicName = "pose_full";
    //odometry_pub = ros_node->create_publisher<nav_msgs::msg::Odometry>(topicName, 1000);
    odometry_pub = ros_node->create_publisher<maneuver::msg::FullPose>(topicName, 10);

}

  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override {
    if (!ros_node) {
      RCLCPP_ERROR(rclcpp::get_logger("OdometrySensor"), "ROS node not initialized.");
      return;
    }

    rclcpp::spin_some(ros_node);
    auto poseComp = ecm.Component<components::Pose>(modelEntity);
    auto linearVelComp = ecm.Component<components::LinearVelocity>(entity);
    auto angularVelComp = ecm.Component<components::AngularVelocity>(entity);
    auto linearAccComp = ecm.Component<components::LinearAcceleration>(entity);
    auto angularAccComp = ecm.Component<components::AngularAcceleration>(entity);

    if (!poseComp || !linearVelComp || !angularVelComp || !linearAccComp || !angularAccComp)
      return;

    sequence++;
    rclcpp::Time curr_time = ros_node->now();

    maneuver::msg::FullPose full_msg;
    full_msg.header.stamp = curr_time;
    full_msg.header.frame_id = "world";

    auto pose = poseComp->Data();
    auto linearVel = linearVelComp->Data();
    auto angularVel = angularVelComp->Data();
    auto linearAcc = linearAccComp->Data();
    auto angularAcc = angularAccComp->Data();

    full_msg.pose.position.x = pose.Pos().X();
    full_msg.pose.position.y = pose.Pos().Y();
    full_msg.pose.position.z = pose.Pos().Z();

    full_msg.pose.orientation.x = pose.Rot().X();
    full_msg.pose.orientation.y = pose.Rot().Y();
    full_msg.pose.orientation.z = pose.Rot().Z();
    full_msg.pose.orientation.w = pose.Rot().W();

    auto rotation = pose.Rot();

    gz::math::Vector3d linearVelworldFrame = rotation * linearVel;
    full_msg.vel.linear.x = linearVelworldFrame.X();
    full_msg.vel.linear.y = linearVelworldFrame.Y();
    full_msg.vel.linear.z = linearVelworldFrame.Z();

    // Convert angular velocity to body frame (use quaternion conjugate to rotate it)
    //gz::math::Vector3d angularVelBodyFrame = rotation.Inverse() * angularVel;
    gz::math::Vector3d angularVelBodyFrame = angularVel;

    full_msg.vel.angular.x = angularVelBodyFrame.X();
    full_msg.vel.angular.y = angularVelBodyFrame.Y();
    full_msg.vel.angular.z = angularVelBodyFrame.Z();

    full_msg.acc.linear.x = linearAcc.X();
    full_msg.acc.linear.y = linearAcc.Y();
    full_msg.acc.linear.z = linearAcc.Z();

    full_msg.acc.angular.x = angularAcc.X();
    full_msg.acc.angular.y = angularAcc.Y();
    full_msg.acc.angular.z = angularAcc.Z();

    odometry_pub->publish(full_msg);
  }


};

GZ_ADD_PLUGIN(OdometrySensor, System, ISystemConfigure, ISystemPreUpdate)
