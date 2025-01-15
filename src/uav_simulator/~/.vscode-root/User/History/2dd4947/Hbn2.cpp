#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <string>

namespace gz {
namespace sim {

class OdometrySensor : public System, public ISystemConfigure, public ISystemPreUpdate {
public: 
  rclcpp::Node::SharedPtr ros_node;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
  std::string namespc;
  uint32_t sequence;
  Entity entity;

  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &, EventManager &) override {
    this->ros_node = rclcpp::Node::make_shared("odometry_sensor");
    this->entity = entity;

    if (_sdf->HasElement("nameSpace"))
      namespc = _sdf->Get<std::string>("nameSpace");

    sequence = 0;

    std::string topicName = namespc + "/odometry";
    odometry_pub = ros_node->create_publisher<nav_msgs::msg::Odometry>(topicName, 1);
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override {
    auto poseComp = ecm.Component<components::Pose>(entity);
    auto linearVelComp = ecm.Component<components::LinearVelocity>(entity);
    auto angularVelComp = ecm.Component<components::AngularVelocity>(entity);

    if (!poseComp || !linearVelComp || !angularVelComp)
      return;

    sequence++;
    rclcpp::Time curr_time = ros_node->now();

    nav_msgs::msg::Odometry full_msg;
    full_msg.header.stamp = curr_time;
    full_msg.header.frame_id = "world";
    full_msg.header.seq = sequence;

    auto pose = poseComp->Data();
    auto linearVel = linearVelComp->Data();
    auto angularVel = angularVelComp->Data();

    full_msg.pose.pose.position.x = pose.Pos().X();
    full_msg.pose.pose.position.y = pose.Pos().Y();
    full_msg.pose.pose.position.z = pose.Pos().Z();

    full_msg.pose.pose.orientation.x = pose.Rot().X();
    full_msg.pose.pose.orientation.y = pose.Rot().Y();
    full_msg.pose.pose.orientation.z = pose.Rot().Z();
    full_msg.pose.pose.orientation.w = pose.Rot().W();

    full_msg.twist.twist.linear.x = linearVel.X();
    full_msg.twist.twist.linear.y = linearVel.Y();
    full_msg.twist.twist.linear.z = linearVel.Z();

    full_msg.twist.twist.angular.x = angularVel.X();
    full_msg.twist.twist.angular.y = angularVel.Y();
    full_msg.twist.twist.angular.z = angularVel.Z();

    odometry_pub->publish(full_msg);
  }
};

GZ_ADD_PLUGIN(OdometrySensor, System, ISystemConfigure, ISystemPreUpdate)

}  // namespace sim
}  // namespace gz
