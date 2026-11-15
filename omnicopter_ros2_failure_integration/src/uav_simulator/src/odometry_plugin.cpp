#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <maneuver/msg/full_pose.hpp>
#include <sstream>

// Position / linear velocity at aggregate COM in world; angular velocity in body.
class OdometrySensor : public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<maneuver::msg::FullPose>::SharedPtr pose_pub;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  gz::sim::Entity base=gz::sim::kNullEntity;
  gz::math::Vector3d com{0,0,0};
 public:
  void Configure(const gz::sim::Entity&entity,const std::shared_ptr<const sdf::Element>&sdf,
                 gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    node=rclcpp::Node::make_shared("odometry_sensor");
    base=gz::sim::Model(entity).LinkByName(ecm,"base_link");
    if(base==gz::sim::kNullEntity)throw std::runtime_error("base_link missing");
    if(sdf->HasElement("com_position"))com=sdf->Get<gz::math::Vector3d>("com_position");
    gz::sim::Link link(base);link.EnableVelocityChecks(ecm);link.EnableAccelerationChecks(ecm);
    pose_pub=node->create_publisher<maneuver::msg::FullPose>("/pose_full",10);
    // One authoritative clock source, directly stamped from Gazebo UpdateInfo.
    clock_pub=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::QoS(10).best_effort());
  }
  void PreUpdate(const gz::sim::UpdateInfo&,gz::sim::EntityComponentManager&)override{
    rclcpp::spin_some(node);
  }
  void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if(ns<0)return;
    const rclcpp::Time stamp(ns,RCL_ROS_TIME);
    rosgraph_msgs::msg::Clock clock;clock.clock=stamp;clock_pub->publish(clock);
    if(info.paused)return;
    gz::sim::Link link(base);
    const auto linear=link.WorldLinearVelocity(ecm),angular=link.WorldAngularVelocity(ecm);
    if(!linear||!angular)return;
    const auto pose=gz::sim::worldPose(base,ecm);
    const auto offset=pose.Rot().RotateVector(com);
    const auto position=pose.Pos()+offset;
    const auto velocity=*linear+angular->Cross(offset);
    const auto omega=pose.Rot().RotateVectorReverse(*angular);
    const auto linear_a=link.WorldLinearAcceleration(ecm),angular_a=link.WorldAngularAcceleration(ecm);
    const auto alpha=angular_a.value_or(gz::math::Vector3d::Zero);
    const auto acceleration=linear_a.value_or(gz::math::Vector3d::Zero)+alpha.Cross(offset)+angular->Cross(angular->Cross(offset));
    const auto alpha_body=pose.Rot().RotateVectorReverse(alpha);
    maneuver::msg::FullPose msg;msg.header.stamp=stamp;msg.header.frame_id="world";
    msg.pose.position.x=position.X();msg.pose.position.y=position.Y();msg.pose.position.z=position.Z();
    msg.pose.orientation.x=pose.Rot().X();msg.pose.orientation.y=pose.Rot().Y();msg.pose.orientation.z=pose.Rot().Z();msg.pose.orientation.w=pose.Rot().W();
    msg.vel.linear.x=velocity.X();msg.vel.linear.y=velocity.Y();msg.vel.linear.z=velocity.Z();
    msg.vel.angular.x=omega.X();msg.vel.angular.y=omega.Y();msg.vel.angular.z=omega.Z();
    msg.acc.linear.x=acceleration.X();msg.acc.linear.y=acceleration.Y();msg.acc.linear.z=acceleration.Z();
    msg.acc.angular.x=alpha_body.X();msg.acc.angular.y=alpha_body.Y();msg.acc.angular.z=alpha_body.Z();
    pose_pub->publish(msg);
  }
};
GZ_ADD_PLUGIN(OdometrySensor,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
