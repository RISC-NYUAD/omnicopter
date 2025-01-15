// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/Pose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/pose.hpp"


#ifndef CONTROLLER__MSG__DETAIL__POSE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_Pose_yaw
{
public:
  explicit Init_Pose_yaw(::controller::msg::Pose & msg)
  : msg_(msg)
  {}
  ::controller::msg::Pose yaw(::controller::msg::Pose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

class Init_Pose_pitch
{
public:
  explicit Init_Pose_pitch(::controller::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_yaw pitch(::controller::msg::Pose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Pose_yaw(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

class Init_Pose_roll
{
public:
  explicit Init_Pose_roll(::controller::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_pitch roll(::controller::msg::Pose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Pose_pitch(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

class Init_Pose_z
{
public:
  explicit Init_Pose_z(::controller::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_roll z(::controller::msg::Pose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Pose_roll(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

class Init_Pose_y
{
public:
  explicit Init_Pose_y(::controller::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_z y(::controller::msg::Pose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose_z(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

class Init_Pose_x
{
public:
  Init_Pose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_y x(::controller::msg::Pose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose_y(msg_);
  }

private:
  ::controller::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::Pose>()
{
  return controller::msg::builder::Init_Pose_x();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__POSE__BUILDER_HPP_
