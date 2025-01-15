// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/msg/full_pose.hpp"


#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__BUILDER_HPP_
#define MANEUVER__MSG__DETAIL__FULL_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/msg/detail/full_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace msg
{

namespace builder
{

class Init_FullPose_acc
{
public:
  explicit Init_FullPose_acc(::maneuver::msg::FullPose & msg)
  : msg_(msg)
  {}
  ::maneuver::msg::FullPose acc(::maneuver::msg::FullPose::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::msg::FullPose msg_;
};

class Init_FullPose_vel
{
public:
  explicit Init_FullPose_vel(::maneuver::msg::FullPose & msg)
  : msg_(msg)
  {}
  Init_FullPose_acc vel(::maneuver::msg::FullPose::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_FullPose_acc(msg_);
  }

private:
  ::maneuver::msg::FullPose msg_;
};

class Init_FullPose_pose
{
public:
  explicit Init_FullPose_pose(::maneuver::msg::FullPose & msg)
  : msg_(msg)
  {}
  Init_FullPose_vel pose(::maneuver::msg::FullPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_FullPose_vel(msg_);
  }

private:
  ::maneuver::msg::FullPose msg_;
};

class Init_FullPose_header
{
public:
  Init_FullPose_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FullPose_pose header(::maneuver::msg::FullPose::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FullPose_pose(msg_);
  }

private:
  ::maneuver::msg::FullPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::msg::FullPose>()
{
  return maneuver::msg::builder::Init_FullPose_header();
}

}  // namespace maneuver

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__BUILDER_HPP_
