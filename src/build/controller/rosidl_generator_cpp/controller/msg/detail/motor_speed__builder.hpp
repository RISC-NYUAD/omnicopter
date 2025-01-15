// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/motor_speed.hpp"


#ifndef CONTROLLER__MSG__DETAIL__MOTOR_SPEED__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__MOTOR_SPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/motor_speed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_MotorSpeed_velocity
{
public:
  explicit Init_MotorSpeed_velocity(::controller::msg::MotorSpeed & msg)
  : msg_(msg)
  {}
  ::controller::msg::MotorSpeed velocity(::controller::msg::MotorSpeed::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::MotorSpeed msg_;
};

class Init_MotorSpeed_name
{
public:
  Init_MotorSpeed_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSpeed_velocity name(::controller::msg::MotorSpeed::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MotorSpeed_velocity(msg_);
  }

private:
  ::controller::msg::MotorSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::MotorSpeed>()
{
  return controller::msg::builder::Init_MotorSpeed_name();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOTOR_SPEED__BUILDER_HPP_
