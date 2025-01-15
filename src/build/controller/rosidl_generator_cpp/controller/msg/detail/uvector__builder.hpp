// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/uvector.hpp"


#ifndef CONTROLLER__MSG__DETAIL__UVECTOR__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__UVECTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/uvector__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_Uvector_value
{
public:
  Init_Uvector_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::msg::Uvector value(::controller::msg::Uvector::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::Uvector msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::Uvector>()
{
  return controller::msg::builder::Init_Uvector_value();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__UVECTOR__BUILDER_HPP_
