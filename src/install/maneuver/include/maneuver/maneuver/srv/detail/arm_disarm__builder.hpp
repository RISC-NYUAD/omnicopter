// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/ArmDisarm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/arm_disarm.hpp"


#ifndef MANEUVER__SRV__DETAIL__ARM_DISARM__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__ARM_DISARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/arm_disarm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::ArmDisarm_Request>()
{
  return ::maneuver::srv::ArmDisarm_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_ArmDisarm_Response_success
{
public:
  Init_ArmDisarm_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::ArmDisarm_Response success(::maneuver::srv::ArmDisarm_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::ArmDisarm_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::ArmDisarm_Response>()
{
  return maneuver::srv::builder::Init_ArmDisarm_Response_success();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_ArmDisarm_Event_response
{
public:
  explicit Init_ArmDisarm_Event_response(::maneuver::srv::ArmDisarm_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::ArmDisarm_Event response(::maneuver::srv::ArmDisarm_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::ArmDisarm_Event msg_;
};

class Init_ArmDisarm_Event_request
{
public:
  explicit Init_ArmDisarm_Event_request(::maneuver::srv::ArmDisarm_Event & msg)
  : msg_(msg)
  {}
  Init_ArmDisarm_Event_response request(::maneuver::srv::ArmDisarm_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ArmDisarm_Event_response(msg_);
  }

private:
  ::maneuver::srv::ArmDisarm_Event msg_;
};

class Init_ArmDisarm_Event_info
{
public:
  Init_ArmDisarm_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmDisarm_Event_request info(::maneuver::srv::ArmDisarm_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ArmDisarm_Event_request(msg_);
  }

private:
  ::maneuver::srv::ArmDisarm_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::ArmDisarm_Event>()
{
  return maneuver::srv::builder::Init_ArmDisarm_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__ARM_DISARM__BUILDER_HPP_
