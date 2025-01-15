// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/LiftOff.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/lift_off.hpp"


#ifndef MANEUVER__SRV__DETAIL__LIFT_OFF__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__LIFT_OFF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/lift_off__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_LiftOff_Request_duration
{
public:
  explicit Init_LiftOff_Request_duration(::maneuver::srv::LiftOff_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::LiftOff_Request duration(::maneuver::srv::LiftOff_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Request msg_;
};

class Init_LiftOff_Request_height
{
public:
  Init_LiftOff_Request_height()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LiftOff_Request_duration height(::maneuver::srv::LiftOff_Request::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_LiftOff_Request_duration(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::LiftOff_Request>()
{
  return maneuver::srv::builder::Init_LiftOff_Request_height();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_LiftOff_Response_status
{
public:
  Init_LiftOff_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::LiftOff_Response status(::maneuver::srv::LiftOff_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::LiftOff_Response>()
{
  return maneuver::srv::builder::Init_LiftOff_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_LiftOff_Event_response
{
public:
  explicit Init_LiftOff_Event_response(::maneuver::srv::LiftOff_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::LiftOff_Event response(::maneuver::srv::LiftOff_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Event msg_;
};

class Init_LiftOff_Event_request
{
public:
  explicit Init_LiftOff_Event_request(::maneuver::srv::LiftOff_Event & msg)
  : msg_(msg)
  {}
  Init_LiftOff_Event_response request(::maneuver::srv::LiftOff_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_LiftOff_Event_response(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Event msg_;
};

class Init_LiftOff_Event_info
{
public:
  Init_LiftOff_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LiftOff_Event_request info(::maneuver::srv::LiftOff_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_LiftOff_Event_request(msg_);
  }

private:
  ::maneuver::srv::LiftOff_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::LiftOff_Event>()
{
  return maneuver::srv::builder::Init_LiftOff_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__LIFT_OFF__BUILDER_HPP_
