// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/RotateTo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/rotate_to.hpp"


#ifndef MANEUVER__SRV__DETAIL__ROTATE_TO__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__ROTATE_TO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/rotate_to__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_RotateTo_Request_duration
{
public:
  explicit Init_RotateTo_Request_duration(::maneuver::srv::RotateTo_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::RotateTo_Request duration(::maneuver::srv::RotateTo_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Request msg_;
};

class Init_RotateTo_Request_yaw
{
public:
  explicit Init_RotateTo_Request_yaw(::maneuver::srv::RotateTo_Request & msg)
  : msg_(msg)
  {}
  Init_RotateTo_Request_duration yaw(::maneuver::srv::RotateTo_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_RotateTo_Request_duration(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Request msg_;
};

class Init_RotateTo_Request_pitch
{
public:
  explicit Init_RotateTo_Request_pitch(::maneuver::srv::RotateTo_Request & msg)
  : msg_(msg)
  {}
  Init_RotateTo_Request_yaw pitch(::maneuver::srv::RotateTo_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RotateTo_Request_yaw(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Request msg_;
};

class Init_RotateTo_Request_roll
{
public:
  Init_RotateTo_Request_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RotateTo_Request_pitch roll(::maneuver::srv::RotateTo_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RotateTo_Request_pitch(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::RotateTo_Request>()
{
  return maneuver::srv::builder::Init_RotateTo_Request_roll();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_RotateTo_Response_status
{
public:
  Init_RotateTo_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::RotateTo_Response status(::maneuver::srv::RotateTo_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::RotateTo_Response>()
{
  return maneuver::srv::builder::Init_RotateTo_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_RotateTo_Event_response
{
public:
  explicit Init_RotateTo_Event_response(::maneuver::srv::RotateTo_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::RotateTo_Event response(::maneuver::srv::RotateTo_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Event msg_;
};

class Init_RotateTo_Event_request
{
public:
  explicit Init_RotateTo_Event_request(::maneuver::srv::RotateTo_Event & msg)
  : msg_(msg)
  {}
  Init_RotateTo_Event_response request(::maneuver::srv::RotateTo_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RotateTo_Event_response(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Event msg_;
};

class Init_RotateTo_Event_info
{
public:
  Init_RotateTo_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RotateTo_Event_request info(::maneuver::srv::RotateTo_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RotateTo_Event_request(msg_);
  }

private:
  ::maneuver::srv::RotateTo_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::RotateTo_Event>()
{
  return maneuver::srv::builder::Init_RotateTo_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__ROTATE_TO__BUILDER_HPP_
