// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/GotoPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/goto_point.hpp"


#ifndef MANEUVER__SRV__DETAIL__GOTO_POINT__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__GOTO_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/goto_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_GotoPoint_Request_duration
{
public:
  explicit Init_GotoPoint_Request_duration(::maneuver::srv::GotoPoint_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::GotoPoint_Request duration(::maneuver::srv::GotoPoint_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Request msg_;
};

class Init_GotoPoint_Request_yaw
{
public:
  explicit Init_GotoPoint_Request_yaw(::maneuver::srv::GotoPoint_Request & msg)
  : msg_(msg)
  {}
  Init_GotoPoint_Request_duration yaw(::maneuver::srv::GotoPoint_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GotoPoint_Request_duration(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Request msg_;
};

class Init_GotoPoint_Request_z
{
public:
  explicit Init_GotoPoint_Request_z(::maneuver::srv::GotoPoint_Request & msg)
  : msg_(msg)
  {}
  Init_GotoPoint_Request_yaw z(::maneuver::srv::GotoPoint_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_GotoPoint_Request_yaw(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Request msg_;
};

class Init_GotoPoint_Request_y
{
public:
  explicit Init_GotoPoint_Request_y(::maneuver::srv::GotoPoint_Request & msg)
  : msg_(msg)
  {}
  Init_GotoPoint_Request_z y(::maneuver::srv::GotoPoint_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GotoPoint_Request_z(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Request msg_;
};

class Init_GotoPoint_Request_x
{
public:
  Init_GotoPoint_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GotoPoint_Request_y x(::maneuver::srv::GotoPoint_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GotoPoint_Request_y(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::GotoPoint_Request>()
{
  return maneuver::srv::builder::Init_GotoPoint_Request_x();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_GotoPoint_Response_status
{
public:
  Init_GotoPoint_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::GotoPoint_Response status(::maneuver::srv::GotoPoint_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::GotoPoint_Response>()
{
  return maneuver::srv::builder::Init_GotoPoint_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_GotoPoint_Event_response
{
public:
  explicit Init_GotoPoint_Event_response(::maneuver::srv::GotoPoint_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::GotoPoint_Event response(::maneuver::srv::GotoPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Event msg_;
};

class Init_GotoPoint_Event_request
{
public:
  explicit Init_GotoPoint_Event_request(::maneuver::srv::GotoPoint_Event & msg)
  : msg_(msg)
  {}
  Init_GotoPoint_Event_response request(::maneuver::srv::GotoPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GotoPoint_Event_response(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Event msg_;
};

class Init_GotoPoint_Event_info
{
public:
  Init_GotoPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GotoPoint_Event_request info(::maneuver::srv::GotoPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GotoPoint_Event_request(msg_);
  }

private:
  ::maneuver::srv::GotoPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::GotoPoint_Event>()
{
  return maneuver::srv::builder::Init_GotoPoint_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__GOTO_POINT__BUILDER_HPP_
