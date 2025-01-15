// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/Goto6DPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/goto6_d_point.hpp"


#ifndef MANEUVER__SRV__DETAIL__GOTO6_D_POINT__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__GOTO6_D_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/goto6_d_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Goto6DPoint_Request_duration
{
public:
  explicit Init_Goto6DPoint_Request_duration(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Goto6DPoint_Request duration(::maneuver::srv::Goto6DPoint_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_yaw
{
public:
  explicit Init_Goto6DPoint_Request_yaw(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Request_duration yaw(::maneuver::srv::Goto6DPoint_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Goto6DPoint_Request_duration(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_pitch
{
public:
  explicit Init_Goto6DPoint_Request_pitch(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Request_yaw pitch(::maneuver::srv::Goto6DPoint_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Goto6DPoint_Request_yaw(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_roll
{
public:
  explicit Init_Goto6DPoint_Request_roll(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Request_pitch roll(::maneuver::srv::Goto6DPoint_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Goto6DPoint_Request_pitch(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_z
{
public:
  explicit Init_Goto6DPoint_Request_z(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Request_roll z(::maneuver::srv::Goto6DPoint_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Goto6DPoint_Request_roll(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_y
{
public:
  explicit Init_Goto6DPoint_Request_y(::maneuver::srv::Goto6DPoint_Request & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Request_z y(::maneuver::srv::Goto6DPoint_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Goto6DPoint_Request_z(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

class Init_Goto6DPoint_Request_x
{
public:
  Init_Goto6DPoint_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goto6DPoint_Request_y x(::maneuver::srv::Goto6DPoint_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Goto6DPoint_Request_y(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Goto6DPoint_Request>()
{
  return maneuver::srv::builder::Init_Goto6DPoint_Request_x();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Goto6DPoint_Response_status
{
public:
  Init_Goto6DPoint_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::Goto6DPoint_Response status(::maneuver::srv::Goto6DPoint_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Goto6DPoint_Response>()
{
  return maneuver::srv::builder::Init_Goto6DPoint_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Goto6DPoint_Event_response
{
public:
  explicit Init_Goto6DPoint_Event_response(::maneuver::srv::Goto6DPoint_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Goto6DPoint_Event response(::maneuver::srv::Goto6DPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Event msg_;
};

class Init_Goto6DPoint_Event_request
{
public:
  explicit Init_Goto6DPoint_Event_request(::maneuver::srv::Goto6DPoint_Event & msg)
  : msg_(msg)
  {}
  Init_Goto6DPoint_Event_response request(::maneuver::srv::Goto6DPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Goto6DPoint_Event_response(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Event msg_;
};

class Init_Goto6DPoint_Event_info
{
public:
  Init_Goto6DPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goto6DPoint_Event_request info(::maneuver::srv::Goto6DPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Goto6DPoint_Event_request(msg_);
  }

private:
  ::maneuver::srv::Goto6DPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Goto6DPoint_Event>()
{
  return maneuver::srv::builder::Init_Goto6DPoint_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__GOTO6_D_POINT__BUILDER_HPP_
