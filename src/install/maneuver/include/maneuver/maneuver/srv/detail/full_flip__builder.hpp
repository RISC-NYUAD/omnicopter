// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/FullFlip.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/full_flip.hpp"


#ifndef MANEUVER__SRV__DETAIL__FULL_FLIP__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__FULL_FLIP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/full_flip__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_FullFlip_Request_duration
{
public:
  explicit Init_FullFlip_Request_duration(::maneuver::srv::FullFlip_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::FullFlip_Request duration(::maneuver::srv::FullFlip_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Request msg_;
};

class Init_FullFlip_Request_pitch_bool
{
public:
  explicit Init_FullFlip_Request_pitch_bool(::maneuver::srv::FullFlip_Request & msg)
  : msg_(msg)
  {}
  Init_FullFlip_Request_duration pitch_bool(::maneuver::srv::FullFlip_Request::_pitch_bool_type arg)
  {
    msg_.pitch_bool = std::move(arg);
    return Init_FullFlip_Request_duration(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Request msg_;
};

class Init_FullFlip_Request_roll_bool
{
public:
  Init_FullFlip_Request_roll_bool()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FullFlip_Request_pitch_bool roll_bool(::maneuver::srv::FullFlip_Request::_roll_bool_type arg)
  {
    msg_.roll_bool = std::move(arg);
    return Init_FullFlip_Request_pitch_bool(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::FullFlip_Request>()
{
  return maneuver::srv::builder::Init_FullFlip_Request_roll_bool();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_FullFlip_Response_status
{
public:
  Init_FullFlip_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::FullFlip_Response status(::maneuver::srv::FullFlip_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::FullFlip_Response>()
{
  return maneuver::srv::builder::Init_FullFlip_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_FullFlip_Event_response
{
public:
  explicit Init_FullFlip_Event_response(::maneuver::srv::FullFlip_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::FullFlip_Event response(::maneuver::srv::FullFlip_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Event msg_;
};

class Init_FullFlip_Event_request
{
public:
  explicit Init_FullFlip_Event_request(::maneuver::srv::FullFlip_Event & msg)
  : msg_(msg)
  {}
  Init_FullFlip_Event_response request(::maneuver::srv::FullFlip_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FullFlip_Event_response(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Event msg_;
};

class Init_FullFlip_Event_info
{
public:
  Init_FullFlip_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FullFlip_Event_request info(::maneuver::srv::FullFlip_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FullFlip_Event_request(msg_);
  }

private:
  ::maneuver::srv::FullFlip_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::FullFlip_Event>()
{
  return maneuver::srv::builder::Init_FullFlip_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__FULL_FLIP__BUILDER_HPP_
