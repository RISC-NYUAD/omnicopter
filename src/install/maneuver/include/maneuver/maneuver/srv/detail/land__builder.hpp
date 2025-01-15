// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/Land.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/land.hpp"


#ifndef MANEUVER__SRV__DETAIL__LAND__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__LAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/land__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Land_Request_duration_2
{
public:
  explicit Init_Land_Request_duration_2(::maneuver::srv::Land_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Land_Request duration_2(::maneuver::srv::Land_Request::_duration_2_type arg)
  {
    msg_.duration_2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Land_Request msg_;
};

class Init_Land_Request_height_2
{
public:
  explicit Init_Land_Request_height_2(::maneuver::srv::Land_Request & msg)
  : msg_(msg)
  {}
  Init_Land_Request_duration_2 height_2(::maneuver::srv::Land_Request::_height_2_type arg)
  {
    msg_.height_2 = std::move(arg);
    return Init_Land_Request_duration_2(msg_);
  }

private:
  ::maneuver::srv::Land_Request msg_;
};

class Init_Land_Request_duration_1
{
public:
  explicit Init_Land_Request_duration_1(::maneuver::srv::Land_Request & msg)
  : msg_(msg)
  {}
  Init_Land_Request_height_2 duration_1(::maneuver::srv::Land_Request::_duration_1_type arg)
  {
    msg_.duration_1 = std::move(arg);
    return Init_Land_Request_height_2(msg_);
  }

private:
  ::maneuver::srv::Land_Request msg_;
};

class Init_Land_Request_height_1
{
public:
  Init_Land_Request_height_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Land_Request_duration_1 height_1(::maneuver::srv::Land_Request::_height_1_type arg)
  {
    msg_.height_1 = std::move(arg);
    return Init_Land_Request_duration_1(msg_);
  }

private:
  ::maneuver::srv::Land_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Land_Request>()
{
  return maneuver::srv::builder::Init_Land_Request_height_1();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Land_Response_status
{
public:
  Init_Land_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::Land_Response status(::maneuver::srv::Land_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Land_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Land_Response>()
{
  return maneuver::srv::builder::Init_Land_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Land_Event_response
{
public:
  explicit Init_Land_Event_response(::maneuver::srv::Land_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Land_Event response(::maneuver::srv::Land_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Land_Event msg_;
};

class Init_Land_Event_request
{
public:
  explicit Init_Land_Event_request(::maneuver::srv::Land_Event & msg)
  : msg_(msg)
  {}
  Init_Land_Event_response request(::maneuver::srv::Land_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Land_Event_response(msg_);
  }

private:
  ::maneuver::srv::Land_Event msg_;
};

class Init_Land_Event_info
{
public:
  Init_Land_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Land_Event_request info(::maneuver::srv::Land_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Land_Event_request(msg_);
  }

private:
  ::maneuver::srv::Land_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Land_Event>()
{
  return maneuver::srv::builder::Init_Land_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__LAND__BUILDER_HPP_
