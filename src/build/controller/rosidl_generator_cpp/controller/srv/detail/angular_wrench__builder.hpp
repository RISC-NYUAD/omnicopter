// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/AngularWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/angular_wrench.hpp"


#ifndef CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/angular_wrench__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_AngularWrench_Request_duration
{
public:
  explicit Init_AngularWrench_Request_duration(::controller::srv::AngularWrench_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::AngularWrench_Request duration(::controller::srv::AngularWrench_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::AngularWrench_Request msg_;
};

class Init_AngularWrench_Request_phi2
{
public:
  explicit Init_AngularWrench_Request_phi2(::controller::srv::AngularWrench_Request & msg)
  : msg_(msg)
  {}
  Init_AngularWrench_Request_duration phi2(::controller::srv::AngularWrench_Request::_phi2_type arg)
  {
    msg_.phi2 = std::move(arg);
    return Init_AngularWrench_Request_duration(msg_);
  }

private:
  ::controller::srv::AngularWrench_Request msg_;
};

class Init_AngularWrench_Request_phi1
{
public:
  explicit Init_AngularWrench_Request_phi1(::controller::srv::AngularWrench_Request & msg)
  : msg_(msg)
  {}
  Init_AngularWrench_Request_phi2 phi1(::controller::srv::AngularWrench_Request::_phi1_type arg)
  {
    msg_.phi1 = std::move(arg);
    return Init_AngularWrench_Request_phi2(msg_);
  }

private:
  ::controller::srv::AngularWrench_Request msg_;
};

class Init_AngularWrench_Request_fz
{
public:
  Init_AngularWrench_Request_fz()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AngularWrench_Request_phi1 fz(::controller::srv::AngularWrench_Request::_fz_type arg)
  {
    msg_.fz = std::move(arg);
    return Init_AngularWrench_Request_phi1(msg_);
  }

private:
  ::controller::srv::AngularWrench_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::AngularWrench_Request>()
{
  return controller::srv::builder::Init_AngularWrench_Request_fz();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_AngularWrench_Response_status
{
public:
  Init_AngularWrench_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::AngularWrench_Response status(::controller::srv::AngularWrench_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::AngularWrench_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::AngularWrench_Response>()
{
  return controller::srv::builder::Init_AngularWrench_Response_status();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_AngularWrench_Event_response
{
public:
  explicit Init_AngularWrench_Event_response(::controller::srv::AngularWrench_Event & msg)
  : msg_(msg)
  {}
  ::controller::srv::AngularWrench_Event response(::controller::srv::AngularWrench_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::AngularWrench_Event msg_;
};

class Init_AngularWrench_Event_request
{
public:
  explicit Init_AngularWrench_Event_request(::controller::srv::AngularWrench_Event & msg)
  : msg_(msg)
  {}
  Init_AngularWrench_Event_response request(::controller::srv::AngularWrench_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_AngularWrench_Event_response(msg_);
  }

private:
  ::controller::srv::AngularWrench_Event msg_;
};

class Init_AngularWrench_Event_info
{
public:
  Init_AngularWrench_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AngularWrench_Event_request info(::controller::srv::AngularWrench_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_AngularWrench_Event_request(msg_);
  }

private:
  ::controller::srv::AngularWrench_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::AngularWrench_Event>()
{
  return controller::srv::builder::Init_AngularWrench_Event_info();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__BUILDER_HPP_
