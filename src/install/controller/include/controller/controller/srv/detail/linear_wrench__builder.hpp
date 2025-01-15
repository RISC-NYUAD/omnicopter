// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/linear_wrench.hpp"


#ifndef CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/linear_wrench__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_LinearWrench_Request_duration
{
public:
  explicit Init_LinearWrench_Request_duration(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::LinearWrench_Request duration(::controller::srv::LinearWrench_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_ramp
{
public:
  explicit Init_LinearWrench_Request_ramp(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_duration ramp(::controller::srv::LinearWrench_Request::_ramp_type arg)
  {
    msg_.ramp = std::move(arg);
    return Init_LinearWrench_Request_duration(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fz2
{
public:
  explicit Init_LinearWrench_Request_fz2(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_ramp fz2(::controller::srv::LinearWrench_Request::_fz2_type arg)
  {
    msg_.fz2 = std::move(arg);
    return Init_LinearWrench_Request_ramp(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fy2
{
public:
  explicit Init_LinearWrench_Request_fy2(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_fz2 fy2(::controller::srv::LinearWrench_Request::_fy2_type arg)
  {
    msg_.fy2 = std::move(arg);
    return Init_LinearWrench_Request_fz2(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fx2
{
public:
  explicit Init_LinearWrench_Request_fx2(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_fy2 fx2(::controller::srv::LinearWrench_Request::_fx2_type arg)
  {
    msg_.fx2 = std::move(arg);
    return Init_LinearWrench_Request_fy2(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fz1
{
public:
  explicit Init_LinearWrench_Request_fz1(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_fx2 fz1(::controller::srv::LinearWrench_Request::_fz1_type arg)
  {
    msg_.fz1 = std::move(arg);
    return Init_LinearWrench_Request_fx2(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fy1
{
public:
  explicit Init_LinearWrench_Request_fy1(::controller::srv::LinearWrench_Request & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Request_fz1 fy1(::controller::srv::LinearWrench_Request::_fy1_type arg)
  {
    msg_.fy1 = std::move(arg);
    return Init_LinearWrench_Request_fz1(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

class Init_LinearWrench_Request_fx1
{
public:
  Init_LinearWrench_Request_fx1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearWrench_Request_fy1 fx1(::controller::srv::LinearWrench_Request::_fx1_type arg)
  {
    msg_.fx1 = std::move(arg);
    return Init_LinearWrench_Request_fy1(msg_);
  }

private:
  ::controller::srv::LinearWrench_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::LinearWrench_Request>()
{
  return controller::srv::builder::Init_LinearWrench_Request_fx1();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_LinearWrench_Response_status
{
public:
  Init_LinearWrench_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::LinearWrench_Response status(::controller::srv::LinearWrench_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::LinearWrench_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::LinearWrench_Response>()
{
  return controller::srv::builder::Init_LinearWrench_Response_status();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_LinearWrench_Event_response
{
public:
  explicit Init_LinearWrench_Event_response(::controller::srv::LinearWrench_Event & msg)
  : msg_(msg)
  {}
  ::controller::srv::LinearWrench_Event response(::controller::srv::LinearWrench_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::LinearWrench_Event msg_;
};

class Init_LinearWrench_Event_request
{
public:
  explicit Init_LinearWrench_Event_request(::controller::srv::LinearWrench_Event & msg)
  : msg_(msg)
  {}
  Init_LinearWrench_Event_response request(::controller::srv::LinearWrench_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_LinearWrench_Event_response(msg_);
  }

private:
  ::controller::srv::LinearWrench_Event msg_;
};

class Init_LinearWrench_Event_info
{
public:
  Init_LinearWrench_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearWrench_Event_request info(::controller::srv::LinearWrench_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_LinearWrench_Event_request(msg_);
  }

private:
  ::controller::srv::LinearWrench_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::LinearWrench_Event>()
{
  return controller::srv::builder::Init_LinearWrench_Event_info();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__BUILDER_HPP_
