// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/error_msg.hpp"


#ifndef CONTROLLER__MSG__DETAIL__ERROR_MSG__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__ERROR_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/error_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_ErrorMsg_weight
{
public:
  explicit Init_ErrorMsg_weight(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  ::controller::msg::ErrorMsg weight(::controller::msg::ErrorMsg::_weight_type arg)
  {
    msg_.weight = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_prop
{
public:
  explicit Init_ErrorMsg_prop(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_weight prop(::controller::msg::ErrorMsg::_prop_type arg)
  {
    msg_.prop = std::move(arg);
    return Init_ErrorMsg_weight(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_exwrench
{
public:
  explicit Init_ErrorMsg_exwrench(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_prop exwrench(::controller::msg::ErrorMsg::_exwrench_type arg)
  {
    msg_.exwrench = std::move(arg);
    return Init_ErrorMsg_prop(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_wrench
{
public:
  explicit Init_ErrorMsg_wrench(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_exwrench wrench(::controller::msg::ErrorMsg::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return Init_ErrorMsg_exwrench(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_accd
{
public:
  explicit Init_ErrorMsg_accd(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_wrench accd(::controller::msg::ErrorMsg::_accd_type arg)
  {
    msg_.accd = std::move(arg);
    return Init_ErrorMsg_wrench(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_ier
{
public:
  explicit Init_ErrorMsg_ier(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_accd ier(::controller::msg::ErrorMsg::_ier_type arg)
  {
    msg_.ier = std::move(arg);
    return Init_ErrorMsg_accd(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_iex
{
public:
  explicit Init_ErrorMsg_iex(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_ier iex(::controller::msg::ErrorMsg::_iex_type arg)
  {
    msg_.iex = std::move(arg);
    return Init_ErrorMsg_ier(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_ew
{
public:
  explicit Init_ErrorMsg_ew(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_iex ew(::controller::msg::ErrorMsg::_ew_type arg)
  {
    msg_.ew = std::move(arg);
    return Init_ErrorMsg_iex(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_er
{
public:
  explicit Init_ErrorMsg_er(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_ew er(::controller::msg::ErrorMsg::_er_type arg)
  {
    msg_.er = std::move(arg);
    return Init_ErrorMsg_ew(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_ea
{
public:
  explicit Init_ErrorMsg_ea(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_er ea(::controller::msg::ErrorMsg::_ea_type arg)
  {
    msg_.ea = std::move(arg);
    return Init_ErrorMsg_er(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_ev
{
public:
  explicit Init_ErrorMsg_ev(::controller::msg::ErrorMsg & msg)
  : msg_(msg)
  {}
  Init_ErrorMsg_ea ev(::controller::msg::ErrorMsg::_ev_type arg)
  {
    msg_.ev = std::move(arg);
    return Init_ErrorMsg_ea(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

class Init_ErrorMsg_ex
{
public:
  Init_ErrorMsg_ex()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ErrorMsg_ev ex(::controller::msg::ErrorMsg::_ex_type arg)
  {
    msg_.ex = std::move(arg);
    return Init_ErrorMsg_ev(msg_);
  }

private:
  ::controller::msg::ErrorMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::ErrorMsg>()
{
  return controller::msg::builder::Init_ErrorMsg_ex();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MSG__BUILDER_HPP_
