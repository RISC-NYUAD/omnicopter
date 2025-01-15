// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from maneuver:srv/Ellipse5D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/ellipse5_d.hpp"


#ifndef MANEUVER__SRV__DETAIL__ELLIPSE5_D__BUILDER_HPP_
#define MANEUVER__SRV__DETAIL__ELLIPSE5_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "maneuver/srv/detail/ellipse5_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Ellipse5D_Request_duration
{
public:
  explicit Init_Ellipse5D_Request_duration(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Ellipse5D_Request duration(::maneuver::srv::Ellipse5D_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_yaw
{
public:
  explicit Init_Ellipse5D_Request_yaw(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_duration yaw(::maneuver::srv::Ellipse5D_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Ellipse5D_Request_duration(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_pitch_end
{
public:
  explicit Init_Ellipse5D_Request_pitch_end(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_yaw pitch_end(::maneuver::srv::Ellipse5D_Request::_pitch_end_type arg)
  {
    msg_.pitch_end = std::move(arg);
    return Init_Ellipse5D_Request_yaw(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_pitch_mid
{
public:
  explicit Init_Ellipse5D_Request_pitch_mid(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_pitch_end pitch_mid(::maneuver::srv::Ellipse5D_Request::_pitch_mid_type arg)
  {
    msg_.pitch_mid = std::move(arg);
    return Init_Ellipse5D_Request_pitch_end(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_pitch_start
{
public:
  explicit Init_Ellipse5D_Request_pitch_start(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_pitch_mid pitch_start(::maneuver::srv::Ellipse5D_Request::_pitch_start_type arg)
  {
    msg_.pitch_start = std::move(arg);
    return Init_Ellipse5D_Request_pitch_mid(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_roll_end
{
public:
  explicit Init_Ellipse5D_Request_roll_end(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_pitch_start roll_end(::maneuver::srv::Ellipse5D_Request::_roll_end_type arg)
  {
    msg_.roll_end = std::move(arg);
    return Init_Ellipse5D_Request_pitch_start(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_roll_mid
{
public:
  explicit Init_Ellipse5D_Request_roll_mid(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_roll_end roll_mid(::maneuver::srv::Ellipse5D_Request::_roll_mid_type arg)
  {
    msg_.roll_mid = std::move(arg);
    return Init_Ellipse5D_Request_roll_end(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_roll_start
{
public:
  explicit Init_Ellipse5D_Request_roll_start(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_roll_mid roll_start(::maneuver::srv::Ellipse5D_Request::_roll_start_type arg)
  {
    msg_.roll_start = std::move(arg);
    return Init_Ellipse5D_Request_roll_mid(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_z_max
{
public:
  explicit Init_Ellipse5D_Request_z_max(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_roll_start z_max(::maneuver::srv::Ellipse5D_Request::_z_max_type arg)
  {
    msg_.z_max = std::move(arg);
    return Init_Ellipse5D_Request_roll_start(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_z_min
{
public:
  explicit Init_Ellipse5D_Request_z_min(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_z_max z_min(::maneuver::srv::Ellipse5D_Request::_z_min_type arg)
  {
    msg_.z_min = std::move(arg);
    return Init_Ellipse5D_Request_z_max(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_y_max
{
public:
  explicit Init_Ellipse5D_Request_y_max(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_z_min y_max(::maneuver::srv::Ellipse5D_Request::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return Init_Ellipse5D_Request_z_min(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_y_min
{
public:
  explicit Init_Ellipse5D_Request_y_min(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_y_max y_min(::maneuver::srv::Ellipse5D_Request::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_Ellipse5D_Request_y_max(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_x_max
{
public:
  explicit Init_Ellipse5D_Request_x_max(::maneuver::srv::Ellipse5D_Request & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Request_y_min x_max(::maneuver::srv::Ellipse5D_Request::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_Ellipse5D_Request_y_min(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

class Init_Ellipse5D_Request_x_min
{
public:
  Init_Ellipse5D_Request_x_min()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ellipse5D_Request_x_max x_min(::maneuver::srv::Ellipse5D_Request::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_Ellipse5D_Request_x_max(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Ellipse5D_Request>()
{
  return maneuver::srv::builder::Init_Ellipse5D_Request_x_min();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Ellipse5D_Response_status
{
public:
  Init_Ellipse5D_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::maneuver::srv::Ellipse5D_Response status(::maneuver::srv::Ellipse5D_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Ellipse5D_Response>()
{
  return maneuver::srv::builder::Init_Ellipse5D_Response_status();
}

}  // namespace maneuver


namespace maneuver
{

namespace srv
{

namespace builder
{

class Init_Ellipse5D_Event_response
{
public:
  explicit Init_Ellipse5D_Event_response(::maneuver::srv::Ellipse5D_Event & msg)
  : msg_(msg)
  {}
  ::maneuver::srv::Ellipse5D_Event response(::maneuver::srv::Ellipse5D_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Event msg_;
};

class Init_Ellipse5D_Event_request
{
public:
  explicit Init_Ellipse5D_Event_request(::maneuver::srv::Ellipse5D_Event & msg)
  : msg_(msg)
  {}
  Init_Ellipse5D_Event_response request(::maneuver::srv::Ellipse5D_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Ellipse5D_Event_response(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Event msg_;
};

class Init_Ellipse5D_Event_info
{
public:
  Init_Ellipse5D_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ellipse5D_Event_request info(::maneuver::srv::Ellipse5D_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Ellipse5D_Event_request(msg_);
  }

private:
  ::maneuver::srv::Ellipse5D_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::maneuver::srv::Ellipse5D_Event>()
{
  return maneuver::srv::builder::Init_Ellipse5D_Event_info();
}

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__ELLIPSE5_D__BUILDER_HPP_
