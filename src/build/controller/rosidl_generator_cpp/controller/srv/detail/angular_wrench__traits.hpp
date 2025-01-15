// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/AngularWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/angular_wrench.hpp"


#ifndef CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/angular_wrench__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const AngularWrench_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: fz
  {
    out << "fz: ";
    rosidl_generator_traits::value_to_yaml(msg.fz, out);
    out << ", ";
  }

  // member: phi1
  {
    out << "phi1: ";
    rosidl_generator_traits::value_to_yaml(msg.phi1, out);
    out << ", ";
  }

  // member: phi2
  {
    out << "phi2: ";
    rosidl_generator_traits::value_to_yaml(msg.phi2, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AngularWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fz: ";
    rosidl_generator_traits::value_to_yaml(msg.fz, out);
    out << "\n";
  }

  // member: phi1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phi1: ";
    rosidl_generator_traits::value_to_yaml(msg.phi1, out);
    out << "\n";
  }

  // member: phi2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phi2: ";
    rosidl_generator_traits::value_to_yaml(msg.phi2, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AngularWrench_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::srv::AngularWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::AngularWrench_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::AngularWrench_Request>()
{
  return "controller::srv::AngularWrench_Request";
}

template<>
inline const char * name<controller::srv::AngularWrench_Request>()
{
  return "controller/srv/AngularWrench_Request";
}

template<>
struct has_fixed_size<controller::srv::AngularWrench_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::AngularWrench_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::AngularWrench_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const AngularWrench_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AngularWrench_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AngularWrench_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::srv::AngularWrench_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::AngularWrench_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::AngularWrench_Response>()
{
  return "controller::srv::AngularWrench_Response";
}

template<>
inline const char * name<controller::srv::AngularWrench_Response>()
{
  return "controller/srv/AngularWrench_Response";
}

template<>
struct has_fixed_size<controller::srv::AngularWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::AngularWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::AngularWrench_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const AngularWrench_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AngularWrench_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AngularWrench_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::srv::AngularWrench_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::AngularWrench_Event & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::AngularWrench_Event>()
{
  return "controller::srv::AngularWrench_Event";
}

template<>
inline const char * name<controller::srv::AngularWrench_Event>()
{
  return "controller/srv/AngularWrench_Event";
}

template<>
struct has_fixed_size<controller::srv::AngularWrench_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::AngularWrench_Event>
  : std::integral_constant<bool, has_bounded_size<controller::srv::AngularWrench_Request>::value && has_bounded_size<controller::srv::AngularWrench_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<controller::srv::AngularWrench_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::AngularWrench>()
{
  return "controller::srv::AngularWrench";
}

template<>
inline const char * name<controller::srv::AngularWrench>()
{
  return "controller/srv/AngularWrench";
}

template<>
struct has_fixed_size<controller::srv::AngularWrench>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::AngularWrench_Request>::value &&
    has_fixed_size<controller::srv::AngularWrench_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::AngularWrench>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::AngularWrench_Request>::value &&
    has_bounded_size<controller::srv::AngularWrench_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::AngularWrench>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::AngularWrench_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::AngularWrench_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__TRAITS_HPP_
