// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/linear_wrench.hpp"


#ifndef CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/linear_wrench__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const LinearWrench_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: fx1
  {
    out << "fx1: ";
    rosidl_generator_traits::value_to_yaml(msg.fx1, out);
    out << ", ";
  }

  // member: fy1
  {
    out << "fy1: ";
    rosidl_generator_traits::value_to_yaml(msg.fy1, out);
    out << ", ";
  }

  // member: fz1
  {
    out << "fz1: ";
    rosidl_generator_traits::value_to_yaml(msg.fz1, out);
    out << ", ";
  }

  // member: fx2
  {
    out << "fx2: ";
    rosidl_generator_traits::value_to_yaml(msg.fx2, out);
    out << ", ";
  }

  // member: fy2
  {
    out << "fy2: ";
    rosidl_generator_traits::value_to_yaml(msg.fy2, out);
    out << ", ";
  }

  // member: fz2
  {
    out << "fz2: ";
    rosidl_generator_traits::value_to_yaml(msg.fz2, out);
    out << ", ";
  }

  // member: ramp
  {
    out << "ramp: ";
    rosidl_generator_traits::value_to_yaml(msg.ramp, out);
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
  const LinearWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fx1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx1: ";
    rosidl_generator_traits::value_to_yaml(msg.fx1, out);
    out << "\n";
  }

  // member: fy1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy1: ";
    rosidl_generator_traits::value_to_yaml(msg.fy1, out);
    out << "\n";
  }

  // member: fz1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fz1: ";
    rosidl_generator_traits::value_to_yaml(msg.fz1, out);
    out << "\n";
  }

  // member: fx2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx2: ";
    rosidl_generator_traits::value_to_yaml(msg.fx2, out);
    out << "\n";
  }

  // member: fy2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy2: ";
    rosidl_generator_traits::value_to_yaml(msg.fy2, out);
    out << "\n";
  }

  // member: fz2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fz2: ";
    rosidl_generator_traits::value_to_yaml(msg.fz2, out);
    out << "\n";
  }

  // member: ramp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ramp: ";
    rosidl_generator_traits::value_to_yaml(msg.ramp, out);
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

inline std::string to_yaml(const LinearWrench_Request & msg, bool use_flow_style = false)
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
  const controller::srv::LinearWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::LinearWrench_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::LinearWrench_Request>()
{
  return "controller::srv::LinearWrench_Request";
}

template<>
inline const char * name<controller::srv::LinearWrench_Request>()
{
  return "controller/srv/LinearWrench_Request";
}

template<>
struct has_fixed_size<controller::srv::LinearWrench_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::LinearWrench_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::LinearWrench_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const LinearWrench_Response & msg,
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
  const LinearWrench_Response & msg,
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

inline std::string to_yaml(const LinearWrench_Response & msg, bool use_flow_style = false)
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
  const controller::srv::LinearWrench_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::LinearWrench_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::LinearWrench_Response>()
{
  return "controller::srv::LinearWrench_Response";
}

template<>
inline const char * name<controller::srv::LinearWrench_Response>()
{
  return "controller/srv/LinearWrench_Response";
}

template<>
struct has_fixed_size<controller::srv::LinearWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::LinearWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::LinearWrench_Response>
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
  const LinearWrench_Event & msg,
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
  const LinearWrench_Event & msg,
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

inline std::string to_yaml(const LinearWrench_Event & msg, bool use_flow_style = false)
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
  const controller::srv::LinearWrench_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::LinearWrench_Event & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::LinearWrench_Event>()
{
  return "controller::srv::LinearWrench_Event";
}

template<>
inline const char * name<controller::srv::LinearWrench_Event>()
{
  return "controller/srv/LinearWrench_Event";
}

template<>
struct has_fixed_size<controller::srv::LinearWrench_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::LinearWrench_Event>
  : std::integral_constant<bool, has_bounded_size<controller::srv::LinearWrench_Request>::value && has_bounded_size<controller::srv::LinearWrench_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<controller::srv::LinearWrench_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::LinearWrench>()
{
  return "controller::srv::LinearWrench";
}

template<>
inline const char * name<controller::srv::LinearWrench>()
{
  return "controller/srv/LinearWrench";
}

template<>
struct has_fixed_size<controller::srv::LinearWrench>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::LinearWrench_Request>::value &&
    has_fixed_size<controller::srv::LinearWrench_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::LinearWrench>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::LinearWrench_Request>::value &&
    has_bounded_size<controller::srv::LinearWrench_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::LinearWrench>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::LinearWrench_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::LinearWrench_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__TRAITS_HPP_
