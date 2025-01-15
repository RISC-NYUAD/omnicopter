// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from maneuver:srv/Land.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/land.hpp"


#ifndef MANEUVER__SRV__DETAIL__LAND__TRAITS_HPP_
#define MANEUVER__SRV__DETAIL__LAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "maneuver/srv/detail/land__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Land_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: height_1
  {
    out << "height_1: ";
    rosidl_generator_traits::value_to_yaml(msg.height_1, out);
    out << ", ";
  }

  // member: duration_1
  {
    out << "duration_1: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_1, out);
    out << ", ";
  }

  // member: height_2
  {
    out << "height_2: ";
    rosidl_generator_traits::value_to_yaml(msg.height_2, out);
    out << ", ";
  }

  // member: duration_2
  {
    out << "duration_2: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Land_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: height_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_1: ";
    rosidl_generator_traits::value_to_yaml(msg.height_1, out);
    out << "\n";
  }

  // member: duration_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration_1: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_1, out);
    out << "\n";
  }

  // member: height_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_2: ";
    rosidl_generator_traits::value_to_yaml(msg.height_2, out);
    out << "\n";
  }

  // member: duration_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration_2: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Land_Request & msg, bool use_flow_style = false)
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

}  // namespace maneuver

namespace rosidl_generator_traits
{

[[deprecated("use maneuver::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const maneuver::srv::Land_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Land_Request & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Land_Request>()
{
  return "maneuver::srv::Land_Request";
}

template<>
inline const char * name<maneuver::srv::Land_Request>()
{
  return "maneuver/srv/Land_Request";
}

template<>
struct has_fixed_size<maneuver::srv::Land_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Land_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Land_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Land_Response & msg,
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
  const Land_Response & msg,
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

inline std::string to_yaml(const Land_Response & msg, bool use_flow_style = false)
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

}  // namespace maneuver

namespace rosidl_generator_traits
{

[[deprecated("use maneuver::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const maneuver::srv::Land_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Land_Response & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Land_Response>()
{
  return "maneuver::srv::Land_Response";
}

template<>
inline const char * name<maneuver::srv::Land_Response>()
{
  return "maneuver/srv/Land_Response";
}

template<>
struct has_fixed_size<maneuver::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Land_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Land_Event & msg,
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
  const Land_Event & msg,
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

inline std::string to_yaml(const Land_Event & msg, bool use_flow_style = false)
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

}  // namespace maneuver

namespace rosidl_generator_traits
{

[[deprecated("use maneuver::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const maneuver::srv::Land_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Land_Event & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Land_Event>()
{
  return "maneuver::srv::Land_Event";
}

template<>
inline const char * name<maneuver::srv::Land_Event>()
{
  return "maneuver/srv/Land_Event";
}

template<>
struct has_fixed_size<maneuver::srv::Land_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<maneuver::srv::Land_Event>
  : std::integral_constant<bool, has_bounded_size<maneuver::srv::Land_Request>::value && has_bounded_size<maneuver::srv::Land_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<maneuver::srv::Land_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<maneuver::srv::Land>()
{
  return "maneuver::srv::Land";
}

template<>
inline const char * name<maneuver::srv::Land>()
{
  return "maneuver/srv/Land";
}

template<>
struct has_fixed_size<maneuver::srv::Land>
  : std::integral_constant<
    bool,
    has_fixed_size<maneuver::srv::Land_Request>::value &&
    has_fixed_size<maneuver::srv::Land_Response>::value
  >
{
};

template<>
struct has_bounded_size<maneuver::srv::Land>
  : std::integral_constant<
    bool,
    has_bounded_size<maneuver::srv::Land_Request>::value &&
    has_bounded_size<maneuver::srv::Land_Response>::value
  >
{
};

template<>
struct is_service<maneuver::srv::Land>
  : std::true_type
{
};

template<>
struct is_service_request<maneuver::srv::Land_Request>
  : std::true_type
{
};

template<>
struct is_service_response<maneuver::srv::Land_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MANEUVER__SRV__DETAIL__LAND__TRAITS_HPP_
