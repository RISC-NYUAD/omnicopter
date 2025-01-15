// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from maneuver:srv/Goto6DPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/goto6_d_point.hpp"


#ifndef MANEUVER__SRV__DETAIL__GOTO6_D_POINT__TRAITS_HPP_
#define MANEUVER__SRV__DETAIL__GOTO6_D_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "maneuver/srv/detail/goto6_d_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Goto6DPoint_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
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
  const Goto6DPoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
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

inline std::string to_yaml(const Goto6DPoint_Request & msg, bool use_flow_style = false)
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
  const maneuver::srv::Goto6DPoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Goto6DPoint_Request & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Goto6DPoint_Request>()
{
  return "maneuver::srv::Goto6DPoint_Request";
}

template<>
inline const char * name<maneuver::srv::Goto6DPoint_Request>()
{
  return "maneuver/srv/Goto6DPoint_Request";
}

template<>
struct has_fixed_size<maneuver::srv::Goto6DPoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Goto6DPoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Goto6DPoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Goto6DPoint_Response & msg,
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
  const Goto6DPoint_Response & msg,
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

inline std::string to_yaml(const Goto6DPoint_Response & msg, bool use_flow_style = false)
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
  const maneuver::srv::Goto6DPoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Goto6DPoint_Response & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Goto6DPoint_Response>()
{
  return "maneuver::srv::Goto6DPoint_Response";
}

template<>
inline const char * name<maneuver::srv::Goto6DPoint_Response>()
{
  return "maneuver/srv/Goto6DPoint_Response";
}

template<>
struct has_fixed_size<maneuver::srv::Goto6DPoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Goto6DPoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Goto6DPoint_Response>
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
  const Goto6DPoint_Event & msg,
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
  const Goto6DPoint_Event & msg,
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

inline std::string to_yaml(const Goto6DPoint_Event & msg, bool use_flow_style = false)
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
  const maneuver::srv::Goto6DPoint_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Goto6DPoint_Event & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Goto6DPoint_Event>()
{
  return "maneuver::srv::Goto6DPoint_Event";
}

template<>
inline const char * name<maneuver::srv::Goto6DPoint_Event>()
{
  return "maneuver/srv/Goto6DPoint_Event";
}

template<>
struct has_fixed_size<maneuver::srv::Goto6DPoint_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<maneuver::srv::Goto6DPoint_Event>
  : std::integral_constant<bool, has_bounded_size<maneuver::srv::Goto6DPoint_Request>::value && has_bounded_size<maneuver::srv::Goto6DPoint_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<maneuver::srv::Goto6DPoint_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<maneuver::srv::Goto6DPoint>()
{
  return "maneuver::srv::Goto6DPoint";
}

template<>
inline const char * name<maneuver::srv::Goto6DPoint>()
{
  return "maneuver/srv/Goto6DPoint";
}

template<>
struct has_fixed_size<maneuver::srv::Goto6DPoint>
  : std::integral_constant<
    bool,
    has_fixed_size<maneuver::srv::Goto6DPoint_Request>::value &&
    has_fixed_size<maneuver::srv::Goto6DPoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<maneuver::srv::Goto6DPoint>
  : std::integral_constant<
    bool,
    has_bounded_size<maneuver::srv::Goto6DPoint_Request>::value &&
    has_bounded_size<maneuver::srv::Goto6DPoint_Response>::value
  >
{
};

template<>
struct is_service<maneuver::srv::Goto6DPoint>
  : std::true_type
{
};

template<>
struct is_service_request<maneuver::srv::Goto6DPoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<maneuver::srv::Goto6DPoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MANEUVER__SRV__DETAIL__GOTO6_D_POINT__TRAITS_HPP_
