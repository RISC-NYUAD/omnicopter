// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from maneuver:srv/Ellipse5D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/ellipse5_d.hpp"


#ifndef MANEUVER__SRV__DETAIL__ELLIPSE5_D__TRAITS_HPP_
#define MANEUVER__SRV__DETAIL__ELLIPSE5_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "maneuver/srv/detail/ellipse5_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Ellipse5D_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_min
  {
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << ", ";
  }

  // member: x_max
  {
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << ", ";
  }

  // member: y_min
  {
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << ", ";
  }

  // member: y_max
  {
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
    out << ", ";
  }

  // member: z_min
  {
    out << "z_min: ";
    rosidl_generator_traits::value_to_yaml(msg.z_min, out);
    out << ", ";
  }

  // member: z_max
  {
    out << "z_max: ";
    rosidl_generator_traits::value_to_yaml(msg.z_max, out);
    out << ", ";
  }

  // member: roll_start
  {
    out << "roll_start: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_start, out);
    out << ", ";
  }

  // member: roll_mid
  {
    out << "roll_mid: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_mid, out);
    out << ", ";
  }

  // member: roll_end
  {
    out << "roll_end: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_end, out);
    out << ", ";
  }

  // member: pitch_start
  {
    out << "pitch_start: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_start, out);
    out << ", ";
  }

  // member: pitch_mid
  {
    out << "pitch_mid: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_mid, out);
    out << ", ";
  }

  // member: pitch_end
  {
    out << "pitch_end: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_end, out);
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
  const Ellipse5D_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << "\n";
  }

  // member: x_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << "\n";
  }

  // member: y_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << "\n";
  }

  // member: y_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
    out << "\n";
  }

  // member: z_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_min: ";
    rosidl_generator_traits::value_to_yaml(msg.z_min, out);
    out << "\n";
  }

  // member: z_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_max: ";
    rosidl_generator_traits::value_to_yaml(msg.z_max, out);
    out << "\n";
  }

  // member: roll_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_start: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_start, out);
    out << "\n";
  }

  // member: roll_mid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_mid: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_mid, out);
    out << "\n";
  }

  // member: roll_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_end: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_end, out);
    out << "\n";
  }

  // member: pitch_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_start: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_start, out);
    out << "\n";
  }

  // member: pitch_mid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_mid: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_mid, out);
    out << "\n";
  }

  // member: pitch_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_end: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_end, out);
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

inline std::string to_yaml(const Ellipse5D_Request & msg, bool use_flow_style = false)
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
  const maneuver::srv::Ellipse5D_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Ellipse5D_Request & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Ellipse5D_Request>()
{
  return "maneuver::srv::Ellipse5D_Request";
}

template<>
inline const char * name<maneuver::srv::Ellipse5D_Request>()
{
  return "maneuver/srv/Ellipse5D_Request";
}

template<>
struct has_fixed_size<maneuver::srv::Ellipse5D_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Ellipse5D_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Ellipse5D_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const Ellipse5D_Response & msg,
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
  const Ellipse5D_Response & msg,
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

inline std::string to_yaml(const Ellipse5D_Response & msg, bool use_flow_style = false)
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
  const maneuver::srv::Ellipse5D_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Ellipse5D_Response & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Ellipse5D_Response>()
{
  return "maneuver::srv::Ellipse5D_Response";
}

template<>
inline const char * name<maneuver::srv::Ellipse5D_Response>()
{
  return "maneuver/srv/Ellipse5D_Response";
}

template<>
struct has_fixed_size<maneuver::srv::Ellipse5D_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::Ellipse5D_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::Ellipse5D_Response>
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
  const Ellipse5D_Event & msg,
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
  const Ellipse5D_Event & msg,
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

inline std::string to_yaml(const Ellipse5D_Event & msg, bool use_flow_style = false)
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
  const maneuver::srv::Ellipse5D_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::Ellipse5D_Event & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::Ellipse5D_Event>()
{
  return "maneuver::srv::Ellipse5D_Event";
}

template<>
inline const char * name<maneuver::srv::Ellipse5D_Event>()
{
  return "maneuver/srv/Ellipse5D_Event";
}

template<>
struct has_fixed_size<maneuver::srv::Ellipse5D_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<maneuver::srv::Ellipse5D_Event>
  : std::integral_constant<bool, has_bounded_size<maneuver::srv::Ellipse5D_Request>::value && has_bounded_size<maneuver::srv::Ellipse5D_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<maneuver::srv::Ellipse5D_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<maneuver::srv::Ellipse5D>()
{
  return "maneuver::srv::Ellipse5D";
}

template<>
inline const char * name<maneuver::srv::Ellipse5D>()
{
  return "maneuver/srv/Ellipse5D";
}

template<>
struct has_fixed_size<maneuver::srv::Ellipse5D>
  : std::integral_constant<
    bool,
    has_fixed_size<maneuver::srv::Ellipse5D_Request>::value &&
    has_fixed_size<maneuver::srv::Ellipse5D_Response>::value
  >
{
};

template<>
struct has_bounded_size<maneuver::srv::Ellipse5D>
  : std::integral_constant<
    bool,
    has_bounded_size<maneuver::srv::Ellipse5D_Request>::value &&
    has_bounded_size<maneuver::srv::Ellipse5D_Response>::value
  >
{
};

template<>
struct is_service<maneuver::srv::Ellipse5D>
  : std::true_type
{
};

template<>
struct is_service_request<maneuver::srv::Ellipse5D_Request>
  : std::true_type
{
};

template<>
struct is_service_response<maneuver::srv::Ellipse5D_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MANEUVER__SRV__DETAIL__ELLIPSE5_D__TRAITS_HPP_
