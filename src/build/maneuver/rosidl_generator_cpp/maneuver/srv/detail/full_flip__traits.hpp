// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from maneuver:srv/FullFlip.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/full_flip.hpp"


#ifndef MANEUVER__SRV__DETAIL__FULL_FLIP__TRAITS_HPP_
#define MANEUVER__SRV__DETAIL__FULL_FLIP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "maneuver/srv/detail/full_flip__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const FullFlip_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: roll_bool
  {
    out << "roll_bool: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_bool, out);
    out << ", ";
  }

  // member: pitch_bool
  {
    out << "pitch_bool: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_bool, out);
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
  const FullFlip_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: roll_bool
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_bool: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_bool, out);
    out << "\n";
  }

  // member: pitch_bool
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_bool: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_bool, out);
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

inline std::string to_yaml(const FullFlip_Request & msg, bool use_flow_style = false)
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
  const maneuver::srv::FullFlip_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::FullFlip_Request & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::FullFlip_Request>()
{
  return "maneuver::srv::FullFlip_Request";
}

template<>
inline const char * name<maneuver::srv::FullFlip_Request>()
{
  return "maneuver/srv/FullFlip_Request";
}

template<>
struct has_fixed_size<maneuver::srv::FullFlip_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::FullFlip_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::FullFlip_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace maneuver
{

namespace srv
{

inline void to_flow_style_yaml(
  const FullFlip_Response & msg,
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
  const FullFlip_Response & msg,
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

inline std::string to_yaml(const FullFlip_Response & msg, bool use_flow_style = false)
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
  const maneuver::srv::FullFlip_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::FullFlip_Response & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::FullFlip_Response>()
{
  return "maneuver::srv::FullFlip_Response";
}

template<>
inline const char * name<maneuver::srv::FullFlip_Response>()
{
  return "maneuver/srv/FullFlip_Response";
}

template<>
struct has_fixed_size<maneuver::srv::FullFlip_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<maneuver::srv::FullFlip_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<maneuver::srv::FullFlip_Response>
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
  const FullFlip_Event & msg,
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
  const FullFlip_Event & msg,
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

inline std::string to_yaml(const FullFlip_Event & msg, bool use_flow_style = false)
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
  const maneuver::srv::FullFlip_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::srv::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::srv::FullFlip_Event & msg)
{
  return maneuver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::srv::FullFlip_Event>()
{
  return "maneuver::srv::FullFlip_Event";
}

template<>
inline const char * name<maneuver::srv::FullFlip_Event>()
{
  return "maneuver/srv/FullFlip_Event";
}

template<>
struct has_fixed_size<maneuver::srv::FullFlip_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<maneuver::srv::FullFlip_Event>
  : std::integral_constant<bool, has_bounded_size<maneuver::srv::FullFlip_Request>::value && has_bounded_size<maneuver::srv::FullFlip_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<maneuver::srv::FullFlip_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<maneuver::srv::FullFlip>()
{
  return "maneuver::srv::FullFlip";
}

template<>
inline const char * name<maneuver::srv::FullFlip>()
{
  return "maneuver/srv/FullFlip";
}

template<>
struct has_fixed_size<maneuver::srv::FullFlip>
  : std::integral_constant<
    bool,
    has_fixed_size<maneuver::srv::FullFlip_Request>::value &&
    has_fixed_size<maneuver::srv::FullFlip_Response>::value
  >
{
};

template<>
struct has_bounded_size<maneuver::srv::FullFlip>
  : std::integral_constant<
    bool,
    has_bounded_size<maneuver::srv::FullFlip_Request>::value &&
    has_bounded_size<maneuver::srv::FullFlip_Response>::value
  >
{
};

template<>
struct is_service<maneuver::srv::FullFlip>
  : std::true_type
{
};

template<>
struct is_service_request<maneuver::srv::FullFlip_Request>
  : std::true_type
{
};

template<>
struct is_service_response<maneuver::srv::FullFlip_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MANEUVER__SRV__DETAIL__FULL_FLIP__TRAITS_HPP_
