// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/error_msg.hpp"


#ifndef CONTROLLER__MSG__DETAIL__ERROR_MSG__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__ERROR_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/error_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'ex'
// Member 'ev'
// Member 'ea'
// Member 'er'
// Member 'ew'
// Member 'iex'
// Member 'ier'
// Member 'accd'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const ErrorMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: ex
  {
    out << "ex: ";
    to_flow_style_yaml(msg.ex, out);
    out << ", ";
  }

  // member: ev
  {
    out << "ev: ";
    to_flow_style_yaml(msg.ev, out);
    out << ", ";
  }

  // member: ea
  {
    out << "ea: ";
    to_flow_style_yaml(msg.ea, out);
    out << ", ";
  }

  // member: er
  {
    out << "er: ";
    to_flow_style_yaml(msg.er, out);
    out << ", ";
  }

  // member: ew
  {
    out << "ew: ";
    to_flow_style_yaml(msg.ew, out);
    out << ", ";
  }

  // member: iex
  {
    out << "iex: ";
    to_flow_style_yaml(msg.iex, out);
    out << ", ";
  }

  // member: ier
  {
    out << "ier: ";
    to_flow_style_yaml(msg.ier, out);
    out << ", ";
  }

  // member: accd
  {
    out << "accd: ";
    to_flow_style_yaml(msg.accd, out);
    out << ", ";
  }

  // member: wrench
  {
    if (msg.wrench.size() == 0) {
      out << "wrench: []";
    } else {
      out << "wrench: [";
      size_t pending_items = msg.wrench.size();
      for (auto item : msg.wrench) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: exwrench
  {
    if (msg.exwrench.size() == 0) {
      out << "exwrench: []";
    } else {
      out << "exwrench: [";
      size_t pending_items = msg.exwrench.size();
      for (auto item : msg.exwrench) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: prop
  {
    if (msg.prop.size() == 0) {
      out << "prop: []";
    } else {
      out << "prop: [";
      size_t pending_items = msg.prop.size();
      for (auto item : msg.prop) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: weight
  {
    out << "weight: ";
    rosidl_generator_traits::value_to_yaml(msg.weight, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ErrorMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ex
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ex:\n";
    to_block_style_yaml(msg.ex, out, indentation + 2);
  }

  // member: ev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ev:\n";
    to_block_style_yaml(msg.ev, out, indentation + 2);
  }

  // member: ea
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ea:\n";
    to_block_style_yaml(msg.ea, out, indentation + 2);
  }

  // member: er
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "er:\n";
    to_block_style_yaml(msg.er, out, indentation + 2);
  }

  // member: ew
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ew:\n";
    to_block_style_yaml(msg.ew, out, indentation + 2);
  }

  // member: iex
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iex:\n";
    to_block_style_yaml(msg.iex, out, indentation + 2);
  }

  // member: ier
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ier:\n";
    to_block_style_yaml(msg.ier, out, indentation + 2);
  }

  // member: accd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accd:\n";
    to_block_style_yaml(msg.accd, out, indentation + 2);
  }

  // member: wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wrench.size() == 0) {
      out << "wrench: []\n";
    } else {
      out << "wrench:\n";
      for (auto item : msg.wrench) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: exwrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.exwrench.size() == 0) {
      out << "exwrench: []\n";
    } else {
      out << "exwrench:\n";
      for (auto item : msg.exwrench) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: prop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prop.size() == 0) {
      out << "prop: []\n";
    } else {
      out << "prop:\n";
      for (auto item : msg.prop) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: weight
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weight: ";
    rosidl_generator_traits::value_to_yaml(msg.weight, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ErrorMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::msg::ErrorMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::ErrorMsg & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::ErrorMsg>()
{
  return "controller::msg::ErrorMsg";
}

template<>
inline const char * name<controller::msg::ErrorMsg>()
{
  return "controller/msg/ErrorMsg";
}

template<>
struct has_fixed_size<controller::msg::ErrorMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::msg::ErrorMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::msg::ErrorMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MSG__TRAITS_HPP_
