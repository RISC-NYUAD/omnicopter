// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/motor_speed.hpp"


#ifndef CONTROLLER__MSG__DETAIL__MOTOR_SPEED__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__MOTOR_SPEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/motor_speed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSpeed & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    if (msg.name.size() == 0) {
      out << "name: []";
    } else {
      out << "name: [";
      size_t pending_items = msg.name.size();
      for (auto item : msg.name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity
  {
    if (msg.velocity.size() == 0) {
      out << "velocity: []";
    } else {
      out << "velocity: [";
      size_t pending_items = msg.velocity.size();
      for (auto item : msg.velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const MotorSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.name.size() == 0) {
      out << "name: []\n";
    } else {
      out << "name:\n";
      for (auto item : msg.name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity.size() == 0) {
      out << "velocity: []\n";
    } else {
      out << "velocity:\n";
      for (auto item : msg.velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSpeed & msg, bool use_flow_style = false)
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
  const controller::msg::MotorSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::MotorSpeed & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::MotorSpeed>()
{
  return "controller::msg::MotorSpeed";
}

template<>
inline const char * name<controller::msg::MotorSpeed>()
{
  return "controller/msg/MotorSpeed";
}

template<>
struct has_fixed_size<controller::msg::MotorSpeed>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::msg::MotorSpeed>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::msg::MotorSpeed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__MOTOR_SPEED__TRAITS_HPP_
