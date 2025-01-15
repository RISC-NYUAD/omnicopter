// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/uvector.hpp"


#ifndef CONTROLLER__MSG__DETAIL__UVECTOR__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__UVECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/uvector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const Uvector & msg,
  std::ostream & out)
{
  out << "{";
  // member: value
  {
    if (msg.value.size() == 0) {
      out << "value: []";
    } else {
      out << "value: [";
      size_t pending_items = msg.value.size();
      for (auto item : msg.value) {
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
  const Uvector & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.value.size() == 0) {
      out << "value: []\n";
    } else {
      out << "value:\n";
      for (auto item : msg.value) {
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

inline std::string to_yaml(const Uvector & msg, bool use_flow_style = false)
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
  const controller::msg::Uvector & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::Uvector & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::Uvector>()
{
  return "controller::msg::Uvector";
}

template<>
inline const char * name<controller::msg::Uvector>()
{
  return "controller/msg/Uvector";
}

template<>
struct has_fixed_size<controller::msg::Uvector>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::Uvector>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::Uvector>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__UVECTOR__TRAITS_HPP_
