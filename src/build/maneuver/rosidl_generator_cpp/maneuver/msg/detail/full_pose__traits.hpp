// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/msg/full_pose.hpp"


#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__TRAITS_HPP_
#define MANEUVER__MSG__DETAIL__FULL_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "maneuver/msg/detail/full_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__traits.hpp"

namespace maneuver
{

namespace msg
{

inline void to_flow_style_yaml(
  const FullPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: vel
  {
    out << "vel: ";
    to_flow_style_yaml(msg.vel, out);
    out << ", ";
  }

  // member: acc
  {
    out << "acc: ";
    to_flow_style_yaml(msg.acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FullPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel:\n";
    to_block_style_yaml(msg.vel, out, indentation + 2);
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc:\n";
    to_block_style_yaml(msg.acc, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FullPose & msg, bool use_flow_style = false)
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

}  // namespace maneuver

namespace rosidl_generator_traits
{

[[deprecated("use maneuver::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const maneuver::msg::FullPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  maneuver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use maneuver::msg::to_yaml() instead")]]
inline std::string to_yaml(const maneuver::msg::FullPose & msg)
{
  return maneuver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<maneuver::msg::FullPose>()
{
  return "maneuver::msg::FullPose";
}

template<>
inline const char * name<maneuver::msg::FullPose>()
{
  return "maneuver/msg/FullPose";
}

template<>
struct has_fixed_size<maneuver::msg::FullPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Accel>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<maneuver::msg::FullPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Accel>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<maneuver::msg::FullPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__TRAITS_HPP_
