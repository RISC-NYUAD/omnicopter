// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/msg/full_pose.hpp"


#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_HPP_
#define MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__maneuver__msg__FullPose __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__msg__FullPose __declspec(deprecated)
#endif

namespace maneuver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FullPose_
{
  using Type = FullPose_<ContainerAllocator>;

  explicit FullPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    vel(_init),
    acc(_init)
  {
    (void)_init;
  }

  explicit FullPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    vel(_alloc, _init),
    acc(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _vel_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _vel_type vel;
  using _acc_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _acc_type acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__vel(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__acc(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    maneuver::msg::FullPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::msg::FullPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::msg::FullPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::msg::FullPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::msg::FullPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::msg::FullPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::msg::FullPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::msg::FullPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::msg::FullPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::msg::FullPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__msg__FullPose
    std::shared_ptr<maneuver::msg::FullPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__msg__FullPose
    std::shared_ptr<maneuver::msg::FullPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FullPose_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const FullPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FullPose_

// alias to use template instance with default allocator
using FullPose =
  maneuver::msg::FullPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace maneuver

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_HPP_
