// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/motor_speed.hpp"


#ifndef CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__MotorSpeed __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__MotorSpeed __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSpeed_
{
  using Type = MotorSpeed_<ContainerAllocator>;

  explicit MotorSpeed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorSpeed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _velocity_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::MotorSpeed_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::MotorSpeed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::MotorSpeed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::MotorSpeed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::MotorSpeed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MotorSpeed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::MotorSpeed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MotorSpeed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::MotorSpeed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::MotorSpeed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__MotorSpeed
    std::shared_ptr<controller::msg::MotorSpeed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__MotorSpeed
    std::shared_ptr<controller::msg::MotorSpeed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSpeed_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSpeed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSpeed_

// alias to use template instance with default allocator
using MotorSpeed =
  controller::msg::MotorSpeed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_HPP_
