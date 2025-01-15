// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/uvector.hpp"


#ifndef CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__Uvector __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__Uvector __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Uvector_
{
  using Type = Uvector_<ContainerAllocator>;

  explicit Uvector_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 12>::iterator, float>(this->value.begin(), this->value.end(), 0.0f);
    }
  }

  explicit Uvector_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : value(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 12>::iterator, float>(this->value.begin(), this->value.end(), 0.0f);
    }
  }

  // field types and members
  using _value_type =
    std::array<float, 12>;
  _value_type value;

  // setters for named parameter idiom
  Type & set__value(
    const std::array<float, 12> & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::Uvector_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::Uvector_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::Uvector_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::Uvector_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::Uvector_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::Uvector_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::Uvector_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::Uvector_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::Uvector_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::Uvector_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__Uvector
    std::shared_ptr<controller::msg::Uvector_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__Uvector
    std::shared_ptr<controller::msg::Uvector_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Uvector_ & other) const
  {
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const Uvector_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Uvector_

// alias to use template instance with default allocator
using Uvector =
  controller::msg::Uvector_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_HPP_
