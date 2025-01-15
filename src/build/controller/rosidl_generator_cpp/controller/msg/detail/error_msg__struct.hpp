// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/error_msg.hpp"


#ifndef CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'ex'
// Member 'ev'
// Member 'ea'
// Member 'er'
// Member 'ew'
// Member 'iex'
// Member 'ier'
// Member 'accd'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__msg__ErrorMsg __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__ErrorMsg __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ErrorMsg_
{
  using Type = ErrorMsg_<ContainerAllocator>;

  explicit ErrorMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ex(_init),
    ev(_init),
    ea(_init),
    er(_init),
    ew(_init),
    iex(_init),
    ier(_init),
    accd(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->weight = 0.0f;
    }
  }

  explicit ErrorMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ex(_alloc, _init),
    ev(_alloc, _init),
    ea(_alloc, _init),
    er(_alloc, _init),
    ew(_alloc, _init),
    iex(_alloc, _init),
    ier(_alloc, _init),
    accd(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->weight = 0.0f;
    }
  }

  // field types and members
  using _ex_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _ex_type ex;
  using _ev_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _ev_type ev;
  using _ea_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _ea_type ea;
  using _er_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _er_type er;
  using _ew_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _ew_type ew;
  using _iex_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _iex_type iex;
  using _ier_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _ier_type ier;
  using _accd_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _accd_type accd;
  using _wrench_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _wrench_type wrench;
  using _exwrench_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _exwrench_type exwrench;
  using _prop_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _prop_type prop;
  using _weight_type =
    float;
  _weight_type weight;

  // setters for named parameter idiom
  Type & set__ex(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->ex = _arg;
    return *this;
  }
  Type & set__ev(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->ev = _arg;
    return *this;
  }
  Type & set__ea(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->ea = _arg;
    return *this;
  }
  Type & set__er(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->er = _arg;
    return *this;
  }
  Type & set__ew(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->ew = _arg;
    return *this;
  }
  Type & set__iex(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->iex = _arg;
    return *this;
  }
  Type & set__ier(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->ier = _arg;
    return *this;
  }
  Type & set__accd(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->accd = _arg;
    return *this;
  }
  Type & set__wrench(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }
  Type & set__exwrench(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->exwrench = _arg;
    return *this;
  }
  Type & set__prop(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->prop = _arg;
    return *this;
  }
  Type & set__weight(
    const float & _arg)
  {
    this->weight = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::ErrorMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::ErrorMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::ErrorMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::ErrorMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::ErrorMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::ErrorMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::ErrorMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::ErrorMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::ErrorMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::ErrorMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__ErrorMsg
    std::shared_ptr<controller::msg::ErrorMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__ErrorMsg
    std::shared_ptr<controller::msg::ErrorMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ErrorMsg_ & other) const
  {
    if (this->ex != other.ex) {
      return false;
    }
    if (this->ev != other.ev) {
      return false;
    }
    if (this->ea != other.ea) {
      return false;
    }
    if (this->er != other.er) {
      return false;
    }
    if (this->ew != other.ew) {
      return false;
    }
    if (this->iex != other.iex) {
      return false;
    }
    if (this->ier != other.ier) {
      return false;
    }
    if (this->accd != other.accd) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    if (this->exwrench != other.exwrench) {
      return false;
    }
    if (this->prop != other.prop) {
      return false;
    }
    if (this->weight != other.weight) {
      return false;
    }
    return true;
  }
  bool operator!=(const ErrorMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ErrorMsg_

// alias to use template instance with default allocator
using ErrorMsg =
  controller::msg::ErrorMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_HPP_
