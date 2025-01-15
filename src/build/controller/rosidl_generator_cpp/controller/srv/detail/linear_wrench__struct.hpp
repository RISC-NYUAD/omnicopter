// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/linear_wrench.hpp"


#ifndef CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__LinearWrench_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__LinearWrench_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LinearWrench_Request_
{
  using Type = LinearWrench_Request_<ContainerAllocator>;

  explicit LinearWrench_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx1 = 0.0f;
      this->fy1 = 0.0f;
      this->fz1 = 0.0f;
      this->fx2 = 0.0f;
      this->fy2 = 0.0f;
      this->fz2 = 0.0f;
      this->ramp = 0.0f;
      this->duration = 0.0f;
    }
  }

  explicit LinearWrench_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx1 = 0.0f;
      this->fy1 = 0.0f;
      this->fz1 = 0.0f;
      this->fx2 = 0.0f;
      this->fy2 = 0.0f;
      this->fz2 = 0.0f;
      this->ramp = 0.0f;
      this->duration = 0.0f;
    }
  }

  // field types and members
  using _fx1_type =
    float;
  _fx1_type fx1;
  using _fy1_type =
    float;
  _fy1_type fy1;
  using _fz1_type =
    float;
  _fz1_type fz1;
  using _fx2_type =
    float;
  _fx2_type fx2;
  using _fy2_type =
    float;
  _fy2_type fy2;
  using _fz2_type =
    float;
  _fz2_type fz2;
  using _ramp_type =
    float;
  _ramp_type ramp;
  using _duration_type =
    float;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__fx1(
    const float & _arg)
  {
    this->fx1 = _arg;
    return *this;
  }
  Type & set__fy1(
    const float & _arg)
  {
    this->fy1 = _arg;
    return *this;
  }
  Type & set__fz1(
    const float & _arg)
  {
    this->fz1 = _arg;
    return *this;
  }
  Type & set__fx2(
    const float & _arg)
  {
    this->fx2 = _arg;
    return *this;
  }
  Type & set__fy2(
    const float & _arg)
  {
    this->fy2 = _arg;
    return *this;
  }
  Type & set__fz2(
    const float & _arg)
  {
    this->fz2 = _arg;
    return *this;
  }
  Type & set__ramp(
    const float & _arg)
  {
    this->ramp = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::LinearWrench_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::LinearWrench_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__LinearWrench_Request
    std::shared_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__LinearWrench_Request
    std::shared_ptr<controller::srv::LinearWrench_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearWrench_Request_ & other) const
  {
    if (this->fx1 != other.fx1) {
      return false;
    }
    if (this->fy1 != other.fy1) {
      return false;
    }
    if (this->fz1 != other.fz1) {
      return false;
    }
    if (this->fx2 != other.fx2) {
      return false;
    }
    if (this->fy2 != other.fy2) {
      return false;
    }
    if (this->fz2 != other.fz2) {
      return false;
    }
    if (this->ramp != other.ramp) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearWrench_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearWrench_Request_

// alias to use template instance with default allocator
using LinearWrench_Request =
  controller::srv::LinearWrench_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__LinearWrench_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__LinearWrench_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LinearWrench_Response_
{
  using Type = LinearWrench_Response_<ContainerAllocator>;

  explicit LinearWrench_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit LinearWrench_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::LinearWrench_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::LinearWrench_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__LinearWrench_Response
    std::shared_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__LinearWrench_Response
    std::shared_ptr<controller::srv::LinearWrench_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearWrench_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearWrench_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearWrench_Response_

// alias to use template instance with default allocator
using LinearWrench_Response =
  controller::srv::LinearWrench_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__LinearWrench_Event __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__LinearWrench_Event __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LinearWrench_Event_
{
  using Type = LinearWrench_Event_<ContainerAllocator>;

  explicit LinearWrench_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit LinearWrench_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<controller::srv::LinearWrench_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::srv::LinearWrench_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<controller::srv::LinearWrench_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::srv::LinearWrench_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<controller::srv::LinearWrench_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::srv::LinearWrench_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<controller::srv::LinearWrench_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::srv::LinearWrench_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::LinearWrench_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::LinearWrench_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearWrench_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__LinearWrench_Event
    std::shared_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__LinearWrench_Event
    std::shared_ptr<controller::srv::LinearWrench_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearWrench_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearWrench_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearWrench_Event_

// alias to use template instance with default allocator
using LinearWrench_Event =
  controller::srv::LinearWrench_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct LinearWrench
{
  using Request = controller::srv::LinearWrench_Request;
  using Response = controller::srv::LinearWrench_Response;
  using Event = controller::srv::LinearWrench_Event;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_HPP_
