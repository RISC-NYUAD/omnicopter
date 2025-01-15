// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from maneuver:srv/LiftOff.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/lift_off.hpp"


#ifndef MANEUVER__SRV__DETAIL__LIFT_OFF__STRUCT_HPP_
#define MANEUVER__SRV__DETAIL__LIFT_OFF__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__maneuver__srv__LiftOff_Request __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__LiftOff_Request __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LiftOff_Request_
{
  using Type = LiftOff_Request_<ContainerAllocator>;

  explicit LiftOff_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0.0f;
      this->duration = 0.0f;
    }
  }

  explicit LiftOff_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0.0f;
      this->duration = 0.0f;
    }
  }

  // field types and members
  using _height_type =
    float;
  _height_type height;
  using _duration_type =
    float;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
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
    maneuver::srv::LiftOff_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::LiftOff_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__LiftOff_Request
    std::shared_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__LiftOff_Request
    std::shared_ptr<maneuver::srv::LiftOff_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LiftOff_Request_ & other) const
  {
    if (this->height != other.height) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const LiftOff_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LiftOff_Request_

// alias to use template instance with default allocator
using LiftOff_Request =
  maneuver::srv::LiftOff_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver


#ifndef _WIN32
# define DEPRECATED__maneuver__srv__LiftOff_Response __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__LiftOff_Response __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LiftOff_Response_
{
  using Type = LiftOff_Response_<ContainerAllocator>;

  explicit LiftOff_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit LiftOff_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    maneuver::srv::LiftOff_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::LiftOff_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__LiftOff_Response
    std::shared_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__LiftOff_Response
    std::shared_ptr<maneuver::srv::LiftOff_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LiftOff_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const LiftOff_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LiftOff_Response_

// alias to use template instance with default allocator
using LiftOff_Response =
  maneuver::srv::LiftOff_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__maneuver__srv__LiftOff_Event __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__LiftOff_Event __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LiftOff_Event_
{
  using Type = LiftOff_Event_<ContainerAllocator>;

  explicit LiftOff_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit LiftOff_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<maneuver::srv::LiftOff_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::LiftOff_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<maneuver::srv::LiftOff_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::LiftOff_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<maneuver::srv::LiftOff_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::LiftOff_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<maneuver::srv::LiftOff_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::LiftOff_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    maneuver::srv::LiftOff_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::LiftOff_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::LiftOff_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__LiftOff_Event
    std::shared_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__LiftOff_Event
    std::shared_ptr<maneuver::srv::LiftOff_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LiftOff_Event_ & other) const
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
  bool operator!=(const LiftOff_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LiftOff_Event_

// alias to use template instance with default allocator
using LiftOff_Event =
  maneuver::srv::LiftOff_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver

namespace maneuver
{

namespace srv
{

struct LiftOff
{
  using Request = maneuver::srv::LiftOff_Request;
  using Response = maneuver::srv::LiftOff_Response;
  using Event = maneuver::srv::LiftOff_Event;
};

}  // namespace srv

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__LIFT_OFF__STRUCT_HPP_
