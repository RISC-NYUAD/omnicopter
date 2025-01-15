// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from maneuver:srv/Land.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/land.hpp"


#ifndef MANEUVER__SRV__DETAIL__LAND__STRUCT_HPP_
#define MANEUVER__SRV__DETAIL__LAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__maneuver__srv__Land_Request __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__Land_Request __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Land_Request_
{
  using Type = Land_Request_<ContainerAllocator>;

  explicit Land_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height_1 = 0.0f;
      this->duration_1 = 0.0f;
      this->height_2 = 0.0f;
      this->duration_2 = 0.0f;
    }
  }

  explicit Land_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height_1 = 0.0f;
      this->duration_1 = 0.0f;
      this->height_2 = 0.0f;
      this->duration_2 = 0.0f;
    }
  }

  // field types and members
  using _height_1_type =
    float;
  _height_1_type height_1;
  using _duration_1_type =
    float;
  _duration_1_type duration_1;
  using _height_2_type =
    float;
  _height_2_type height_2;
  using _duration_2_type =
    float;
  _duration_2_type duration_2;

  // setters for named parameter idiom
  Type & set__height_1(
    const float & _arg)
  {
    this->height_1 = _arg;
    return *this;
  }
  Type & set__duration_1(
    const float & _arg)
  {
    this->duration_1 = _arg;
    return *this;
  }
  Type & set__height_2(
    const float & _arg)
  {
    this->height_2 = _arg;
    return *this;
  }
  Type & set__duration_2(
    const float & _arg)
  {
    this->duration_2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    maneuver::srv::Land_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::Land_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::Land_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::Land_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::Land_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::Land_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__Land_Request
    std::shared_ptr<maneuver::srv::Land_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__Land_Request
    std::shared_ptr<maneuver::srv::Land_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Land_Request_ & other) const
  {
    if (this->height_1 != other.height_1) {
      return false;
    }
    if (this->duration_1 != other.duration_1) {
      return false;
    }
    if (this->height_2 != other.height_2) {
      return false;
    }
    if (this->duration_2 != other.duration_2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Land_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Land_Request_

// alias to use template instance with default allocator
using Land_Request =
  maneuver::srv::Land_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver


#ifndef _WIN32
# define DEPRECATED__maneuver__srv__Land_Response __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__Land_Response __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Land_Response_
{
  using Type = Land_Response_<ContainerAllocator>;

  explicit Land_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit Land_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    maneuver::srv::Land_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::Land_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::Land_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::Land_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::Land_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::Land_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__Land_Response
    std::shared_ptr<maneuver::srv::Land_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__Land_Response
    std::shared_ptr<maneuver::srv::Land_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Land_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Land_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Land_Response_

// alias to use template instance with default allocator
using Land_Response =
  maneuver::srv::Land_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__maneuver__srv__Land_Event __attribute__((deprecated))
#else
# define DEPRECATED__maneuver__srv__Land_Event __declspec(deprecated)
#endif

namespace maneuver
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Land_Event_
{
  using Type = Land_Event_<ContainerAllocator>;

  explicit Land_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Land_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<maneuver::srv::Land_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::Land_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<maneuver::srv::Land_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::Land_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<maneuver::srv::Land_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::Land_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<maneuver::srv::Land_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<maneuver::srv::Land_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    maneuver::srv::Land_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const maneuver::srv::Land_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<maneuver::srv::Land_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<maneuver::srv::Land_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      maneuver::srv::Land_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<maneuver::srv::Land_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<maneuver::srv::Land_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<maneuver::srv::Land_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__maneuver__srv__Land_Event
    std::shared_ptr<maneuver::srv::Land_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__maneuver__srv__Land_Event
    std::shared_ptr<maneuver::srv::Land_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Land_Event_ & other) const
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
  bool operator!=(const Land_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Land_Event_

// alias to use template instance with default allocator
using Land_Event =
  maneuver::srv::Land_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace maneuver

namespace maneuver
{

namespace srv
{

struct Land
{
  using Request = maneuver::srv::Land_Request;
  using Response = maneuver::srv::Land_Response;
  using Event = maneuver::srv::Land_Event;
};

}  // namespace srv

}  // namespace maneuver

#endif  // MANEUVER__SRV__DETAIL__LAND__STRUCT_HPP_
