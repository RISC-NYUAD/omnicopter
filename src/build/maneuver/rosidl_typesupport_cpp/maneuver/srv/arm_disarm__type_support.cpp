// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from maneuver:srv/ArmDisarm.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "maneuver/srv/detail/arm_disarm__functions.h"
#include "maneuver/srv/detail/arm_disarm__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ArmDisarm_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ArmDisarm_Request_type_support_ids_t;

static const _ArmDisarm_Request_type_support_ids_t _ArmDisarm_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ArmDisarm_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ArmDisarm_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ArmDisarm_Request_type_support_symbol_names_t _ArmDisarm_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, ArmDisarm_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, ArmDisarm_Request)),
  }
};

typedef struct _ArmDisarm_Request_type_support_data_t
{
  void * data[2];
} _ArmDisarm_Request_type_support_data_t;

static _ArmDisarm_Request_type_support_data_t _ArmDisarm_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ArmDisarm_Request_message_typesupport_map = {
  2,
  "maneuver",
  &_ArmDisarm_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ArmDisarm_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ArmDisarm_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ArmDisarm_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ArmDisarm_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__ArmDisarm_Request__get_type_hash,
  &maneuver__srv__ArmDisarm_Request__get_type_description,
  &maneuver__srv__ArmDisarm_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::ArmDisarm_Request>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::ArmDisarm_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, ArmDisarm_Request)() {
  return get_message_type_support_handle<maneuver::srv::ArmDisarm_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/arm_disarm__functions.h"
// already included above
// #include "maneuver/srv/detail/arm_disarm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ArmDisarm_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ArmDisarm_Response_type_support_ids_t;

static const _ArmDisarm_Response_type_support_ids_t _ArmDisarm_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ArmDisarm_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ArmDisarm_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ArmDisarm_Response_type_support_symbol_names_t _ArmDisarm_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, ArmDisarm_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, ArmDisarm_Response)),
  }
};

typedef struct _ArmDisarm_Response_type_support_data_t
{
  void * data[2];
} _ArmDisarm_Response_type_support_data_t;

static _ArmDisarm_Response_type_support_data_t _ArmDisarm_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ArmDisarm_Response_message_typesupport_map = {
  2,
  "maneuver",
  &_ArmDisarm_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ArmDisarm_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ArmDisarm_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ArmDisarm_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ArmDisarm_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__ArmDisarm_Response__get_type_hash,
  &maneuver__srv__ArmDisarm_Response__get_type_description,
  &maneuver__srv__ArmDisarm_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::ArmDisarm_Response>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::ArmDisarm_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, ArmDisarm_Response)() {
  return get_message_type_support_handle<maneuver::srv::ArmDisarm_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/arm_disarm__functions.h"
// already included above
// #include "maneuver/srv/detail/arm_disarm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ArmDisarm_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ArmDisarm_Event_type_support_ids_t;

static const _ArmDisarm_Event_type_support_ids_t _ArmDisarm_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ArmDisarm_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ArmDisarm_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ArmDisarm_Event_type_support_symbol_names_t _ArmDisarm_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, ArmDisarm_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, ArmDisarm_Event)),
  }
};

typedef struct _ArmDisarm_Event_type_support_data_t
{
  void * data[2];
} _ArmDisarm_Event_type_support_data_t;

static _ArmDisarm_Event_type_support_data_t _ArmDisarm_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ArmDisarm_Event_message_typesupport_map = {
  2,
  "maneuver",
  &_ArmDisarm_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ArmDisarm_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ArmDisarm_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ArmDisarm_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ArmDisarm_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__ArmDisarm_Event__get_type_hash,
  &maneuver__srv__ArmDisarm_Event__get_type_description,
  &maneuver__srv__ArmDisarm_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::ArmDisarm_Event>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::ArmDisarm_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, ArmDisarm_Event)() {
  return get_message_type_support_handle<maneuver::srv::ArmDisarm_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "maneuver/srv/detail/arm_disarm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ArmDisarm_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ArmDisarm_type_support_ids_t;

static const _ArmDisarm_type_support_ids_t _ArmDisarm_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ArmDisarm_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ArmDisarm_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ArmDisarm_type_support_symbol_names_t _ArmDisarm_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, ArmDisarm)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, ArmDisarm)),
  }
};

typedef struct _ArmDisarm_type_support_data_t
{
  void * data[2];
} _ArmDisarm_type_support_data_t;

static _ArmDisarm_type_support_data_t _ArmDisarm_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ArmDisarm_service_typesupport_map = {
  2,
  "maneuver",
  &_ArmDisarm_service_typesupport_ids.typesupport_identifier[0],
  &_ArmDisarm_service_typesupport_symbol_names.symbol_name[0],
  &_ArmDisarm_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ArmDisarm_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ArmDisarm_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::ArmDisarm_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::ArmDisarm_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::ArmDisarm_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<maneuver::srv::ArmDisarm>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<maneuver::srv::ArmDisarm>,
  &maneuver__srv__ArmDisarm__get_type_hash,
  &maneuver__srv__ArmDisarm__get_type_description,
  &maneuver__srv__ArmDisarm__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<maneuver::srv::ArmDisarm>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::ArmDisarm_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, ArmDisarm)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<maneuver::srv::ArmDisarm>();
}

#ifdef __cplusplus
}
#endif
