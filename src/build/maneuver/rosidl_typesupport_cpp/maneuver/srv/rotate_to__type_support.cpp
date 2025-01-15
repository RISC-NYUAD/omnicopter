// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from maneuver:srv/RotateTo.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "maneuver/srv/detail/rotate_to__functions.h"
#include "maneuver/srv/detail/rotate_to__struct.hpp"
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

typedef struct _RotateTo_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RotateTo_Request_type_support_ids_t;

static const _RotateTo_Request_type_support_ids_t _RotateTo_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RotateTo_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RotateTo_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RotateTo_Request_type_support_symbol_names_t _RotateTo_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, RotateTo_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, RotateTo_Request)),
  }
};

typedef struct _RotateTo_Request_type_support_data_t
{
  void * data[2];
} _RotateTo_Request_type_support_data_t;

static _RotateTo_Request_type_support_data_t _RotateTo_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RotateTo_Request_message_typesupport_map = {
  2,
  "maneuver",
  &_RotateTo_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RotateTo_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RotateTo_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RotateTo_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RotateTo_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__RotateTo_Request__get_type_hash,
  &maneuver__srv__RotateTo_Request__get_type_description,
  &maneuver__srv__RotateTo_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::RotateTo_Request>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::RotateTo_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, RotateTo_Request)() {
  return get_message_type_support_handle<maneuver::srv::RotateTo_Request>();
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
// #include "maneuver/srv/detail/rotate_to__functions.h"
// already included above
// #include "maneuver/srv/detail/rotate_to__struct.hpp"
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

typedef struct _RotateTo_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RotateTo_Response_type_support_ids_t;

static const _RotateTo_Response_type_support_ids_t _RotateTo_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RotateTo_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RotateTo_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RotateTo_Response_type_support_symbol_names_t _RotateTo_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, RotateTo_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, RotateTo_Response)),
  }
};

typedef struct _RotateTo_Response_type_support_data_t
{
  void * data[2];
} _RotateTo_Response_type_support_data_t;

static _RotateTo_Response_type_support_data_t _RotateTo_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RotateTo_Response_message_typesupport_map = {
  2,
  "maneuver",
  &_RotateTo_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RotateTo_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RotateTo_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RotateTo_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RotateTo_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__RotateTo_Response__get_type_hash,
  &maneuver__srv__RotateTo_Response__get_type_description,
  &maneuver__srv__RotateTo_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::RotateTo_Response>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::RotateTo_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, RotateTo_Response)() {
  return get_message_type_support_handle<maneuver::srv::RotateTo_Response>();
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
// #include "maneuver/srv/detail/rotate_to__functions.h"
// already included above
// #include "maneuver/srv/detail/rotate_to__struct.hpp"
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

typedef struct _RotateTo_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RotateTo_Event_type_support_ids_t;

static const _RotateTo_Event_type_support_ids_t _RotateTo_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RotateTo_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RotateTo_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RotateTo_Event_type_support_symbol_names_t _RotateTo_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, RotateTo_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, RotateTo_Event)),
  }
};

typedef struct _RotateTo_Event_type_support_data_t
{
  void * data[2];
} _RotateTo_Event_type_support_data_t;

static _RotateTo_Event_type_support_data_t _RotateTo_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RotateTo_Event_message_typesupport_map = {
  2,
  "maneuver",
  &_RotateTo_Event_message_typesupport_ids.typesupport_identifier[0],
  &_RotateTo_Event_message_typesupport_symbol_names.symbol_name[0],
  &_RotateTo_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RotateTo_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RotateTo_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__RotateTo_Event__get_type_hash,
  &maneuver__srv__RotateTo_Event__get_type_description,
  &maneuver__srv__RotateTo_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::RotateTo_Event>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::RotateTo_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, RotateTo_Event)() {
  return get_message_type_support_handle<maneuver::srv::RotateTo_Event>();
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
// #include "maneuver/srv/detail/rotate_to__struct.hpp"
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

typedef struct _RotateTo_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RotateTo_type_support_ids_t;

static const _RotateTo_type_support_ids_t _RotateTo_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RotateTo_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RotateTo_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RotateTo_type_support_symbol_names_t _RotateTo_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, RotateTo)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, RotateTo)),
  }
};

typedef struct _RotateTo_type_support_data_t
{
  void * data[2];
} _RotateTo_type_support_data_t;

static _RotateTo_type_support_data_t _RotateTo_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RotateTo_service_typesupport_map = {
  2,
  "maneuver",
  &_RotateTo_service_typesupport_ids.typesupport_identifier[0],
  &_RotateTo_service_typesupport_symbol_names.symbol_name[0],
  &_RotateTo_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RotateTo_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RotateTo_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::RotateTo_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::RotateTo_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::RotateTo_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<maneuver::srv::RotateTo>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<maneuver::srv::RotateTo>,
  &maneuver__srv__RotateTo__get_type_hash,
  &maneuver__srv__RotateTo__get_type_description,
  &maneuver__srv__RotateTo__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<maneuver::srv::RotateTo>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::RotateTo_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, RotateTo)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<maneuver::srv::RotateTo>();
}

#ifdef __cplusplus
}
#endif
