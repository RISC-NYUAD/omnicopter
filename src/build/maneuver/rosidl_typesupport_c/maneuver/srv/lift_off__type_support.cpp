// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from maneuver:srv/LiftOff.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "maneuver/srv/detail/lift_off__struct.h"
#include "maneuver/srv/detail/lift_off__type_support.h"
#include "maneuver/srv/detail/lift_off__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LiftOff_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LiftOff_Request_type_support_ids_t;

static const _LiftOff_Request_type_support_ids_t _LiftOff_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LiftOff_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LiftOff_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LiftOff_Request_type_support_symbol_names_t _LiftOff_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, LiftOff_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, LiftOff_Request)),
  }
};

typedef struct _LiftOff_Request_type_support_data_t
{
  void * data[2];
} _LiftOff_Request_type_support_data_t;

static _LiftOff_Request_type_support_data_t _LiftOff_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LiftOff_Request_message_typesupport_map = {
  2,
  "maneuver",
  &_LiftOff_Request_message_typesupport_ids.typesupport_identifier[0],
  &_LiftOff_Request_message_typesupport_symbol_names.symbol_name[0],
  &_LiftOff_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LiftOff_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LiftOff_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__LiftOff_Request__get_type_hash,
  &maneuver__srv__LiftOff_Request__get_type_description,
  &maneuver__srv__LiftOff_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, LiftOff_Request)() {
  return &::maneuver::srv::rosidl_typesupport_c::LiftOff_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/lift_off__struct.h"
// already included above
// #include "maneuver/srv/detail/lift_off__type_support.h"
// already included above
// #include "maneuver/srv/detail/lift_off__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LiftOff_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LiftOff_Response_type_support_ids_t;

static const _LiftOff_Response_type_support_ids_t _LiftOff_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LiftOff_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LiftOff_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LiftOff_Response_type_support_symbol_names_t _LiftOff_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, LiftOff_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, LiftOff_Response)),
  }
};

typedef struct _LiftOff_Response_type_support_data_t
{
  void * data[2];
} _LiftOff_Response_type_support_data_t;

static _LiftOff_Response_type_support_data_t _LiftOff_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LiftOff_Response_message_typesupport_map = {
  2,
  "maneuver",
  &_LiftOff_Response_message_typesupport_ids.typesupport_identifier[0],
  &_LiftOff_Response_message_typesupport_symbol_names.symbol_name[0],
  &_LiftOff_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LiftOff_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LiftOff_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__LiftOff_Response__get_type_hash,
  &maneuver__srv__LiftOff_Response__get_type_description,
  &maneuver__srv__LiftOff_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, LiftOff_Response)() {
  return &::maneuver::srv::rosidl_typesupport_c::LiftOff_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/lift_off__struct.h"
// already included above
// #include "maneuver/srv/detail/lift_off__type_support.h"
// already included above
// #include "maneuver/srv/detail/lift_off__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LiftOff_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LiftOff_Event_type_support_ids_t;

static const _LiftOff_Event_type_support_ids_t _LiftOff_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LiftOff_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LiftOff_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LiftOff_Event_type_support_symbol_names_t _LiftOff_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, LiftOff_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, LiftOff_Event)),
  }
};

typedef struct _LiftOff_Event_type_support_data_t
{
  void * data[2];
} _LiftOff_Event_type_support_data_t;

static _LiftOff_Event_type_support_data_t _LiftOff_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LiftOff_Event_message_typesupport_map = {
  2,
  "maneuver",
  &_LiftOff_Event_message_typesupport_ids.typesupport_identifier[0],
  &_LiftOff_Event_message_typesupport_symbol_names.symbol_name[0],
  &_LiftOff_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LiftOff_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LiftOff_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__LiftOff_Event__get_type_hash,
  &maneuver__srv__LiftOff_Event__get_type_description,
  &maneuver__srv__LiftOff_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, LiftOff_Event)() {
  return &::maneuver::srv::rosidl_typesupport_c::LiftOff_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/lift_off__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace maneuver
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _LiftOff_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LiftOff_type_support_ids_t;

static const _LiftOff_type_support_ids_t _LiftOff_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LiftOff_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LiftOff_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LiftOff_type_support_symbol_names_t _LiftOff_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, LiftOff)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, LiftOff)),
  }
};

typedef struct _LiftOff_type_support_data_t
{
  void * data[2];
} _LiftOff_type_support_data_t;

static _LiftOff_type_support_data_t _LiftOff_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LiftOff_service_typesupport_map = {
  2,
  "maneuver",
  &_LiftOff_service_typesupport_ids.typesupport_identifier[0],
  &_LiftOff_service_typesupport_symbol_names.symbol_name[0],
  &_LiftOff_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t LiftOff_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LiftOff_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &LiftOff_Request_message_type_support_handle,
  &LiftOff_Response_message_type_support_handle,
  &LiftOff_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    LiftOff
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    LiftOff
  ),
  &maneuver__srv__LiftOff__get_type_hash,
  &maneuver__srv__LiftOff__get_type_description,
  &maneuver__srv__LiftOff__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, LiftOff)() {
  return &::maneuver::srv::rosidl_typesupport_c::LiftOff_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
