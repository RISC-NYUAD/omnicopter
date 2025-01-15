// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from maneuver:srv/GotoPoint.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "maneuver/srv/detail/goto_point__struct.h"
#include "maneuver/srv/detail/goto_point__type_support.h"
#include "maneuver/srv/detail/goto_point__functions.h"
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

typedef struct _GotoPoint_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GotoPoint_Request_type_support_ids_t;

static const _GotoPoint_Request_type_support_ids_t _GotoPoint_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GotoPoint_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GotoPoint_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GotoPoint_Request_type_support_symbol_names_t _GotoPoint_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, GotoPoint_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, GotoPoint_Request)),
  }
};

typedef struct _GotoPoint_Request_type_support_data_t
{
  void * data[2];
} _GotoPoint_Request_type_support_data_t;

static _GotoPoint_Request_type_support_data_t _GotoPoint_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GotoPoint_Request_message_typesupport_map = {
  2,
  "maneuver",
  &_GotoPoint_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GotoPoint_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GotoPoint_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GotoPoint_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GotoPoint_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__GotoPoint_Request__get_type_hash,
  &maneuver__srv__GotoPoint_Request__get_type_description,
  &maneuver__srv__GotoPoint_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, GotoPoint_Request)() {
  return &::maneuver::srv::rosidl_typesupport_c::GotoPoint_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/goto_point__struct.h"
// already included above
// #include "maneuver/srv/detail/goto_point__type_support.h"
// already included above
// #include "maneuver/srv/detail/goto_point__functions.h"
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

typedef struct _GotoPoint_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GotoPoint_Response_type_support_ids_t;

static const _GotoPoint_Response_type_support_ids_t _GotoPoint_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GotoPoint_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GotoPoint_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GotoPoint_Response_type_support_symbol_names_t _GotoPoint_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, GotoPoint_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, GotoPoint_Response)),
  }
};

typedef struct _GotoPoint_Response_type_support_data_t
{
  void * data[2];
} _GotoPoint_Response_type_support_data_t;

static _GotoPoint_Response_type_support_data_t _GotoPoint_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GotoPoint_Response_message_typesupport_map = {
  2,
  "maneuver",
  &_GotoPoint_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GotoPoint_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GotoPoint_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GotoPoint_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GotoPoint_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__GotoPoint_Response__get_type_hash,
  &maneuver__srv__GotoPoint_Response__get_type_description,
  &maneuver__srv__GotoPoint_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, GotoPoint_Response)() {
  return &::maneuver::srv::rosidl_typesupport_c::GotoPoint_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/goto_point__struct.h"
// already included above
// #include "maneuver/srv/detail/goto_point__type_support.h"
// already included above
// #include "maneuver/srv/detail/goto_point__functions.h"
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

typedef struct _GotoPoint_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GotoPoint_Event_type_support_ids_t;

static const _GotoPoint_Event_type_support_ids_t _GotoPoint_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GotoPoint_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GotoPoint_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GotoPoint_Event_type_support_symbol_names_t _GotoPoint_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, GotoPoint_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, GotoPoint_Event)),
  }
};

typedef struct _GotoPoint_Event_type_support_data_t
{
  void * data[2];
} _GotoPoint_Event_type_support_data_t;

static _GotoPoint_Event_type_support_data_t _GotoPoint_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GotoPoint_Event_message_typesupport_map = {
  2,
  "maneuver",
  &_GotoPoint_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GotoPoint_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GotoPoint_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GotoPoint_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GotoPoint_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &maneuver__srv__GotoPoint_Event__get_type_hash,
  &maneuver__srv__GotoPoint_Event__get_type_description,
  &maneuver__srv__GotoPoint_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, GotoPoint_Event)() {
  return &::maneuver::srv::rosidl_typesupport_c::GotoPoint_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "maneuver/srv/detail/goto_point__type_support.h"
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
typedef struct _GotoPoint_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GotoPoint_type_support_ids_t;

static const _GotoPoint_type_support_ids_t _GotoPoint_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GotoPoint_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GotoPoint_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GotoPoint_type_support_symbol_names_t _GotoPoint_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, srv, GotoPoint)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, GotoPoint)),
  }
};

typedef struct _GotoPoint_type_support_data_t
{
  void * data[2];
} _GotoPoint_type_support_data_t;

static _GotoPoint_type_support_data_t _GotoPoint_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GotoPoint_service_typesupport_map = {
  2,
  "maneuver",
  &_GotoPoint_service_typesupport_ids.typesupport_identifier[0],
  &_GotoPoint_service_typesupport_symbol_names.symbol_name[0],
  &_GotoPoint_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GotoPoint_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GotoPoint_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &GotoPoint_Request_message_type_support_handle,
  &GotoPoint_Response_message_type_support_handle,
  &GotoPoint_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    GotoPoint
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    GotoPoint
  ),
  &maneuver__srv__GotoPoint__get_type_hash,
  &maneuver__srv__GotoPoint__get_type_description,
  &maneuver__srv__GotoPoint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, maneuver, srv, GotoPoint)() {
  return &::maneuver::srv::rosidl_typesupport_c::GotoPoint_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
