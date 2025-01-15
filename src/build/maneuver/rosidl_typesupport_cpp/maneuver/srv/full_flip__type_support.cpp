// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from maneuver:srv/FullFlip.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "maneuver/srv/detail/full_flip__functions.h"
#include "maneuver/srv/detail/full_flip__struct.hpp"
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

typedef struct _FullFlip_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FullFlip_Request_type_support_ids_t;

static const _FullFlip_Request_type_support_ids_t _FullFlip_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FullFlip_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FullFlip_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FullFlip_Request_type_support_symbol_names_t _FullFlip_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, FullFlip_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, FullFlip_Request)),
  }
};

typedef struct _FullFlip_Request_type_support_data_t
{
  void * data[2];
} _FullFlip_Request_type_support_data_t;

static _FullFlip_Request_type_support_data_t _FullFlip_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FullFlip_Request_message_typesupport_map = {
  2,
  "maneuver",
  &_FullFlip_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FullFlip_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FullFlip_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FullFlip_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FullFlip_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__FullFlip_Request__get_type_hash,
  &maneuver__srv__FullFlip_Request__get_type_description,
  &maneuver__srv__FullFlip_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::FullFlip_Request>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::FullFlip_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, FullFlip_Request)() {
  return get_message_type_support_handle<maneuver::srv::FullFlip_Request>();
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
// #include "maneuver/srv/detail/full_flip__functions.h"
// already included above
// #include "maneuver/srv/detail/full_flip__struct.hpp"
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

typedef struct _FullFlip_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FullFlip_Response_type_support_ids_t;

static const _FullFlip_Response_type_support_ids_t _FullFlip_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FullFlip_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FullFlip_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FullFlip_Response_type_support_symbol_names_t _FullFlip_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, FullFlip_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, FullFlip_Response)),
  }
};

typedef struct _FullFlip_Response_type_support_data_t
{
  void * data[2];
} _FullFlip_Response_type_support_data_t;

static _FullFlip_Response_type_support_data_t _FullFlip_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FullFlip_Response_message_typesupport_map = {
  2,
  "maneuver",
  &_FullFlip_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FullFlip_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FullFlip_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FullFlip_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FullFlip_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__FullFlip_Response__get_type_hash,
  &maneuver__srv__FullFlip_Response__get_type_description,
  &maneuver__srv__FullFlip_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::FullFlip_Response>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::FullFlip_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, FullFlip_Response)() {
  return get_message_type_support_handle<maneuver::srv::FullFlip_Response>();
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
// #include "maneuver/srv/detail/full_flip__functions.h"
// already included above
// #include "maneuver/srv/detail/full_flip__struct.hpp"
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

typedef struct _FullFlip_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FullFlip_Event_type_support_ids_t;

static const _FullFlip_Event_type_support_ids_t _FullFlip_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FullFlip_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FullFlip_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FullFlip_Event_type_support_symbol_names_t _FullFlip_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, FullFlip_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, FullFlip_Event)),
  }
};

typedef struct _FullFlip_Event_type_support_data_t
{
  void * data[2];
} _FullFlip_Event_type_support_data_t;

static _FullFlip_Event_type_support_data_t _FullFlip_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FullFlip_Event_message_typesupport_map = {
  2,
  "maneuver",
  &_FullFlip_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FullFlip_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FullFlip_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FullFlip_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FullFlip_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &maneuver__srv__FullFlip_Event__get_type_hash,
  &maneuver__srv__FullFlip_Event__get_type_description,
  &maneuver__srv__FullFlip_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<maneuver::srv::FullFlip_Event>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::FullFlip_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, FullFlip_Event)() {
  return get_message_type_support_handle<maneuver::srv::FullFlip_Event>();
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
// #include "maneuver/srv/detail/full_flip__struct.hpp"
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

typedef struct _FullFlip_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FullFlip_type_support_ids_t;

static const _FullFlip_type_support_ids_t _FullFlip_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FullFlip_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FullFlip_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FullFlip_type_support_symbol_names_t _FullFlip_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, srv, FullFlip)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, maneuver, srv, FullFlip)),
  }
};

typedef struct _FullFlip_type_support_data_t
{
  void * data[2];
} _FullFlip_type_support_data_t;

static _FullFlip_type_support_data_t _FullFlip_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FullFlip_service_typesupport_map = {
  2,
  "maneuver",
  &_FullFlip_service_typesupport_ids.typesupport_identifier[0],
  &_FullFlip_service_typesupport_symbol_names.symbol_name[0],
  &_FullFlip_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FullFlip_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FullFlip_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::FullFlip_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::FullFlip_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<maneuver::srv::FullFlip_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<maneuver::srv::FullFlip>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<maneuver::srv::FullFlip>,
  &maneuver__srv__FullFlip__get_type_hash,
  &maneuver__srv__FullFlip__get_type_description,
  &maneuver__srv__FullFlip__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace maneuver

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<maneuver::srv::FullFlip>()
{
  return &::maneuver::srv::rosidl_typesupport_cpp::FullFlip_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, maneuver, srv, FullFlip)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<maneuver::srv::FullFlip>();
}

#ifdef __cplusplus
}
#endif
