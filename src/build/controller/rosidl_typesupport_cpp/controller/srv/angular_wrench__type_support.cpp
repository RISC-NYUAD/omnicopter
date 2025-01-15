// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from controller:srv/AngularWrench.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "controller/srv/detail/angular_wrench__functions.h"
#include "controller/srv/detail/angular_wrench__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _AngularWrench_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AngularWrench_Request_type_support_ids_t;

static const _AngularWrench_Request_type_support_ids_t _AngularWrench_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AngularWrench_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AngularWrench_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AngularWrench_Request_type_support_symbol_names_t _AngularWrench_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, AngularWrench_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, AngularWrench_Request)),
  }
};

typedef struct _AngularWrench_Request_type_support_data_t
{
  void * data[2];
} _AngularWrench_Request_type_support_data_t;

static _AngularWrench_Request_type_support_data_t _AngularWrench_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AngularWrench_Request_message_typesupport_map = {
  2,
  "controller",
  &_AngularWrench_Request_message_typesupport_ids.typesupport_identifier[0],
  &_AngularWrench_Request_message_typesupport_symbol_names.symbol_name[0],
  &_AngularWrench_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AngularWrench_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AngularWrench_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &controller__srv__AngularWrench_Request__get_type_hash,
  &controller__srv__AngularWrench_Request__get_type_description,
  &controller__srv__AngularWrench_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::AngularWrench_Request>()
{
  return &::controller::srv::rosidl_typesupport_cpp::AngularWrench_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, AngularWrench_Request)() {
  return get_message_type_support_handle<controller::srv::AngularWrench_Request>();
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
// #include "controller/srv/detail/angular_wrench__functions.h"
// already included above
// #include "controller/srv/detail/angular_wrench__struct.hpp"
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

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _AngularWrench_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AngularWrench_Response_type_support_ids_t;

static const _AngularWrench_Response_type_support_ids_t _AngularWrench_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AngularWrench_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AngularWrench_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AngularWrench_Response_type_support_symbol_names_t _AngularWrench_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, AngularWrench_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, AngularWrench_Response)),
  }
};

typedef struct _AngularWrench_Response_type_support_data_t
{
  void * data[2];
} _AngularWrench_Response_type_support_data_t;

static _AngularWrench_Response_type_support_data_t _AngularWrench_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AngularWrench_Response_message_typesupport_map = {
  2,
  "controller",
  &_AngularWrench_Response_message_typesupport_ids.typesupport_identifier[0],
  &_AngularWrench_Response_message_typesupport_symbol_names.symbol_name[0],
  &_AngularWrench_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AngularWrench_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AngularWrench_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &controller__srv__AngularWrench_Response__get_type_hash,
  &controller__srv__AngularWrench_Response__get_type_description,
  &controller__srv__AngularWrench_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::AngularWrench_Response>()
{
  return &::controller::srv::rosidl_typesupport_cpp::AngularWrench_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, AngularWrench_Response)() {
  return get_message_type_support_handle<controller::srv::AngularWrench_Response>();
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
// #include "controller/srv/detail/angular_wrench__functions.h"
// already included above
// #include "controller/srv/detail/angular_wrench__struct.hpp"
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

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _AngularWrench_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AngularWrench_Event_type_support_ids_t;

static const _AngularWrench_Event_type_support_ids_t _AngularWrench_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AngularWrench_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AngularWrench_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AngularWrench_Event_type_support_symbol_names_t _AngularWrench_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, AngularWrench_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, AngularWrench_Event)),
  }
};

typedef struct _AngularWrench_Event_type_support_data_t
{
  void * data[2];
} _AngularWrench_Event_type_support_data_t;

static _AngularWrench_Event_type_support_data_t _AngularWrench_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AngularWrench_Event_message_typesupport_map = {
  2,
  "controller",
  &_AngularWrench_Event_message_typesupport_ids.typesupport_identifier[0],
  &_AngularWrench_Event_message_typesupport_symbol_names.symbol_name[0],
  &_AngularWrench_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AngularWrench_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AngularWrench_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &controller__srv__AngularWrench_Event__get_type_hash,
  &controller__srv__AngularWrench_Event__get_type_description,
  &controller__srv__AngularWrench_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::AngularWrench_Event>()
{
  return &::controller::srv::rosidl_typesupport_cpp::AngularWrench_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, AngularWrench_Event)() {
  return get_message_type_support_handle<controller::srv::AngularWrench_Event>();
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
// #include "controller/srv/detail/angular_wrench__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _AngularWrench_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AngularWrench_type_support_ids_t;

static const _AngularWrench_type_support_ids_t _AngularWrench_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AngularWrench_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AngularWrench_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AngularWrench_type_support_symbol_names_t _AngularWrench_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, AngularWrench)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, AngularWrench)),
  }
};

typedef struct _AngularWrench_type_support_data_t
{
  void * data[2];
} _AngularWrench_type_support_data_t;

static _AngularWrench_type_support_data_t _AngularWrench_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AngularWrench_service_typesupport_map = {
  2,
  "controller",
  &_AngularWrench_service_typesupport_ids.typesupport_identifier[0],
  &_AngularWrench_service_typesupport_symbol_names.symbol_name[0],
  &_AngularWrench_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t AngularWrench_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AngularWrench_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<controller::srv::AngularWrench_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<controller::srv::AngularWrench_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<controller::srv::AngularWrench_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<controller::srv::AngularWrench>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<controller::srv::AngularWrench>,
  &controller__srv__AngularWrench__get_type_hash,
  &controller__srv__AngularWrench__get_type_description,
  &controller__srv__AngularWrench__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<controller::srv::AngularWrench>()
{
  return &::controller::srv::rosidl_typesupport_cpp::AngularWrench_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, AngularWrench)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<controller::srv::AngularWrench>();
}

#ifdef __cplusplus
}
#endif
