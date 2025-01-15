// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

#include "controller/srv/detail/linear_wrench__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__LinearWrench__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0x53, 0x10, 0x28, 0x72, 0xbc, 0xa6, 0xef,
      0x05, 0x5b, 0xfd, 0xbf, 0xac, 0x48, 0xeb, 0x59,
      0x6a, 0x92, 0xb3, 0x0e, 0x77, 0x29, 0x4c, 0x00,
      0x75, 0x4d, 0x9e, 0x03, 0x72, 0x65, 0x18, 0x61,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__LinearWrench_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa7, 0xe9, 0x4d, 0xb3, 0xac, 0x06, 0xe9, 0xd5,
      0xaa, 0x0d, 0xbd, 0xea, 0xb9, 0x2a, 0x5c, 0xb5,
      0x60, 0x89, 0xe9, 0x65, 0x57, 0x9a, 0xaa, 0xb6,
      0x21, 0x46, 0x41, 0xc5, 0x66, 0xa5, 0xe1, 0x21,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__LinearWrench_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x13, 0xa0, 0xf0, 0xcb, 0xfb, 0x0e, 0x5c, 0x21,
      0xe8, 0xb1, 0xf0, 0x07, 0x70, 0x83, 0xa6, 0xb9,
      0xb6, 0x4d, 0x44, 0xbd, 0x1d, 0x39, 0x18, 0x83,
      0x06, 0x4c, 0x08, 0xea, 0xb1, 0x70, 0x30, 0x36,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__LinearWrench_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0x77, 0x62, 0x7c, 0x6a, 0xdc, 0x74, 0x88,
      0x0a, 0x06, 0x3c, 0xb9, 0xad, 0x98, 0xe9, 0x69,
      0xce, 0xd8, 0x3a, 0x3e, 0x73, 0x6c, 0x09, 0x09,
      0xbb, 0xbf, 0xfd, 0x86, 0xf7, 0xd5, 0xfb, 0xc8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char controller__srv__LinearWrench__TYPE_NAME[] = "controller/srv/LinearWrench";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char controller__srv__LinearWrench_Event__TYPE_NAME[] = "controller/srv/LinearWrench_Event";
static char controller__srv__LinearWrench_Request__TYPE_NAME[] = "controller/srv/LinearWrench_Request";
static char controller__srv__LinearWrench_Response__TYPE_NAME[] = "controller/srv/LinearWrench_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char controller__srv__LinearWrench__FIELD_NAME__request_message[] = "request_message";
static char controller__srv__LinearWrench__FIELD_NAME__response_message[] = "response_message";
static char controller__srv__LinearWrench__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field controller__srv__LinearWrench__FIELDS[] = {
  {
    {controller__srv__LinearWrench__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__LinearWrench_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription controller__srv__LinearWrench__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__LinearWrench__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__LinearWrench__TYPE_NAME, 27, 27},
      {controller__srv__LinearWrench__FIELDS, 3, 3},
    },
    {controller__srv__LinearWrench__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = controller__srv__LinearWrench_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = controller__srv__LinearWrench_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = controller__srv__LinearWrench_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__LinearWrench_Request__FIELD_NAME__fx1[] = "fx1";
static char controller__srv__LinearWrench_Request__FIELD_NAME__fy1[] = "fy1";
static char controller__srv__LinearWrench_Request__FIELD_NAME__fz1[] = "fz1";
static char controller__srv__LinearWrench_Request__FIELD_NAME__fx2[] = "fx2";
static char controller__srv__LinearWrench_Request__FIELD_NAME__fy2[] = "fy2";
static char controller__srv__LinearWrench_Request__FIELD_NAME__fz2[] = "fz2";
static char controller__srv__LinearWrench_Request__FIELD_NAME__ramp[] = "ramp";
static char controller__srv__LinearWrench_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field controller__srv__LinearWrench_Request__FIELDS[] = {
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fx1, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fy1, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fz1, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fx2, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fy2, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__fz2, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__ramp, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__LinearWrench_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
      {controller__srv__LinearWrench_Request__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__LinearWrench_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field controller__srv__LinearWrench_Response__FIELDS[] = {
  {
    {controller__srv__LinearWrench_Response__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__LinearWrench_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
      {controller__srv__LinearWrench_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__LinearWrench_Event__FIELD_NAME__info[] = "info";
static char controller__srv__LinearWrench_Event__FIELD_NAME__request[] = "request";
static char controller__srv__LinearWrench_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field controller__srv__LinearWrench_Event__FIELDS[] = {
  {
    {controller__srv__LinearWrench_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription controller__srv__LinearWrench_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__LinearWrench_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__LinearWrench_Event__TYPE_NAME, 33, 33},
      {controller__srv__LinearWrench_Event__FIELDS, 3, 3},
    },
    {controller__srv__LinearWrench_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = controller__srv__LinearWrench_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = controller__srv__LinearWrench_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 fx1\n"
  "float32 fy1\n"
  "float32 fz1\n"
  "float32 fx2\n"
  "float32 fy2\n"
  "float32 fz2\n"
  "float32 ramp\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__LinearWrench__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__LinearWrench__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 118, 118},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__LinearWrench_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__LinearWrench_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__LinearWrench_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__LinearWrench_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__LinearWrench_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__LinearWrench_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__LinearWrench__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__LinearWrench__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *controller__srv__LinearWrench_Event__get_individual_type_description_source(NULL);
    sources[3] = *controller__srv__LinearWrench_Request__get_individual_type_description_source(NULL);
    sources[4] = *controller__srv__LinearWrench_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__LinearWrench_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__LinearWrench_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__LinearWrench_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__LinearWrench_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__LinearWrench_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__LinearWrench_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *controller__srv__LinearWrench_Request__get_individual_type_description_source(NULL);
    sources[3] = *controller__srv__LinearWrench_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
