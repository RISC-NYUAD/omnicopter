// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/GotoPoint.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/goto_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__GotoPoint__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x21, 0x57, 0x6a, 0x0b, 0xea, 0x72, 0x95, 0x86,
      0xde, 0xb7, 0x3c, 0x61, 0x82, 0xbb, 0x7d, 0x05,
      0xf9, 0x3b, 0x52, 0x92, 0x8b, 0x10, 0x0b, 0xfe,
      0x1b, 0xf0, 0xc8, 0x75, 0xdd, 0x9c, 0x7c, 0xd3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__GotoPoint_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0xe5, 0xbc, 0x59, 0x7b, 0xb0, 0xb1, 0xf3,
      0xa0, 0xac, 0xb8, 0x73, 0xed, 0xba, 0xeb, 0xcc,
      0x8c, 0x95, 0x87, 0xa1, 0xfd, 0xaf, 0x39, 0x34,
      0x80, 0xd9, 0x63, 0x5d, 0xbe, 0xb3, 0xf8, 0x20,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__GotoPoint_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0xe3, 0x2c, 0x94, 0x3d, 0x61, 0x95, 0x71,
      0xde, 0x5c, 0x48, 0xd5, 0xf7, 0x7f, 0xf3, 0x2a,
      0x61, 0xe3, 0x48, 0x43, 0x7e, 0xe9, 0x88, 0xeb,
      0x78, 0x86, 0x27, 0xa5, 0xb8, 0x83, 0x01, 0x92,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__GotoPoint_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf5, 0x30, 0xd4, 0x91, 0x47, 0x35, 0x18, 0x16,
      0x44, 0x2d, 0xf5, 0xb9, 0x5a, 0x54, 0x1a, 0x96,
      0x62, 0x33, 0x95, 0x52, 0xb8, 0xca, 0x0f, 0x07,
      0x15, 0x75, 0x03, 0xdf, 0xc0, 0xf2, 0xcc, 0x50,
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

static char maneuver__srv__GotoPoint__TYPE_NAME[] = "maneuver/srv/GotoPoint";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__GotoPoint_Event__TYPE_NAME[] = "maneuver/srv/GotoPoint_Event";
static char maneuver__srv__GotoPoint_Request__TYPE_NAME[] = "maneuver/srv/GotoPoint_Request";
static char maneuver__srv__GotoPoint_Response__TYPE_NAME[] = "maneuver/srv/GotoPoint_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__GotoPoint__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__GotoPoint__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__GotoPoint__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__GotoPoint__FIELDS[] = {
  {
    {maneuver__srv__GotoPoint__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__GotoPoint_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__GotoPoint__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__GotoPoint__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__GotoPoint__TYPE_NAME, 22, 22},
      {maneuver__srv__GotoPoint__FIELDS, 3, 3},
    },
    {maneuver__srv__GotoPoint__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__GotoPoint_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__GotoPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__GotoPoint_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__GotoPoint_Request__FIELD_NAME__x[] = "x";
static char maneuver__srv__GotoPoint_Request__FIELD_NAME__y[] = "y";
static char maneuver__srv__GotoPoint_Request__FIELD_NAME__z[] = "z";
static char maneuver__srv__GotoPoint_Request__FIELD_NAME__yaw[] = "yaw";
static char maneuver__srv__GotoPoint_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field maneuver__srv__GotoPoint_Request__FIELDS[] = {
  {
    {maneuver__srv__GotoPoint_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__FIELD_NAME__duration, 8, 8},
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
maneuver__srv__GotoPoint_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
      {maneuver__srv__GotoPoint_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__GotoPoint_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field maneuver__srv__GotoPoint_Response__FIELDS[] = {
  {
    {maneuver__srv__GotoPoint_Response__FIELD_NAME__status, 6, 6},
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
maneuver__srv__GotoPoint_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
      {maneuver__srv__GotoPoint_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__GotoPoint_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__GotoPoint_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__GotoPoint_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__GotoPoint_Event__FIELDS[] = {
  {
    {maneuver__srv__GotoPoint_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__GotoPoint_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__GotoPoint_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__GotoPoint_Event__TYPE_NAME, 28, 28},
      {maneuver__srv__GotoPoint_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__GotoPoint_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__GotoPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__GotoPoint_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 yaw\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__GotoPoint__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__GotoPoint__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 75, 75},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__GotoPoint_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__GotoPoint_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__GotoPoint_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__GotoPoint_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__GotoPoint_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__GotoPoint_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__GotoPoint__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__GotoPoint__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__GotoPoint_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__GotoPoint_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__GotoPoint_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__GotoPoint_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__GotoPoint_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__GotoPoint_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__GotoPoint_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__GotoPoint_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__GotoPoint_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__GotoPoint_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__GotoPoint_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
