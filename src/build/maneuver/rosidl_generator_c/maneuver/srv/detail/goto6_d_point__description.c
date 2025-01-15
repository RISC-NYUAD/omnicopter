// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/Goto6DPoint.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/goto6_d_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Goto6DPoint__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2c, 0x69, 0x53, 0x52, 0x65, 0x7f, 0x60, 0x87,
      0x5c, 0x2f, 0x0e, 0xc8, 0x9a, 0x93, 0x02, 0x7d,
      0x17, 0x2a, 0xae, 0xb9, 0xf3, 0x15, 0x56, 0x3c,
      0x7a, 0x1b, 0x08, 0xb5, 0xb3, 0x65, 0xba, 0xf7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Goto6DPoint_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x18, 0x9f, 0x96, 0x04, 0x3f, 0x25, 0x43, 0x1d,
      0x7c, 0x69, 0xfb, 0x44, 0xae, 0x26, 0x88, 0x59,
      0xad, 0xaa, 0xa6, 0x9c, 0x2e, 0x1b, 0x10, 0x0d,
      0x94, 0xfc, 0x34, 0xb2, 0x18, 0x10, 0x30, 0xe2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Goto6DPoint_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xa9, 0xc2, 0x56, 0x5c, 0xdd, 0xf8, 0xc0,
      0x52, 0xaf, 0xdd, 0x22, 0x4e, 0x55, 0x88, 0xcd,
      0x4c, 0x76, 0x4e, 0xb2, 0x95, 0xf0, 0x85, 0xa0,
      0xb7, 0x5e, 0xfa, 0x53, 0x54, 0x9a, 0x56, 0xa0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Goto6DPoint_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x20, 0x16, 0xc2, 0x76, 0x5c, 0x00, 0x21, 0x07,
      0x1b, 0xe0, 0xb0, 0x6c, 0xb3, 0xc8, 0xb3, 0x65,
      0x8c, 0x61, 0x3d, 0x56, 0x6c, 0xd6, 0x62, 0x76,
      0x37, 0x9f, 0x61, 0x63, 0xaa, 0xf1, 0xeb, 0xb5,
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

static char maneuver__srv__Goto6DPoint__TYPE_NAME[] = "maneuver/srv/Goto6DPoint";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__Goto6DPoint_Event__TYPE_NAME[] = "maneuver/srv/Goto6DPoint_Event";
static char maneuver__srv__Goto6DPoint_Request__TYPE_NAME[] = "maneuver/srv/Goto6DPoint_Request";
static char maneuver__srv__Goto6DPoint_Response__TYPE_NAME[] = "maneuver/srv/Goto6DPoint_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__Goto6DPoint__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__Goto6DPoint__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__Goto6DPoint__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__Goto6DPoint__FIELDS[] = {
  {
    {maneuver__srv__Goto6DPoint__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Goto6DPoint_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__Goto6DPoint__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__Goto6DPoint__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Goto6DPoint__TYPE_NAME, 24, 24},
      {maneuver__srv__Goto6DPoint__FIELDS, 3, 3},
    },
    {maneuver__srv__Goto6DPoint__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__Goto6DPoint_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__Goto6DPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__Goto6DPoint_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__x[] = "x";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__y[] = "y";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__z[] = "z";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__roll[] = "roll";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__pitch[] = "pitch";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__yaw[] = "yaw";
static char maneuver__srv__Goto6DPoint_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field maneuver__srv__Goto6DPoint_Request__FIELDS[] = {
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__FIELD_NAME__duration, 8, 8},
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
maneuver__srv__Goto6DPoint_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
      {maneuver__srv__Goto6DPoint_Request__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Goto6DPoint_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field maneuver__srv__Goto6DPoint_Response__FIELDS[] = {
  {
    {maneuver__srv__Goto6DPoint_Response__FIELD_NAME__status, 6, 6},
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
maneuver__srv__Goto6DPoint_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
      {maneuver__srv__Goto6DPoint_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Goto6DPoint_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__Goto6DPoint_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__Goto6DPoint_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__Goto6DPoint_Event__FIELDS[] = {
  {
    {maneuver__srv__Goto6DPoint_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__Goto6DPoint_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__Goto6DPoint_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Goto6DPoint_Event__TYPE_NAME, 30, 30},
      {maneuver__srv__Goto6DPoint_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__Goto6DPoint_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__Goto6DPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__Goto6DPoint_Response__get_type_description(NULL)->type_description.fields;
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
  "float32 roll\n"
  "float32 pitch\n"
  "float32 yaw\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Goto6DPoint__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Goto6DPoint__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 102, 102},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Goto6DPoint_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Goto6DPoint_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Goto6DPoint_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Goto6DPoint_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Goto6DPoint_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Goto6DPoint_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Goto6DPoint__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Goto6DPoint__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__Goto6DPoint_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__Goto6DPoint_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__Goto6DPoint_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Goto6DPoint_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Goto6DPoint_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Goto6DPoint_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Goto6DPoint_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Goto6DPoint_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Goto6DPoint_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__Goto6DPoint_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__Goto6DPoint_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
