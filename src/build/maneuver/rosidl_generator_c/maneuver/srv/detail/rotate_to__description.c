// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/RotateTo.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/rotate_to__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__RotateTo__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x72, 0x21, 0x27, 0x4a, 0xd7, 0x4c, 0xef, 0x6a,
      0xba, 0xea, 0x7a, 0xc7, 0x7f, 0x6e, 0xa9, 0xc5,
      0x5e, 0x00, 0x90, 0x7d, 0xd9, 0x8f, 0xb2, 0xae,
      0xe5, 0xf3, 0x8e, 0x0d, 0x79, 0xd7, 0xc6, 0xcd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__RotateTo_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x17, 0x72, 0x45, 0xd8, 0x5b, 0x8c, 0x73, 0xb0,
      0x46, 0xc1, 0x84, 0xda, 0xb3, 0xbf, 0xae, 0xf0,
      0xc0, 0xae, 0x74, 0x10, 0xca, 0xb1, 0x33, 0xda,
      0x36, 0xef, 0x7b, 0xc6, 0x0b, 0x81, 0x04, 0xd8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__RotateTo_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf6, 0xbd, 0xdb, 0x49, 0x9f, 0xa0, 0x4c, 0x19,
      0xef, 0xaf, 0xd7, 0xa2, 0x1f, 0xef, 0xbb, 0xf6,
      0x0c, 0x3b, 0xe4, 0x59, 0x0d, 0xef, 0xf7, 0x2f,
      0x5b, 0x9e, 0x5b, 0xba, 0xfc, 0x1a, 0x79, 0xf0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__RotateTo_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xce, 0x1a, 0xba, 0x76, 0x0d, 0xcc, 0x30, 0xbe,
      0x59, 0xd8, 0xc1, 0xf0, 0x92, 0x8b, 0x29, 0xdd,
      0x37, 0xb5, 0x81, 0x26, 0x4d, 0x3c, 0xd1, 0x28,
      0xa2, 0x72, 0x1c, 0x98, 0xca, 0xdc, 0xf7, 0xa8,
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

static char maneuver__srv__RotateTo__TYPE_NAME[] = "maneuver/srv/RotateTo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__RotateTo_Event__TYPE_NAME[] = "maneuver/srv/RotateTo_Event";
static char maneuver__srv__RotateTo_Request__TYPE_NAME[] = "maneuver/srv/RotateTo_Request";
static char maneuver__srv__RotateTo_Response__TYPE_NAME[] = "maneuver/srv/RotateTo_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__RotateTo__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__RotateTo__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__RotateTo__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__RotateTo__FIELDS[] = {
  {
    {maneuver__srv__RotateTo__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__RotateTo_Event__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__RotateTo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Event__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__RotateTo__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__RotateTo__TYPE_NAME, 21, 21},
      {maneuver__srv__RotateTo__FIELDS, 3, 3},
    },
    {maneuver__srv__RotateTo__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__RotateTo_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__RotateTo_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__RotateTo_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__RotateTo_Request__FIELD_NAME__roll[] = "roll";
static char maneuver__srv__RotateTo_Request__FIELD_NAME__pitch[] = "pitch";
static char maneuver__srv__RotateTo_Request__FIELD_NAME__yaw[] = "yaw";
static char maneuver__srv__RotateTo_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field maneuver__srv__RotateTo_Request__FIELDS[] = {
  {
    {maneuver__srv__RotateTo_Request__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Request__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Request__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Request__FIELD_NAME__duration, 8, 8},
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
maneuver__srv__RotateTo_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
      {maneuver__srv__RotateTo_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__RotateTo_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field maneuver__srv__RotateTo_Response__FIELDS[] = {
  {
    {maneuver__srv__RotateTo_Response__FIELD_NAME__status, 6, 6},
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
maneuver__srv__RotateTo_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
      {maneuver__srv__RotateTo_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__RotateTo_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__RotateTo_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__RotateTo_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__RotateTo_Event__FIELDS[] = {
  {
    {maneuver__srv__RotateTo_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__RotateTo_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__RotateTo_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__RotateTo_Event__TYPE_NAME, 27, 27},
      {maneuver__srv__RotateTo_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__RotateTo_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__RotateTo_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__RotateTo_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
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
maneuver__srv__RotateTo__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__RotateTo__TYPE_NAME, 21, 21},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 72, 72},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__RotateTo_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__RotateTo_Request__TYPE_NAME, 29, 29},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__RotateTo_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__RotateTo_Response__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__RotateTo_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__RotateTo_Event__TYPE_NAME, 27, 27},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__RotateTo__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__RotateTo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__RotateTo_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__RotateTo_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__RotateTo_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__RotateTo_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__RotateTo_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__RotateTo_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__RotateTo_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__RotateTo_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__RotateTo_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__RotateTo_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__RotateTo_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
