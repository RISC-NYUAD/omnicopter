// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/Ellipse5D.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/ellipse5_d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Ellipse5D__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0x51, 0x86, 0xf2, 0x5d, 0x3e, 0xac, 0x23,
      0x3d, 0x39, 0xba, 0x80, 0xaa, 0x62, 0xaa, 0x88,
      0xf9, 0xfa, 0xc4, 0x48, 0x1c, 0x11, 0x97, 0xee,
      0x73, 0xdc, 0xbe, 0x6b, 0xfb, 0x30, 0x5b, 0xf9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Ellipse5D_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x47, 0x3e, 0xe0, 0x55, 0x80, 0x61, 0x64, 0x61,
      0xac, 0xec, 0xa2, 0x55, 0xf8, 0x27, 0x27, 0x64,
      0x34, 0x1e, 0x58, 0x00, 0xfe, 0xa8, 0xef, 0x05,
      0x9a, 0x40, 0x5d, 0xf1, 0xe0, 0x6e, 0xfc, 0xf9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Ellipse5D_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfb, 0x17, 0xd9, 0x32, 0xf5, 0x89, 0x29, 0xda,
      0xa8, 0x9e, 0x1e, 0x58, 0x1c, 0x16, 0x7f, 0x54,
      0x00, 0x20, 0x76, 0x79, 0x6d, 0x57, 0xb3, 0x42,
      0x15, 0xc7, 0xfb, 0x28, 0xb6, 0xa4, 0xc9, 0xbb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__Ellipse5D_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x15, 0xc0, 0xed, 0xaf, 0x02, 0x7d, 0x1d, 0x04,
      0xe8, 0x95, 0x31, 0x53, 0xe8, 0x89, 0x83, 0x8c,
      0x33, 0xf5, 0xd8, 0x98, 0x52, 0x6e, 0x84, 0x78,
      0x73, 0xbb, 0x48, 0xe8, 0xa3, 0xcc, 0x7f, 0x32,
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

static char maneuver__srv__Ellipse5D__TYPE_NAME[] = "maneuver/srv/Ellipse5D";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__Ellipse5D_Event__TYPE_NAME[] = "maneuver/srv/Ellipse5D_Event";
static char maneuver__srv__Ellipse5D_Request__TYPE_NAME[] = "maneuver/srv/Ellipse5D_Request";
static char maneuver__srv__Ellipse5D_Response__TYPE_NAME[] = "maneuver/srv/Ellipse5D_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__Ellipse5D__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__Ellipse5D__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__Ellipse5D__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__Ellipse5D__FIELDS[] = {
  {
    {maneuver__srv__Ellipse5D__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__Ellipse5D_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__Ellipse5D__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__Ellipse5D__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Ellipse5D__TYPE_NAME, 22, 22},
      {maneuver__srv__Ellipse5D__FIELDS, 3, 3},
    },
    {maneuver__srv__Ellipse5D__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__Ellipse5D_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__Ellipse5D_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__Ellipse5D_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__x_min[] = "x_min";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__x_max[] = "x_max";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__y_min[] = "y_min";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__y_max[] = "y_max";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__z_min[] = "z_min";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__z_max[] = "z_max";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_start[] = "roll_start";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_mid[] = "roll_mid";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_end[] = "roll_end";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_start[] = "pitch_start";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_mid[] = "pitch_mid";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_end[] = "pitch_end";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__yaw[] = "yaw";
static char maneuver__srv__Ellipse5D_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field maneuver__srv__Ellipse5D_Request__FIELDS[] = {
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__x_min, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__x_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__y_min, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__y_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__z_min, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__z_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_start, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_mid, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__roll_end, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_start, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_mid, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__pitch_end, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__FIELD_NAME__duration, 8, 8},
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
maneuver__srv__Ellipse5D_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
      {maneuver__srv__Ellipse5D_Request__FIELDS, 14, 14},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Ellipse5D_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field maneuver__srv__Ellipse5D_Response__FIELDS[] = {
  {
    {maneuver__srv__Ellipse5D_Response__FIELD_NAME__status, 6, 6},
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
maneuver__srv__Ellipse5D_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
      {maneuver__srv__Ellipse5D_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__Ellipse5D_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__Ellipse5D_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__Ellipse5D_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__Ellipse5D_Event__FIELDS[] = {
  {
    {maneuver__srv__Ellipse5D_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__Ellipse5D_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__Ellipse5D_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__Ellipse5D_Event__TYPE_NAME, 28, 28},
      {maneuver__srv__Ellipse5D_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__Ellipse5D_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__Ellipse5D_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__Ellipse5D_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x_min\n"
  "float32 x_max\n"
  "float32 y_min\n"
  "float32 y_max\n"
  "float32 z_min\n"
  "float32 z_max\n"
  "float32 roll_start\n"
  "float32 roll_mid\n"
  "float32 roll_end\n"
  "float32 pitch_start\n"
  "float32 pitch_mid\n"
  "float32 pitch_end\n"
  "float32 yaw\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Ellipse5D__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Ellipse5D__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 238, 238},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Ellipse5D_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Ellipse5D_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Ellipse5D_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Ellipse5D_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__Ellipse5D_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__Ellipse5D_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Ellipse5D__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Ellipse5D__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__Ellipse5D_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__Ellipse5D_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__Ellipse5D_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Ellipse5D_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Ellipse5D_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Ellipse5D_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Ellipse5D_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__Ellipse5D_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__Ellipse5D_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__Ellipse5D_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__Ellipse5D_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
