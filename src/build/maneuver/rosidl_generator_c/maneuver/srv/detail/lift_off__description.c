// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/LiftOff.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/lift_off__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__LiftOff__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xea, 0xee, 0xa4, 0x13, 0xbc, 0xf7, 0x11, 0xcd,
      0xb3, 0xa4, 0xb8, 0x56, 0x63, 0x88, 0x60, 0xbf,
      0x21, 0xf1, 0xc4, 0x0a, 0xe1, 0xfa, 0xc6, 0x8e,
      0x15, 0xf3, 0xbd, 0xbd, 0x77, 0x3d, 0xb4, 0x82,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__LiftOff_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcf, 0xdb, 0x4d, 0x8a, 0xcb, 0xc7, 0x2d, 0x95,
      0x08, 0x70, 0x5b, 0x01, 0xe6, 0x75, 0x77, 0xaf,
      0xd9, 0xc7, 0xc1, 0x3a, 0x60, 0x85, 0x28, 0x65,
      0xce, 0xdc, 0xe4, 0x17, 0x45, 0xce, 0xa1, 0xb0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__LiftOff_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbc, 0x43, 0x69, 0x65, 0xb5, 0xc9, 0x95, 0xa8,
      0xc4, 0xc1, 0x1f, 0x2f, 0x86, 0x9d, 0xfd, 0x44,
      0xbb, 0x40, 0x4b, 0xce, 0xb6, 0x07, 0xe6, 0x1d,
      0x76, 0xf7, 0xa1, 0xb1, 0x47, 0x22, 0xa4, 0x37,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__LiftOff_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x6f, 0xf0, 0x02, 0x33, 0x89, 0xd2, 0xd0,
      0x51, 0xd6, 0x8e, 0xae, 0xd2, 0x00, 0xd6, 0xa4,
      0x04, 0x16, 0xed, 0xa1, 0xb3, 0x6c, 0x70, 0xef,
      0x1e, 0x21, 0xc1, 0xab, 0xc7, 0xae, 0xdc, 0xe8,
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

static char maneuver__srv__LiftOff__TYPE_NAME[] = "maneuver/srv/LiftOff";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__LiftOff_Event__TYPE_NAME[] = "maneuver/srv/LiftOff_Event";
static char maneuver__srv__LiftOff_Request__TYPE_NAME[] = "maneuver/srv/LiftOff_Request";
static char maneuver__srv__LiftOff_Response__TYPE_NAME[] = "maneuver/srv/LiftOff_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__LiftOff__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__LiftOff__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__LiftOff__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__LiftOff__FIELDS[] = {
  {
    {maneuver__srv__LiftOff__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__LiftOff_Event__TYPE_NAME, 26, 26},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__LiftOff__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Event__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__LiftOff__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__LiftOff__TYPE_NAME, 20, 20},
      {maneuver__srv__LiftOff__FIELDS, 3, 3},
    },
    {maneuver__srv__LiftOff__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__LiftOff_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__LiftOff_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__LiftOff_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__LiftOff_Request__FIELD_NAME__height[] = "height";
static char maneuver__srv__LiftOff_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field maneuver__srv__LiftOff_Request__FIELDS[] = {
  {
    {maneuver__srv__LiftOff_Request__FIELD_NAME__height, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Request__FIELD_NAME__duration, 8, 8},
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
maneuver__srv__LiftOff_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
      {maneuver__srv__LiftOff_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__LiftOff_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field maneuver__srv__LiftOff_Response__FIELDS[] = {
  {
    {maneuver__srv__LiftOff_Response__FIELD_NAME__status, 6, 6},
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
maneuver__srv__LiftOff_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
      {maneuver__srv__LiftOff_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__LiftOff_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__LiftOff_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__LiftOff_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__LiftOff_Event__FIELDS[] = {
  {
    {maneuver__srv__LiftOff_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__LiftOff_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__LiftOff_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__LiftOff_Event__TYPE_NAME, 26, 26},
      {maneuver__srv__LiftOff_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__LiftOff_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__LiftOff_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__LiftOff_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 height\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__LiftOff__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__LiftOff__TYPE_NAME, 20, 20},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 48, 48},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__LiftOff_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__LiftOff_Request__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__LiftOff_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__LiftOff_Response__TYPE_NAME, 29, 29},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__LiftOff_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__LiftOff_Event__TYPE_NAME, 26, 26},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__LiftOff__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__LiftOff__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__LiftOff_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__LiftOff_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__LiftOff_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__LiftOff_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__LiftOff_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__LiftOff_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__LiftOff_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__LiftOff_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__LiftOff_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__LiftOff_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__LiftOff_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
