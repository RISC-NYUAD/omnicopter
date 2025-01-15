// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from maneuver:srv/ArmDisarm.idl
// generated code does not contain a copyright notice

#include "maneuver/srv/detail/arm_disarm__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__ArmDisarm__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x01, 0x7b, 0x3f, 0xc0, 0x58, 0xd3, 0xad, 0x22,
      0x59, 0x2b, 0x44, 0x3f, 0x49, 0xa9, 0x8c, 0x9c,
      0x95, 0x20, 0xd6, 0x00, 0xb9, 0x36, 0x0c, 0x7e,
      0xb4, 0x42, 0xf7, 0xd4, 0xa7, 0xff, 0xf1, 0xd5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__ArmDisarm_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x66, 0x83, 0xd3, 0x61, 0x00, 0x08, 0x9d, 0x55,
      0x36, 0xe7, 0xb7, 0x0b, 0xa3, 0x5c, 0x00, 0x9d,
      0x3d, 0xd9, 0xee, 0x2f, 0x21, 0x0e, 0x3e, 0x1f,
      0x28, 0x43, 0x4d, 0xec, 0xd7, 0xfd, 0x62, 0x1a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__ArmDisarm_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0x10, 0x00, 0x51, 0xb4, 0x34, 0x60, 0x90,
      0xac, 0x8e, 0x64, 0x31, 0x34, 0x28, 0x24, 0xc9,
      0x63, 0x6f, 0x0c, 0x82, 0x7c, 0x55, 0xa5, 0x05,
      0x2f, 0x2b, 0x76, 0xd2, 0x20, 0x05, 0x58, 0xb0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_maneuver
const rosidl_type_hash_t *
maneuver__srv__ArmDisarm_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x62, 0x4f, 0xc1, 0x75, 0x19, 0xa9, 0xd4, 0xa6,
      0x52, 0x87, 0x8f, 0x26, 0xed, 0xbf, 0xef, 0xd0,
      0x4f, 0x98, 0x03, 0xdd, 0xb9, 0x94, 0xb7, 0x85,
      0x9b, 0xb4, 0xd7, 0x5d, 0xc8, 0xd4, 0xdd, 0x40,
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

static char maneuver__srv__ArmDisarm__TYPE_NAME[] = "maneuver/srv/ArmDisarm";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char maneuver__srv__ArmDisarm_Event__TYPE_NAME[] = "maneuver/srv/ArmDisarm_Event";
static char maneuver__srv__ArmDisarm_Request__TYPE_NAME[] = "maneuver/srv/ArmDisarm_Request";
static char maneuver__srv__ArmDisarm_Response__TYPE_NAME[] = "maneuver/srv/ArmDisarm_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char maneuver__srv__ArmDisarm__FIELD_NAME__request_message[] = "request_message";
static char maneuver__srv__ArmDisarm__FIELD_NAME__response_message[] = "response_message";
static char maneuver__srv__ArmDisarm__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field maneuver__srv__ArmDisarm__FIELDS[] = {
  {
    {maneuver__srv__ArmDisarm__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {maneuver__srv__ArmDisarm_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__ArmDisarm__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__ArmDisarm__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__ArmDisarm__TYPE_NAME, 22, 22},
      {maneuver__srv__ArmDisarm__FIELDS, 3, 3},
    },
    {maneuver__srv__ArmDisarm__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__ArmDisarm_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__ArmDisarm_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = maneuver__srv__ArmDisarm_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__ArmDisarm_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field maneuver__srv__ArmDisarm_Request__FIELDS[] = {
  {
    {maneuver__srv__ArmDisarm_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__ArmDisarm_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
      {maneuver__srv__ArmDisarm_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__ArmDisarm_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field maneuver__srv__ArmDisarm_Response__FIELDS[] = {
  {
    {maneuver__srv__ArmDisarm_Response__FIELD_NAME__success, 7, 7},
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
maneuver__srv__ArmDisarm_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
      {maneuver__srv__ArmDisarm_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char maneuver__srv__ArmDisarm_Event__FIELD_NAME__info[] = "info";
static char maneuver__srv__ArmDisarm_Event__FIELD_NAME__request[] = "request";
static char maneuver__srv__ArmDisarm_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field maneuver__srv__ArmDisarm_Event__FIELDS[] = {
  {
    {maneuver__srv__ArmDisarm_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription maneuver__srv__ArmDisarm_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
maneuver__srv__ArmDisarm_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {maneuver__srv__ArmDisarm_Event__TYPE_NAME, 28, 28},
      {maneuver__srv__ArmDisarm_Event__FIELDS, 3, 3},
    },
    {maneuver__srv__ArmDisarm_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = maneuver__srv__ArmDisarm_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = maneuver__srv__ArmDisarm_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__ArmDisarm__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__ArmDisarm__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 16, 16},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__ArmDisarm_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__ArmDisarm_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__ArmDisarm_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__ArmDisarm_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
maneuver__srv__ArmDisarm_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {maneuver__srv__ArmDisarm_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__ArmDisarm__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__ArmDisarm__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__ArmDisarm_Event__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__ArmDisarm_Request__get_individual_type_description_source(NULL);
    sources[4] = *maneuver__srv__ArmDisarm_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__ArmDisarm_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__ArmDisarm_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__ArmDisarm_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__ArmDisarm_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
maneuver__srv__ArmDisarm_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *maneuver__srv__ArmDisarm_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *maneuver__srv__ArmDisarm_Request__get_individual_type_description_source(NULL);
    sources[3] = *maneuver__srv__ArmDisarm_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
