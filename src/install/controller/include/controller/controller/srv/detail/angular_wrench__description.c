// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:srv/AngularWrench.idl
// generated code does not contain a copyright notice

#include "controller/srv/detail/angular_wrench__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__AngularWrench__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1a, 0x89, 0x14, 0x27, 0x62, 0x9d, 0x32, 0x71,
      0x72, 0xf2, 0x48, 0x20, 0xac, 0x43, 0xa1, 0x30,
      0xa3, 0x37, 0xe2, 0x4d, 0x9c, 0x35, 0x7e, 0xc6,
      0x9e, 0x2e, 0x40, 0x3f, 0x5a, 0x38, 0xfa, 0x24,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__AngularWrench_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0x27, 0xda, 0x3d, 0x89, 0x71, 0x62, 0xed,
      0xb3, 0xbb, 0x69, 0x0e, 0x55, 0x3c, 0x11, 0x63,
      0x9f, 0x67, 0x5f, 0x2d, 0x23, 0xd4, 0x68, 0x00,
      0x3a, 0x11, 0x9f, 0x45, 0x58, 0x38, 0x57, 0x0f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__AngularWrench_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xed, 0xae, 0x56, 0x5c, 0xe4, 0x33, 0x7c, 0xf5,
      0xbb, 0x56, 0x50, 0x0d, 0x34, 0x62, 0x32, 0xc8,
      0x8b, 0xba, 0x72, 0x0e, 0x01, 0x8d, 0x98, 0x2c,
      0x4a, 0x11, 0x5e, 0x74, 0x5f, 0x6b, 0x7b, 0x04,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__srv__AngularWrench_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0xf5, 0x58, 0xad, 0x18, 0xc1, 0x9c, 0xf9,
      0xe3, 0xdb, 0xc7, 0xff, 0xd7, 0x5a, 0x23, 0x0e,
      0x3b, 0x61, 0xb1, 0xda, 0x56, 0xe5, 0xf4, 0x49,
      0xe4, 0xc2, 0x81, 0xb3, 0x1b, 0x2a, 0x8b, 0x91,
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

static char controller__srv__AngularWrench__TYPE_NAME[] = "controller/srv/AngularWrench";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char controller__srv__AngularWrench_Event__TYPE_NAME[] = "controller/srv/AngularWrench_Event";
static char controller__srv__AngularWrench_Request__TYPE_NAME[] = "controller/srv/AngularWrench_Request";
static char controller__srv__AngularWrench_Response__TYPE_NAME[] = "controller/srv/AngularWrench_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char controller__srv__AngularWrench__FIELD_NAME__request_message[] = "request_message";
static char controller__srv__AngularWrench__FIELD_NAME__response_message[] = "response_message";
static char controller__srv__AngularWrench__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field controller__srv__AngularWrench__FIELDS[] = {
  {
    {controller__srv__AngularWrench__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {controller__srv__AngularWrench_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription controller__srv__AngularWrench__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__AngularWrench__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__AngularWrench__TYPE_NAME, 28, 28},
      {controller__srv__AngularWrench__FIELDS, 3, 3},
    },
    {controller__srv__AngularWrench__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = controller__srv__AngularWrench_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = controller__srv__AngularWrench_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = controller__srv__AngularWrench_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__AngularWrench_Request__FIELD_NAME__fz[] = "fz";
static char controller__srv__AngularWrench_Request__FIELD_NAME__phi1[] = "phi1";
static char controller__srv__AngularWrench_Request__FIELD_NAME__phi2[] = "phi2";
static char controller__srv__AngularWrench_Request__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field controller__srv__AngularWrench_Request__FIELDS[] = {
  {
    {controller__srv__AngularWrench_Request__FIELD_NAME__fz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Request__FIELD_NAME__phi1, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Request__FIELD_NAME__phi2, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Request__FIELD_NAME__duration, 8, 8},
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
controller__srv__AngularWrench_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
      {controller__srv__AngularWrench_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__AngularWrench_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field controller__srv__AngularWrench_Response__FIELDS[] = {
  {
    {controller__srv__AngularWrench_Response__FIELD_NAME__status, 6, 6},
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
controller__srv__AngularWrench_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
      {controller__srv__AngularWrench_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char controller__srv__AngularWrench_Event__FIELD_NAME__info[] = "info";
static char controller__srv__AngularWrench_Event__FIELD_NAME__request[] = "request";
static char controller__srv__AngularWrench_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field controller__srv__AngularWrench_Event__FIELDS[] = {
  {
    {controller__srv__AngularWrench_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription controller__srv__AngularWrench_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__srv__AngularWrench_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__srv__AngularWrench_Event__TYPE_NAME, 34, 34},
      {controller__srv__AngularWrench_Event__FIELDS, 3, 3},
    },
    {controller__srv__AngularWrench_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = controller__srv__AngularWrench_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = controller__srv__AngularWrench_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 fz\n"
  "float32 phi1\n"
  "float32 phi2\n"
  "float32 duration\n"
  "---\n"
  "bool status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__AngularWrench__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__AngularWrench__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 70, 70},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__AngularWrench_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__AngularWrench_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__AngularWrench_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__AngularWrench_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
controller__srv__AngularWrench_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__srv__AngularWrench_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__AngularWrench__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__AngularWrench__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *controller__srv__AngularWrench_Event__get_individual_type_description_source(NULL);
    sources[3] = *controller__srv__AngularWrench_Request__get_individual_type_description_source(NULL);
    sources[4] = *controller__srv__AngularWrench_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__AngularWrench_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__AngularWrench_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__AngularWrench_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__AngularWrench_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__srv__AngularWrench_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__srv__AngularWrench_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *controller__srv__AngularWrench_Request__get_individual_type_description_source(NULL);
    sources[3] = *controller__srv__AngularWrench_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
