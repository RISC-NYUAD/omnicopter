// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

#include "controller/msg/detail/error_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__msg__ErrorMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x18, 0x9b, 0x95, 0x35, 0x62, 0xc7, 0xe8, 0x22,
      0x4e, 0x16, 0x06, 0x26, 0x2e, 0xdd, 0x4b, 0x2d,
      0x7c, 0x16, 0x46, 0xb8, 0xfe, 0x15, 0xc6, 0xeb,
      0xf3, 0x80, 0x01, 0x2d, 0x9c, 0x64, 0xaa, 0x34,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char controller__msg__ErrorMsg__TYPE_NAME[] = "controller/msg/ErrorMsg";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char controller__msg__ErrorMsg__FIELD_NAME__ex[] = "ex";
static char controller__msg__ErrorMsg__FIELD_NAME__ev[] = "ev";
static char controller__msg__ErrorMsg__FIELD_NAME__ea[] = "ea";
static char controller__msg__ErrorMsg__FIELD_NAME__er[] = "er";
static char controller__msg__ErrorMsg__FIELD_NAME__ew[] = "ew";
static char controller__msg__ErrorMsg__FIELD_NAME__iex[] = "iex";
static char controller__msg__ErrorMsg__FIELD_NAME__ier[] = "ier";
static char controller__msg__ErrorMsg__FIELD_NAME__accd[] = "accd";
static char controller__msg__ErrorMsg__FIELD_NAME__wrench[] = "wrench";
static char controller__msg__ErrorMsg__FIELD_NAME__exwrench[] = "exwrench";
static char controller__msg__ErrorMsg__FIELD_NAME__prop[] = "prop";
static char controller__msg__ErrorMsg__FIELD_NAME__weight[] = "weight";

static rosidl_runtime_c__type_description__Field controller__msg__ErrorMsg__FIELDS[] = {
  {
    {controller__msg__ErrorMsg__FIELD_NAME__ex, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__ev, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__ea, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__er, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__ew, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__iex, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__ier, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__accd, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__wrench, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__exwrench, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__prop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__ErrorMsg__FIELD_NAME__weight, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription controller__msg__ErrorMsg__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__msg__ErrorMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__msg__ErrorMsg__TYPE_NAME, 23, 23},
      {controller__msg__ErrorMsg__FIELDS, 12, 12},
    },
    {controller__msg__ErrorMsg__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "geometry_msgs/Vector3 ex\n"
  "geometry_msgs/Vector3 ev\n"
  "geometry_msgs/Vector3 ea\n"
  "geometry_msgs/Vector3 er\n"
  "geometry_msgs/Vector3 ew\n"
  "geometry_msgs/Vector3 iex\n"
  "geometry_msgs/Vector3 ier\n"
  "geometry_msgs/Vector3 accd\n"
  "float32[] wrench\n"
  "float32[] exwrench\n"
  "float32[] prop\n"
  "float32 weight";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__msg__ErrorMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__msg__ErrorMsg__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 270, 270},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__msg__ErrorMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__msg__ErrorMsg__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
