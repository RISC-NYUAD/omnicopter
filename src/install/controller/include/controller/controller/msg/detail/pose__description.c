// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:msg/Pose.idl
// generated code does not contain a copyright notice

#include "controller/msg/detail/pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__msg__Pose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x46, 0x47, 0xf9, 0x17, 0x22, 0xeb, 0x15, 0x90,
      0xad, 0xc6, 0xb2, 0xc1, 0x5f, 0x0b, 0x69, 0xef,
      0xa1, 0xb9, 0x19, 0xa4, 0x30, 0x74, 0xfe, 0x0d,
      0xb9, 0x5a, 0xdb, 0xbb, 0x77, 0x37, 0x93, 0x35,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char controller__msg__Pose__TYPE_NAME[] = "controller/msg/Pose";

// Define type names, field names, and default values
static char controller__msg__Pose__FIELD_NAME__x[] = "x";
static char controller__msg__Pose__FIELD_NAME__y[] = "y";
static char controller__msg__Pose__FIELD_NAME__z[] = "z";
static char controller__msg__Pose__FIELD_NAME__roll[] = "roll";
static char controller__msg__Pose__FIELD_NAME__pitch[] = "pitch";
static char controller__msg__Pose__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field controller__msg__Pose__FIELDS[] = {
  {
    {controller__msg__Pose__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__Pose__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__Pose__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__Pose__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__Pose__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__Pose__FIELD_NAME__yaw, 3, 3},
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
controller__msg__Pose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__msg__Pose__TYPE_NAME, 19, 19},
      {controller__msg__Pose__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
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
  "float32 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__msg__Pose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__msg__Pose__TYPE_NAME, 19, 19},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 69, 69},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__msg__Pose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__msg__Pose__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
