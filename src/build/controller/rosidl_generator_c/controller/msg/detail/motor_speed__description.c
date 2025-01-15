// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

#include "controller/msg/detail/motor_speed__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__msg__MotorSpeed__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd1, 0x30, 0x76, 0x5a, 0x7d, 0xe0, 0x30, 0x21,
      0x6b, 0x2e, 0xb3, 0x8f, 0x5f, 0x23, 0x33, 0xe7,
      0xe2, 0x04, 0x50, 0x55, 0xf7, 0x0c, 0x25, 0xea,
      0x6a, 0x9c, 0xf9, 0x5f, 0x13, 0xc0, 0x2e, 0xff,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char controller__msg__MotorSpeed__TYPE_NAME[] = "controller/msg/MotorSpeed";

// Define type names, field names, and default values
static char controller__msg__MotorSpeed__FIELD_NAME__name[] = "name";
static char controller__msg__MotorSpeed__FIELD_NAME__velocity[] = "velocity";

static rosidl_runtime_c__type_description__Field controller__msg__MotorSpeed__FIELDS[] = {
  {
    {controller__msg__MotorSpeed__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {controller__msg__MotorSpeed__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__msg__MotorSpeed__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__msg__MotorSpeed__TYPE_NAME, 25, 25},
      {controller__msg__MotorSpeed__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string[] name\n"
  "float32[] velocity";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__msg__MotorSpeed__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__msg__MotorSpeed__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 33, 33},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__msg__MotorSpeed__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__msg__MotorSpeed__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
