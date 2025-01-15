// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

#include "controller/msg/detail/uvector__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_controller
const rosidl_type_hash_t *
controller__msg__Uvector__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2a, 0x69, 0xf6, 0xd2, 0x23, 0xad, 0x39, 0x2a,
      0x92, 0x06, 0x0b, 0xb4, 0xdb, 0x94, 0x5f, 0x0d,
      0xda, 0xa2, 0xef, 0xec, 0x6b, 0xbf, 0x38, 0x88,
      0x8c, 0x0b, 0x2c, 0xe7, 0x03, 0xc3, 0x23, 0x22,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char controller__msg__Uvector__TYPE_NAME[] = "controller/msg/Uvector";

// Define type names, field names, and default values
static char controller__msg__Uvector__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field controller__msg__Uvector__FIELDS[] = {
  {
    {controller__msg__Uvector__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
controller__msg__Uvector__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {controller__msg__Uvector__TYPE_NAME, 22, 22},
      {controller__msg__Uvector__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32[12] value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
controller__msg__Uvector__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {controller__msg__Uvector__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 18, 18},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
controller__msg__Uvector__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *controller__msg__Uvector__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
