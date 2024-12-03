// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from utmrbc_msgs:msg/Local.idl
// generated code does not contain a copyright notice

#ifndef UTMRBC_MSGS__MSG__DETAIL__LOCAL__STRUCT_H_
#define UTMRBC_MSGS__MSG__DETAIL__LOCAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Local in the package utmrbc_msgs.
typedef struct utmrbc_msgs__msg__Local
{
  float local_x;
  float local_y;
} utmrbc_msgs__msg__Local;

// Struct for a sequence of utmrbc_msgs__msg__Local.
typedef struct utmrbc_msgs__msg__Local__Sequence
{
  utmrbc_msgs__msg__Local * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} utmrbc_msgs__msg__Local__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UTMRBC_MSGS__MSG__DETAIL__LOCAL__STRUCT_H_
