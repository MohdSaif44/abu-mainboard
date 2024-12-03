// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from utmrbc_msgs:msg/Dwa.idl
// generated code does not contain a copyright notice

#ifndef UTMRBC_MSGS__MSG__DETAIL__DWA__STRUCT_H_
#define UTMRBC_MSGS__MSG__DETAIL__DWA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Dwa in the package utmrbc_msgs.
typedef struct utmrbc_msgs__msg__Dwa
{
  float linearvelocity;
  float angularvelocity;
} utmrbc_msgs__msg__Dwa;

// Struct for a sequence of utmrbc_msgs__msg__Dwa.
typedef struct utmrbc_msgs__msg__Dwa__Sequence
{
  utmrbc_msgs__msg__Dwa * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} utmrbc_msgs__msg__Dwa__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UTMRBC_MSGS__MSG__DETAIL__DWA__STRUCT_H_
