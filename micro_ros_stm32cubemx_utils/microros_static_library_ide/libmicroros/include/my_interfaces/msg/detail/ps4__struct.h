// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_interfaces:msg/Ps4.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACES__MSG__DETAIL__PS4__STRUCT_H_
#define MY_INTERFACES__MSG__DETAIL__PS4__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'axes'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Ps4 in the package my_interfaces.
typedef struct my_interfaces__msg__Ps4
{
  rosidl_runtime_c__uint8__Sequence axes;
  uint8_t axes1;
  uint8_t axes2;
  uint8_t axes3;
  uint8_t axes4;
  uint8_t axes5;
  uint8_t axes6;
  uint8_t axes7;
  uint8_t axes8;
  int16_t buttons;
} my_interfaces__msg__Ps4;

// Struct for a sequence of my_interfaces__msg__Ps4.
typedef struct my_interfaces__msg__Ps4__Sequence
{
  my_interfaces__msg__Ps4 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_interfaces__msg__Ps4__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_INTERFACES__MSG__DETAIL__PS4__STRUCT_H_
