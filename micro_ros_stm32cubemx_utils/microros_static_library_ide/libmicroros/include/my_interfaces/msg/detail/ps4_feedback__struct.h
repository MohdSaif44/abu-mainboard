// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_interfaces:msg/Ps4Feedback.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACES__MSG__DETAIL__PS4_FEEDBACK__STRUCT_H_
#define MY_INTERFACES__MSG__DETAIL__PS4_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_LED'.
enum
{
  my_interfaces__msg__Ps4Feedback__TYPE_LED = 0
};

/// Constant 'TYPE_RUMBLE'.
enum
{
  my_interfaces__msg__Ps4Feedback__TYPE_RUMBLE = 1
};

/// Constant 'TYPE_BUZZER'.
enum
{
  my_interfaces__msg__Ps4Feedback__TYPE_BUZZER = 2
};

/// Struct defined in msg/Ps4Feedback in the package my_interfaces.
/**
  * Declare of the type of feedback
 */
typedef struct my_interfaces__msg__Ps4Feedback
{
  uint8_t type;
  /// This will hold an id number for each type of each feedback.
  /// Example, the first led would be id=0, the second would be id=1
  uint8_t id;
  /// Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
  /// actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
  float intensity;
} my_interfaces__msg__Ps4Feedback;

// Struct for a sequence of my_interfaces__msg__Ps4Feedback.
typedef struct my_interfaces__msg__Ps4Feedback__Sequence
{
  my_interfaces__msg__Ps4Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_interfaces__msg__Ps4Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_INTERFACES__MSG__DETAIL__PS4_FEEDBACK__STRUCT_H_
