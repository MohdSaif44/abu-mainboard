// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from utmrbc_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__FUNCTIONS_H_
#define UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "utmrbc_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "utmrbc_msgs/msg/detail/odometry__struct.h"

/// Initialize msg/Odometry message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * utmrbc_msgs__msg__Odometry
 * )) before or use
 * utmrbc_msgs__msg__Odometry__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__init(utmrbc_msgs__msg__Odometry * msg);

/// Finalize msg/Odometry message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
void
utmrbc_msgs__msg__Odometry__fini(utmrbc_msgs__msg__Odometry * msg);

/// Create msg/Odometry message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * utmrbc_msgs__msg__Odometry__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
utmrbc_msgs__msg__Odometry *
utmrbc_msgs__msg__Odometry__create();

/// Destroy msg/Odometry message.
/**
 * It calls
 * utmrbc_msgs__msg__Odometry__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
void
utmrbc_msgs__msg__Odometry__destroy(utmrbc_msgs__msg__Odometry * msg);

/// Check for msg/Odometry message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__are_equal(const utmrbc_msgs__msg__Odometry * lhs, const utmrbc_msgs__msg__Odometry * rhs);

/// Copy a msg/Odometry message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__copy(
  const utmrbc_msgs__msg__Odometry * input,
  utmrbc_msgs__msg__Odometry * output);

/// Initialize array of msg/Odometry messages.
/**
 * It allocates the memory for the number of elements and calls
 * utmrbc_msgs__msg__Odometry__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__Sequence__init(utmrbc_msgs__msg__Odometry__Sequence * array, size_t size);

/// Finalize array of msg/Odometry messages.
/**
 * It calls
 * utmrbc_msgs__msg__Odometry__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
void
utmrbc_msgs__msg__Odometry__Sequence__fini(utmrbc_msgs__msg__Odometry__Sequence * array);

/// Create array of msg/Odometry messages.
/**
 * It allocates the memory for the array and calls
 * utmrbc_msgs__msg__Odometry__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
utmrbc_msgs__msg__Odometry__Sequence *
utmrbc_msgs__msg__Odometry__Sequence__create(size_t size);

/// Destroy array of msg/Odometry messages.
/**
 * It calls
 * utmrbc_msgs__msg__Odometry__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
void
utmrbc_msgs__msg__Odometry__Sequence__destroy(utmrbc_msgs__msg__Odometry__Sequence * array);

/// Check for msg/Odometry message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__Sequence__are_equal(const utmrbc_msgs__msg__Odometry__Sequence * lhs, const utmrbc_msgs__msg__Odometry__Sequence * rhs);

/// Copy an array of msg/Odometry messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_utmrbc_msgs
bool
utmrbc_msgs__msg__Odometry__Sequence__copy(
  const utmrbc_msgs__msg__Odometry__Sequence * input,
  utmrbc_msgs__msg__Odometry__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__FUNCTIONS_H_
