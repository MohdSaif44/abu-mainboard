// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from utmrbc_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_
#define UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Odometry in the package utmrbc_msgs.
typedef struct utmrbc_msgs__msg__Odometry
{
  float odom_pose_x;
  float odom_pose_y;
  float imu_angular_vx;
  float imu_angular_vy;
  float imu_angular_vz;
  float imu_linear_ax;
  float imu_linear_ay;
  float imu_orientation_x;
  float imu_orientation_y;
  float imu_orientation_z;
  float imu_orientation_w;
  float imu_yaw;
} utmrbc_msgs__msg__Odometry;

// Struct for a sequence of utmrbc_msgs__msg__Odometry.
typedef struct utmrbc_msgs__msg__Odometry__Sequence
{
  utmrbc_msgs__msg__Odometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} utmrbc_msgs__msg__Odometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UTMRBC_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_
