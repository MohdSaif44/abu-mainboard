
#ifndef __MICROROS_H__
#define __MICROROS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

// Messages include example
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int16.h>
#include <utmrbc_msgs/msg/odometry.h>
#include <utmrbc_msgs/msg/local.h>
#include <std_msgs/msg/float32.h>


/* Declerations ------------------------------------------------------------------*/
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void microros_init(void* custom_transport_handler, const char * node_name, const size_t executor_number_of_handles);

/* Variables ------------------------------------------------------------------*/
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node, node_1, node_2;
rclc_executor_t executor, executor2;

/* Default examples ------------------------------------------------------------------*/
rcl_publisher_t publisher;
rcl_subscription_t subscriber, subscriber2;
//rcl_service_t service;
//rcl_client_t client;
utmrbc_msgs__msg__Odometry ekf_msg;
utmrbc_msgs__msg__Local local_msg;
std_msgs__msg__Float32 Float32;
//nav_msgs__msg__Odometry odom_filtered_msg;
//example_interfaces__srv__AddTwoInts_Request request_msg;
//example_interfaces__srv__AddTwoInts_Request__Sequence sequence_msg;
//example_interfaces__srv__AddTwoInts_Response response_msg;


#ifdef __cplusplus
}
#endif

#endif // __MICROROS_H__
