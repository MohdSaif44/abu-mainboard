#include "microros.h"

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

rcl_ret_t error;

rcl_ret_t microros_init(void* custom_transport_handler, const char * node_name, const size_t executor_number_of_handles){

	rmw_uros_set_custom_transport(
			true, (void*) &custom_transport_handler,
			cubemx_transport_open,
			cubemx_transport_close,
			cubemx_transport_write,
			cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	rcutils_set_default_allocator(&freeRTOS_allocator);

	allocator = rcl_get_default_allocator();


	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);
	rcl_init_options_set_domain_id(&init_options, 7);
	error = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);


//	error =  rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, node_name, "", &support);

	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, executor_number_of_handles, &allocator);

	return error;

}
