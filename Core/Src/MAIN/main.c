/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {

	set();

	const osThreadAttr_t MicrorosTask_attributes =
	{ .name = "MicrorosTask", .stack_size = 3000 * 4,
			.priority = (osPriority_t) osPriorityHigh, };

	const osThreadAttr_t MainTask_attributes =
	{ .name = "MainTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SecondaryTask_attributes =
	{ .name = "SecondTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t CalculationTask_attributes =
	{ .name = "CalculationTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osSemaphoreAttr_t CalcSemaphore_attributes = { .name = "CalcSemaphore" };
	const osSemaphoreAttr_t MainSemaphore_attributes = { .name = "MainSemaphore" };


	osKernelInitialize();

	MainTaskHandle		  = osThreadNew(MainTask, NULL, &MainTask_attributes);
	MicrorosTaskHandle    = osThreadNew(Microros, NULL, &MicrorosTask_attributes);
	SecondaryTaskHandle   = osThreadNew(SecondaryTask, NULL, &SecondaryTask_attributes);
	CalculationTaskHandle = osThreadNew(Calculation, NULL, &CalculationTask_attributes);
	CalcSemaphore 		  = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);
	MainSemaphore 		  = osSemaphoreNew(1, 0, &MainSemaphore_attributes);

	osKernelStart();
}

void TIM6_DAC_IRQHandler(void) { 	// 10us

	//	SoftPWMUpdate();
	HAL_TIM_IRQHandler(&htim6);
}

void TIM7_IRQHandler(void) { 		// 1ms

	static uint8_t led = 0;
	static uint8_t calc_count = 0;

	osSemaphoreRelease(MainSemaphore);

	if (++calc_count > 7){
		osSemaphoreRelease(CalcSemaphore);
	}

	if (++led > 19){
		led1 = !led1;
		led = 0;
		PSxConnectionHandler(&ps4);
	}

	HAL_TIM_IRQHandler(&htim7);
}


void MainTask(void *argument) {

	while (1) {

		osSemaphoreAcquire(MainSemaphore, osWaitForever);

		static uint8_t led = 0;
		if (++led >= 255) {
			led2 = !led2;
			led = 0;
		}
	}
}


void SecondaryTask(void *argument) {

	while (1) {

	}
}


void Calculation(void *argument) { 	// 7ms

	sys.manual  = 0;
	sys.control = 1;

	while (1) {

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);

		if(ps4.button == CROSS){

			sys.manual = 1;

		}

		if(sys.manual){

			SwerveAlign(158.8, 7.1, 18.9, 17.2, hall_sensor1_pin, hall_sensor2_pin, hall_sensor3_pin, hall_sensor4_pin);
			SwerveRun(0.1, perspective, 20.0, 5, 0);

			if(swerve.aligned == 1){

				if(fabs(ps4.joyL_x)+fabs(ps4.joyL_y)+fabs(ps4.joyR_2 - ps4.joyL_2) > 0.05){

					swerve.run = 1;

				}else{

					swerve.run = 0;
				}

			}

		}

		update_param();

		static uint8_t led = 0;
		if (++led > 50) {
			led3 = !led3;
			led = 0;
		}
	}
}


void Microros(void *argument){

	MX_USB_DEVICE_Init();

	microros_init(&hpcd_USB_OTG_FS, "node", 1);

	/******* Publisher *******/
	rclc_publisher_init_best_effort(
	  &publisher,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(utmrbc_msgs, msg, Odometry),
	  "/ekf_input");

	/******* Subscriper *******/
	rclc_subscription_init_best_effort(
	  &subscriber,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(utmrbc_msgs, msg, Local),
	  "/final_pose");

	rclc_executor_add_subscription(
	  &executor,
	  &subscriber,
	  &local_msg,
	  &subscription_callback,
	  ON_NEW_DATA);

	while(1){

		rclc_executor_spin_some(&executor, 50 * 1000000);
		rcl_publish(&publisher, &ekf_msg, NULL);
		osDelay(1);

	}

}

void subscription_callback(const void * msgin){


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == IMU.huartx) {
		IMU_Handler(&IMU);
	}

	/*	if(huart == ch010.huartx){
		ch0x0Handler(&ch010);
	}
	 */

	/*	else if (huart == modbus.huartx){
		Modbus_Handler(&modbus);
	}
	 */


}

/**
 * @brief  This function is executed in case of error occurrence.
 */


void quaternion_orientation(double roll, double pitch, double yaw){

	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);

	ekf_msg.imu_yaw 		  = yaw;
	ekf_msg.imu_orientation_w = cr * cp * cy + sr * sp * sy;
	ekf_msg.imu_orientation_x = sr * cp * cy - cr * sp * sy;
	ekf_msg.imu_orientation_y = cr * sp * cy + sr * cp * sy;
	ekf_msg.imu_orientation_z = cr * cp * sy - sr * sp * cy;
}

void update_param(void){

	delta_x_pos = CURRENT_X_POS - prev_x_pos;
	delta_y_pos = CURRENT_Y_POS - prev_y_pos;

	x_ekf_pos  = CURRENT_X_POS + delta_x_pos;
	y_ekf_pos  = CURRENT_Y_POS + delta_y_pos;

	x_vel = (delta_x_pos)/0.007;
	y_vel = (delta_y_pos)/0.007;
	v_yaw = ((ch010.hi91.yaw - prev_yaw) * M_PI/180.0) / 0.007;

	prev_x_pos = CURRENT_X_POS;
	prev_y_pos = CURRENT_Y_POS;

	ekf_msg.odom_pose_x = x_ekf_pos;
	ekf_msg.odom_pose_y = y_ekf_pos;

	ekf_msg.imu_angular_vx		= ch010.hi91.gyr[1];
	ekf_msg.imu_angular_vy		= ch010.hi91.gyr[0];
	ekf_msg.imu_angular_vz		= ch010.hi91.gyr[2];

	ekf_msg.imu_linear_ax		= ch010.hi91.acc[1];
	ekf_msg.imu_linear_ay		= ch010.hi91.acc[0];

	PWMEncoder_Angle_Update(&enc1);
	PWMEncoder_Angle_Update(&enc2);
	PWMEncoder_Angle_Update(&enc3);
	PWMEncoder_Angle_Update(&enc4);

	kalman_filter_update();

	quaternion_orientation(ch010.hi91.roll, ch010.hi91.pitch, ch010.hi91.yaw);

}

void Error_Handler(void) {

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
