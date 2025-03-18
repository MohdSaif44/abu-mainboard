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
			.priority = (osPriority_t) osPriorityHigh,   };

	const osThreadAttr_t MainTask_attributes =
	{ .name = "MainTask", .stack_size = 128 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SecondaryTask_attributes =
	{ .name = "SecondTask", .stack_size = 128 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SwerveATask_attributes =
	{ .name = "SwerveATask", .stack_size = 256 * 2, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SwerveBTask_attributes =
	{ .name = "SwerveBTask", .stack_size = 256 * 2, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SwerveCTask_attributes =
	{ .name = "SwerveCTask", .stack_size = 256 * 2, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SwerveDTask_attributes =
	{ .name = "SwerveDTask", .stack_size = 256 * 2, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t CalculationTask_attributes =
	{ .name = "CalculationTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osSemaphoreAttr_t CalcSemaphore_attributes 	 = { .name = "CalcSemaphore" 	 };

	const osSemaphoreAttr_t PathplanSemaphore_attributes = { .name = "PathplanSemaphore" };

	const osSemaphoreAttr_t MainSemaphore_attributes 	 = { .name = "MainSemaphore" 	 };


	osKernelInitialize();

	MainTaskHandle		  = osThreadNew(MainTask, NULL, &MainTask_attributes);
	MicrorosTaskHandle    = osThreadNew(Microros, NULL, &MicrorosTask_attributes);
	SecondaryTaskHandle   = osThreadNew(SecondaryTask, NULL, &SecondaryTask_attributes);
	SwerveATaskHandle 	  = osThreadNew(SwerveATask, NULL, &SwerveATask_attributes);
	SwerveBTaskHandle     = osThreadNew(SwerveBTask, NULL, &SwerveBTask_attributes);
	SwerveCTaskHandle     = osThreadNew(SwerveCTask, NULL, &SwerveCTask_attributes);
	SwerveDTaskHandle     = osThreadNew(SwerveDTask, NULL, &SwerveDTask_attributes);
	CalculationTaskHandle = osThreadNew(Calculation, NULL, &CalculationTask_attributes);
	CalcSemaphore 		  = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);
	MainSemaphore 		  = osSemaphoreNew(1, 0, &MainSemaphore_attributes);

	osKernelStart();
}

// ssh utmrbc@10.161.42.28


void TIM6_DAC_IRQHandler(void) { 	// 10us

	//	SoftPWMUpdate();
	HAL_TIM_IRQHandler(&htim6);
}
uint8_t run_once = 0;
float reciever_offset = 0.0;

void TIM7_IRQHandler(void) { 		// 5ms


	static uint8_t led = 0;
	static uint8_t calc_count = 0;
	static uint8_t rbms_count = 0;

	vel_ramp();

	osSemaphoreRelease(MainSemaphore);
	osSemaphoreRelease(CalcSemaphore);
	osSemaphoreRelease(PathplanSemaphore);

	if(++rbms_count>1){
		RBMS_5ms(&rbms2);
		if(sys.flag13 == 0){
			if(run_once){
				reciever_offset = 0.0;
//				RBMS_Set_Control_Mode(&rbms2, RBMS5, POSITION);
				run_once = 0;
			}
			RBMS_Set_Target_Position(&rbms2, RBMS5, -reciever_offset);
		}

		else if(sys.flag13 == 1){
			if(run_once){
				shooter_heading(local.x, local.y);
//				RBMS_Set_Control_Mode(&rbms2, RBMS5, VELOCITY);
				run_once = 0;
			}
			RBMS_Set_Target_Position(&rbms2, RBMS5, -heading_rbms);

		}

		rbms_count = 0;
	}


	if (++led > 4){
		PSxConnectionHandler(&ps4);
		led1 = !led1;
		led = 0;
	}

	HAL_TIM_IRQHandler(&htim7);
}

//uint8_t hall1,hall2,hall3,hall4;
uint8_t drive_mode = 4;

float x_factor, y_factor, raw_x, raw_y, actual_x, actual_y;

void MainTask(void *argument) {

	z_target_angle 		 = 0.0;
	vesc.pole_pairs 	 = 28;
	vesc.gear_ratio 	 = 2;
	vesc.wheel_diameter  = 0.079;
	vesc1.pole_pairs 	 = 14;
	vesc1.gear_ratio 	 = 1.8;
	vesc1.wheel_diameter = 0.0442;
	actual_y 			 = 0.3;
	actual_x 			 = 0.3;
	while (1) {

		osSemaphoreAcquire(MainSemaphore, osWaitForever);

		actual_y += (((QEIRead(QEI4) - raw_x)*cos(IMU.real_zrad) - (QEIRead(QEI1) - raw_y)*sin(IMU.real_zrad)) * 0.05 * M_PI/ 8192.0);
		actual_x += (((QEIRead(QEI1) - raw_y)*cos(IMU.real_zrad) + (QEIRead(QEI4) - raw_x)*sin(IMU.real_zrad)) * 0.05 * M_PI/ 8192.0);

		raw_x =  QEIRead(QEI4);
		raw_y =  QEIRead(QEI1);


		if(ps4.button & TRIANGLE){

			reset_enc();
			stop_all();
			actual_y = 0.3;
			actual_x = 0.3;

		}
		if(ps4.button & SQUARE){

			IP3_OUT = 1;
			osDelay(1000);

		}
		else{

			IP3_OUT = 0;

		}

		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 1000);

//		if(ps4.button & DOWN){
//
//
//		}

		reciver_heading = 90.0 - atan2f((local.y-local_kd.y),(local.x - local_kd.x)) * 180.0/M_PI;

		static uint8_t led = 0;
		if (++led >= 255) {
			led2 = !led2;
			led = 0;
		}
	}
}


void SecondaryTask(void *argument) {

	while (1) {

		osSemaphoreAcquire(PathplanSemaphore, osWaitForever);

		if(ps4.button & RIGHT){

			WriteBDC(&BDC8, 10000);

		}else if(ps4.button & LEFT){

			WriteBDC(&BDC8, -10000);

		}else{

			WriteBDC(&BDC8, 0);

		}

		if(IP3_IN == 1){
			osDelay(100);
			comm_can_brake(VESC5);
			comm_can_brake(VESC6);
		}

		if(ps4.button & UP){

			while(ps4.button & UP);

			sys.flag11++;
			if (sys.flag11 == 1){
				drive_mode = 3;
			}else{
				drive_mode = 4;
			}
		}

		if(ps4.button & DOWN){
			while(ps4.button & DOWN);
			run_once = 1;
			sys.flag13++;
			if(sys.flag13 > 2){
				sys.flag13 = 1;
			}
		}
	}
}


void SwerveATask (void *argument){

	swerve_init(&swerveA, 3, 1, &filtered_enc1);
	while(1){

		if (swerveA.alligning_status != SWERVE_ALLIGNED) {
			swerve_allign(&swerveA, IP9_PIN, 0.0, 225.530884);
		} else {
			osThreadExit();
		}

	}
}

void SwerveBTask (void *argument){
	swerve_init(&swerveB, 3, 2, &filtered_enc2);
	while(1){

		if (swerveB.alligning_status != SWERVE_ALLIGNED) {
			swerve_allign(&swerveB, IP10_PIN, 0.0, 119.392998);
		} else {
			osThreadExit();
		}

	}
}

void SwerveCTask (void *argument){
	swerve_init(&swerveC, 3, 3, &filtered_enc3);
	while(1){

		if(swerveC.alligning_status != SWERVE_ALLIGNED){
			swerve_allign(&swerveC, IP11_PIN, 0.0, 300.182739);
		}else{
			osThreadExit();
		}
	}
}

void SwerveDTask (void *argument){
	swerve_init(&swerveD, 3, 4, &filtered_enc4);
	while(1){

		if(swerveD.alligning_status != SWERVE_ALLIGNED){
			swerve_allign(&swerveD, IP12_PIN, 0.0, 107.817184);
		}else{
			osThreadExit();
		}
	}
}


uint8_t toggle;

void Calculation(void *argument) { 	// 5ms

	sys.manual    = 0;
	sys.control   = 1;
	sys.automatic = 0;

	while (1) {

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);

//		shooter_heading(local_msg.local_x, local_msg.local_y);

//		hall1 = IP9_IN;
//		hall2 = IP5_IN;
//		hall3 = IP11_IN;
//		hall4 = IP12_IN;

//		if(ps4.button == RIGHT){
//
//			comm_can_set_duty(116, 0.7);
//
//		}
		RBMS_5ms(&rbms1);

		if(ps4.button == CROSS){

			sys.manual 	   = 1;

		}

		if(ps4.button == CIRCLE){

//			VESCVelocity(5.0, 5.0, 0, 0, &vesc1);
			if(IP3_IN == 0){
				comm_can_set_duty(VESC5, 0.8);
				comm_can_set_duty(VESC6, 0.8);

			}
		}

		if(!PB1){

//			VESCVelocity(5.0, 5.0, 0, 0, &vesc1);
			if(IP3_IN == 0){
				comm_can_set_current(VESC5, 70.0);
				comm_can_set_current(VESC6, 70.0);

			}
		}


//		if(ps4.button == LEFT){
//
//			while(ps4.button == LEFT);
//
//			sys.automatic = 1;
//			sys.manual    = 0;
//
//		}

		if(sys.manual){

//			SwerveAlign(94.0, 226.0, 307.0, 111.0, hall_sensor1_pin, hall_sensor2_pin, hall_sensor3_pin, hall_sensor4_pin);

//			D : 70.0

			SwerveRun(1.0, drive_mode, 20.0, 100, 0);

			if(swerveA.alligning_status == SWERVE_ALLIGNED
					&& swerveB.alligning_status == SWERVE_ALLIGNED
					&& swerveC.alligning_status == SWERVE_ALLIGNED
					&& swerveD.alligning_status == SWERVE_ALLIGNED){

				if(fabs(joy_x_vel)+fabs(joy_y_vel)+fabs(w_vel)+fabs(ps4.joyR_2 - ps4.joyL_2) > 0.0){

					MODNUpdate(&modn);

					swerve.run = 1;

				}else{

					swerve.run = 0;
				}

			}

		}

		if(sys.automatic){

			SwerveRun(0.1, pathplan, 20.0, 30, 0);

			if(fabs(vx)+fabs(vy) > 0.05){

				swerve.run = 1;

			}else{

				swerve.run = 0;
			}

		}


		update_param();
		PID(&imu_rotate);

		if(sys.flag14){

			PID(&c_pid);

		}

		static uint8_t led = 0;
		if (++led > 4) {
			led3 = !led3;
			led = 0;
		}
	}
}

void Microros(void *argument){

	MX_USB_DEVICE_Init();

	microros_init(&hpcd_USB_OTG_FS, "node", 2);

	/******* Publisher *******/
	rclc_publisher_init_best_effort(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
			"/encoder");

	/******* Subscriper *******/
	rclc_subscription_init_best_effort(
			&subscriber,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
			"/local");

		/*rclc_subscription_init_best_effort(
		  &subscriber2,
		  &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		  "/basket_x_coordinate");*/

	rclc_subscription_init_best_effort(
			&subscriber3,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
			"/kd/local");


	rclc_executor_add_subscription(
			&executor,
			&subscriber,
			&local,
			&subscription1_callback,
			ON_NEW_DATA);

		/*rclc_executor_add_subscription(
		  &executor,
		  &subscriber2,
		  &Float32,
		  &subscription1_callback,
		  ON_NEW_DATA);*/

	rclc_executor_add_subscription(
			&executor,
			&subscriber3,
			&local_kd,
			&subscription1_callback,
			ON_NEW_DATA);


	while(1){

		rclc_executor_spin_some(&executor, 50 * 1000000);
		rcl_publish(&publisher, &encoder, NULL);
		osDelay(10);

	}

}



void subscription1_callback(const void * msgin){

	sys.flag14=1;

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {


	if (huart == IMU.huartx) {
		IMU_Handler(&IMU);
	}


	/*if(huart == ch010.huartx){
		ch0x0Handler(&ch010);
	}*/


	/*	else if (huart == modbus.huartx){
		Modbus_Handler(&modbus);
	}*/


}

/**
 * @brief  This function is executed in case of error occurrence.
 */


void quaternion_orientation(double roll, double pitch, double yaw){

	double cr = cos(roll  * 0.5);
	double sr = sin(roll  * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cy = cos(yaw   * 0.5);
	double sy = sin(yaw   * 0.5);

	ekf_msg.imu_yaw 		  = yaw;
	ekf_msg.imu_orientation_w = cr * cp * cy + sr * sp * sy;
	ekf_msg.imu_orientation_x = sr * cp * cy - cr * sp * sy;
	ekf_msg.imu_orientation_y = cr * sp * cy + sr * cp * sy;
	ekf_msg.imu_orientation_z = cr * cp * sy - sr * sp * cy;
}

void update_param(void){

	encoder.x = actual_x;
	encoder.y = actual_y;
	encoder.z = IMU.real_z;

	error_x   = target_pos_x - x_ekf_pos;
	error_y	  =	target_pos_y - y_ekf_pos;

	cam_pos = Float32.data;

	xcam_error = 240 - cam_pos;

	kalman_filter_update();

}

float clamp(float min,float value,float max){

	if(value>=max){

		return max;

	}

	else if(value<=min){

		return min;

	}

	else{

		return value;

	}
}
float ratio_vel;

void vel_ramp(){

	tVx	=	 1.0*(ps4.joyL_x*cos(-IMU.real_zrad)-ps4.joyL_y*sin(-IMU.real_zrad));
	tVy	=	 1.0*(ps4.joyL_x*sin(-IMU.real_zrad)+ps4.joyL_y*cos(-IMU.real_zrad));
	tVw = 	-1.0*(ps4.joyR_2 - ps4.joyL_2);

	step_x		=	0.009;
	step_y		=	0.009;
	step_w      =   0.009;

	full_step_x	=	tVx-Vx;
	full_step_y	=	tVy-Vy;
	full_step_w =   tVw-Vw;


	joy_x_vel+=  clamp(-step_x,full_step_x,step_x);
	joy_y_vel+=  clamp(-step_y,full_step_y,step_y);
	joy_w_vel+=  clamp(-step_w,full_step_w,step_w);

	Vx=joy_x_vel;
	Vy=joy_y_vel;
	Vw=joy_w_vel;
}


void stop_all(void){

	VESCStop(&vesc);
	VESCCurrBrake(20, &vesc);
	swerve.run = 0;
	//	sys.automatic = 0;
	HAL_NVIC_SystemReset();
	sys.manual = 1;

}

void Error_Handler(void) {

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
