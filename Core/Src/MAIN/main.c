/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

#define SERVO_UP  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1900);
#define SERVO_MID  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000);
#define SERVO_DOWN  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500);

int main(void) {

	set();

	const osThreadAttr_t MainTask_attributes =
	{ .name = "MainTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t ServoTask_attributes =
	{ .name = "ServoTask", .stack_size = 256 * 4, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t CalculationTask_attributes =
	{ .name = "CalculationTask", .stack_size = 256 * 4, .priority =
			(osPriority_t) osPriorityNormal, };

	const osSemaphoreAttr_t CalcSemaphore_attributes = { .name = "CalcSemaphore" };


	osKernelInitialize();

	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	ServoTaskHandle = osThreadNew(ServoTask, NULL, &ServoTask_attributes);
	CalculationTaskHandle = osThreadNew(Calculation, NULL, &CalculationTask_attributes);
	CalcSemaphore = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);

	osKernelStart();
}

void TIM6_DAC_IRQHandler(void) { //10us

	//	SoftPWMUpdate();
	HAL_TIM_IRQHandler(&htim6);
}

void TIM7_IRQHandler(void) { //5ms

	osSemaphoreRelease(CalcSemaphore);

	static uint8_t led = 0;
	if (++led > 4) {
		led1 = !led1;
		led = 0;
		PSxConnectionHandler(&ps4);
		//		if (HAL_GetTick()-modbus.time > 50){
		//			Modbus_request_new_data(&modbus);
		//		}
	}

	HAL_TIM_IRQHandler(&htim7);
}


void MainTask(void *argument) {

	while (1) {

		//		if(!PB2){  // added, maintask not working
		//
		//			comm_can_set_duty(112, 0.1);
		//			comm_can_set_duty(113, 0.1);
		//			comm_can_set_duty(114, 0.1);
		//			static uint8_t led = 0;
		//					if (++led >= 255) {
		//						led2 = !led2;
		//						led = 0;
		//					}
		//		}

		update_param();

		//		x_vel = 1.5*(-ps4.joyL_x*cos(yaw_angle*3.14/180)-ps4.joyL_y*sin(yaw_angle*3.14/180));
		//		y_vel = 1.5*(-ps4.joyL_x*sin(yaw_angle*3.14/180)+ps4.joyL_y*cos(yaw_angle*3.14/180));

		x_vel = -1.0*ps4.joyL_x;
		y_vel =  1.0*ps4.joyL_y;
		//				w_vel =  0.5*(ps4.joyR_2 - ps4.joyL_2);

		static uint8_t led = 0;
		if (++led >= 255) {
			led3 = !led3;
			led = 0;
		}

		//		PWMEncoder_Angle_Update(&enc1);

	}

}


void ServoTask(void *argument) { //formerly SecondryTask
	PWMTimeBaseInit(&htim1, 20000, 168);
	PWMChannelConfig(&htim1, TIM_CHANNEL_4, IP3_PIN); // hspm1
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500);

	while (1) {
		if(!PB2){  //added


			static uint8_t led = 0;
			if (++led >= 255) {
				led2 = !led2;
				led = 0;
			}
		} //added
		if(ps4.button & RIGHT){
			while(ps4.button & RIGHT);
//			RBMS_Set_Target_Position(&rbms1, RBMS1, -10.0);
//			RBMS_Set_Target_Position(&rbms1, RBMS2, 10.0);
			RBMS_Set_Target_Position(&rbms1, RBMS3, 5.0);
		}

		if(ps4.button & CROSS){
			comm_can_set_duty(113, -0.5);
			comm_can_set_duty(112, 0.2);

		}
		if(ps4.button & CIRCLE){
			//			while(ps4.button & CIRCLE);
			comm_can_set_duty(113, 0.3);
			comm_can_set_duty(112, 0.3);
			SERVO_MID;
		}
		if(ps4.button & TRIANGLE){
			comm_can_set_duty(113, 0.3);
			comm_can_set_duty(112, 0.3);
			SERVO_UP;
		}
		if(ps4.button & SQUARE){
			while(ps4.button & SQUARE);
			SERVO_DOWN;
		}

		if(ps4.button & UP){
			stop_all();
		}
	}
}

void Calculation(void *argument) { //5ms

	update_param();
	target_angle = yaw_angle;

	while (1) {

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);
		RBMS_5ms(&rbms1);
		//		if(!PB1){ //not working either led8 not blinking
		//			comm_can_set_duty(112, 0.1);
		//			comm_can_set_duty(113, 0.1);
		//			comm_can_set_duty(114, 0.1);
		//		}

		static uint8_t led = 0;
		if (++led > 4) {
			led8 = !led8;
			led = 0;
		}

		if(STOP_CONDITION){

			RNSStop(&rns);

		}else{

			RNSVelocity(v1,v2,v3,v4,&rns);

		}

		MODNUpdate(&modn);
		PID(&yaw_pid);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//	if (huart == IMU.huartx) {
	//		IMU_Handler(&IMU);
	//	}

	//	else if (huart == modbus.huartx){
	//		Modbus_Handler(&modbus);
	//	}
}

/**
 * @brief  This function is executed in case of errorYaw occurrence.
 */
void Error_Handler(void) {

}

void update_param(void){

	//	RNSEnquire(RNS_COORDINATE_X_Y_Z, &rns);
	//	x_pos 	  = rns.RNS_data.common_buffer[0].data;
	//	y_pos 	  = rns.RNS_data.common_buffer[1].data;
	//	yaw_angle = rns.RNS_data.common_buffer[2].data;
	//	error_angle  = target_angle - yaw_angle;
	RNSEnquire(RNS_VEL_BOTH, &rns);
	test.m1 = rns.RNS_data.common_buffer[0].data;
	test.m2 = rns.RNS_data.common_buffer[1].data;
	test.m3 = rns.RNS_data.common_buffer[2].data;
	test.m4 = rns.RNS_data.common_buffer[3].data;
}

void stop_all(void){

	HAL_NVIC_SystemReset();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
