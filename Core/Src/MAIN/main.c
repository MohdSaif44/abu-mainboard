/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {

 	set();

	const osThreadAttr_t MainTask_attributes =
	{ .name = "MainTask", .stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t SecondaryTask_attributes =
	{ .name = "SecondTask", .stack_size = 256 * 4, .priority =
			(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t CalculationTask_attributes =
	{ .name = "CalculationTask", .stack_size = 256 * 4, .priority =
			(osPriority_t) osPriorityNormal, };

	const osSemaphoreAttr_t CalcSemaphore_attributes = { .name = "CalcSemaphore" };


	osKernelInitialize();

	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	SecondaryTaskHandle = osThreadNew(SecondaryTask, NULL, &SecondaryTask_attributes);
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

		update_param();

		x_vel = 1*(-ps4.joyL_x*cos(yaw_angle*3.14/180)-ps4.joyL_y*sin(yaw_angle*3.14/180));
		y_vel = 1*(-ps4.joyL_x*sin(yaw_angle*3.14/180)+ps4.joyL_y*cos(yaw_angle*3.14/180));

//		x_vel = -2.0*ps4.joyL_x;
//		y_vel =  2.0*ps4.joyL_y;

		static uint8_t led = 0;
		if (++led >= 255) {
			led3 = !led3;
			led = 0;
		}

//		PWMEncoder_Angle_Update(&enc1);

	}

}


void SecondaryTask(void *argument) {

	while (1) {
		static uint8_t led = 0;
		if (++led >= 255) {
			led2 = !led2;
			led = 0;
		}
	}
}

void Calculation(void *argument) { //5ms

	update_param();
	target_angle = yaw_angle;

	while (1) {

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
