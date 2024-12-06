/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"
#include "Robomaster/Robomaster.h"
#include "usb_device.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

#define swerve_gear_ratio 	2.95
#define swerve_max_turn  	4.0
#define robot_width   		0.616
#define robot_lenght  		0.616
#define BASKET_Y            10
#define BASKET_X            3.75


#define hall_sensor1_pin	GPIOC, GPIO_PIN_8
#define hall_sensor2_pin	GPIOB, GPIO_PIN_14
#define hall_sensor3_pin	GPIOD, GPIO_PIN_3
#define hall_sensor4_pin	GPIOD, GPIO_PIN_4

#define IP1  	HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  	HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  	HAL_GPIO_ReadPin(IP3_PIN)
#define IP4		HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 	HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 	HAL_GPIO_ReadPin(IP6_PIN)

#ifdef newpin
#define IP7		HAL_GPIO_ReadPin(IP7_Analog1_PIN)
#define IP8		HAL_GPIO_ReadPin(IP8_Analog2_PIN)
#else
#define IP7		HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 	HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  	HAL_GPIO_ReadPin(IP9_PIN)

#define IP10    HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  	HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 	HAL_GPIO_ReadPin(IP12_PIN)

#ifndef mainboard3_3
#define IP13  	HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 	HAL_GPIO_ReadPin(IP14_PIN)
#define IP15	HAL_GPIO_ReadPin(IP15_PIN)
#endif

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18 	HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)
#endif

#define Mux1	MUX.mux_data.bit0
#define Mux2	MUX.mux_data.bit1
#define Mux3	MUX.mux_data.bit2
#define Mux4	MUX.mux_data.bit3
#define Mux5	MUX.mux_data.bit4
#define Mux6	MUX.mux_data.bit5
#define Mux7	MUX.mux_data.bit6
#define Mux8	MUX.mux_data.bit7

osEventFlagsId_t evt_id;
osThreadId_t MicrorosTaskHandle;
osThreadId_t MainTaskHandle;
osThreadId_t SecondaryTaskHandle;
osThreadId_t CalculationTaskHandle;
//osThreadId_t ModbusTaskHandle;

osSemaphoreId_t CalcSemaphore;
osSemaphoreId_t MainSemaphore;

typedef union{
	uint32_t flags;
	struct{

		/*Least significant 16 bits can be cleared all at once by
				sys.flags = 0 for example during emergency*/

		unsigned control      :1;
		unsigned tunePid      :1;
		unsigned tunePidex    :1;
		unsigned tunePP		  :1;
		unsigned manual       :1;
		unsigned automatic	  :1;
		unsigned ppstart      :1;
		unsigned ppend		  :1;
		unsigned shot         :1;
		unsigned tuning       :1;
		unsigned side         :1;
		unsigned flag11       :1;
		unsigned flag12       :1;
		unsigned flag13       :1;
		unsigned flag14       :1;
		unsigned ros_ready    :1;


		//Most significant 16 bits are not cleared

		unsigned flag16		  :1;
		unsigned flag17		  :1;
		unsigned flag18		  :1;
		unsigned flag19		  :1;
		unsigned flag20		  :1;
		unsigned flag21		  :1;
		unsigned flag22		  :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29		  :1;
		unsigned flag30		  :1;
		unsigned flag31		  :1;

	};

}sys_t;

volatile sys_t sys;

void RNS_config(CAN_HandleTypeDef* hcanx);

void update_param(void);

#endif /* INC_COMMON_H_ */
