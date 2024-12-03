/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"


uint8_t blockturn = 0;

void set(void) {

	sys.flag16=0;
	Initialize();
	PSxSlaveInit(&ps4, &hi2c1);
	TIMxInit(&htim7, 1000, 84);			// 1ms
	TIMxInit(&htim9, 65535, 74);		// Encoder Alignment
	SwerveInit(FWD_SWERVE, unlimitedturn, swerve_max_turn, robot_width, robot_lenght, swerve_gear_ratio);
	SwerveAligninit(GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
	KalmanFilter_Init(&enc1.Angle, &filtered_enc1, 1.0, 1.0, 0.001, 1.0, 1.0, &KF1);
	KalmanFilter_Init(&enc2.Angle, &filtered_enc2, 1.0, 1.0, 0.001, 1.0, 1.0, &KF2);
	KalmanFilter_Init(&enc3.Angle, &filtered_enc3, 1.0, 1.0, 0.001, 1.0, 1.0, &KF3);
	KalmanFilter_Init(&enc4.Angle, &filtered_enc4, 1.0, 1.0, 0.001, 1.0, 1.0, &KF4);
	RBMS_Init(&rbms2, &hcan2, RBMS_5678);
	RBMS_Config(&rbms2, RBMS5, C610, 1);
	RBMS_Config(&rbms2, RBMS6, C610, 1);
	RBMS_Config(&rbms2, RBMS7, C610, 1);
	RBMS_Config(&rbms2, RBMS8, C620, 1);
	RBMS_PID_Init(&rbms2);
	RBMS_Set_Control_Mode(&rbms2, RBMS5, VELOCITY);
	RBMS_Set_Control_Mode(&rbms2, RBMS6, VELOCITY);
	RBMS_Set_Control_Mode(&rbms2, RBMS7, VELOCITY);
	RBMS_Set_Control_Mode(&rbms2, RBMS8, VELOCITY);
//	TIMxInit(&htim6, 50, 84);			// 50us use for SoftPWM
//	watchdoginit(4, 4);   // use the watchdog refresh inside 5ms loop
//	RNS_config(&hcan1);
//	PMW3901_SlaveInit(&enc, &hi2c2);
//	Modbus_Init(&modbus, &huart4);
//	MODNWheelDirInit(&d2, &d1, &d3, &d4, &modn);

}
	/***NAVI***/

	//	rns.init = 0;
void RNS_config(CAN_HandleTypeDef* hcanx) {

	RNSInit(hcanx, &rns);
	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b10100101, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.125 / 4000 * 3.142, 1.0, 0.125 / 4000 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, 203.20885/ 204.50492, (float)(0.125 * 3.142 / 203.20885));
	RNSSet(&rns, RNS_B_KCD_PTD, 203.56232/ 203.60160, (float)(0.125 * 3.142 / 203.56232));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 17.9120, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 20.7897, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 18.3077, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 18.7605, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  3.0, 3.0, 0.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 3.0, 3.0, 0.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  3.0, 3.0, 0.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 3.0, 3.0, 0.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path

}

void reset_enc(void) {

	QEIReset(QEI1);
	QEIReset(QEI4);

}

void kalman_filter_update(void){

	KalmanFilter_5ms(&KF1);
	KalmanFilter_5ms(&KF2);
	KalmanFilter_5ms(&KF3);
	KalmanFilter_5ms(&KF4);

}


