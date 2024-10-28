/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"


uint8_t blockturn = 0;

void set(void) {

	sys.flag16=0;
//	KalmanFilter_Init(&enc1.Angle, &Filtered_Angle, 1.0, 1.0, 0.001, 1.0, 1.0, &KF);
//	watchdoginit(4, 4);   // use the watchdog refresh inside 5ms loop
	Initialize();
	PSxSlaveInit(&ps4, &hi2c1);
//	PMW3901_SlaveInit(&enc, &hi2c2);
//	Modbus_Init(&modbus, &huart4);
//	TIMxInit(&htim6, 50, 84);			// 50us use for SoftPWM
	TIMxInit(&htim7, 5000, 84);			// 5ms
	TIMxInit(&htim9, 65535, 74);
	RNS_config(&hcan1);
//	ExtixInit(GPIO_PIN_0, 9, 0,&ExtiPin);
	MODNRobotBaseVelInit(MODN_FWD_OMNI, 0.47, 0.47, &modn);
	MODNRobotConInit(&x_vel, &y_vel, &w_vel, &modn);
	MODNWheelVelInit(&v1, &v2, &v3, &v4, &modn);
//	MODNWheelDirInit(&d2, &d1, &d3, &d4, &modn);
	PIDSourceInit(&error_angle, &w_vel, &yaw_pid);
	PIDGainInit(0.005, 0.5, 0.5, 0.5, 0.35, 0.0, 0.0, 10.0, &yaw_pid);

}
	/***NAVI***/

	//	rns.init = 0;
	void RNS_config(CAN_HandleTypeDef* hcanx) {
		RNSInit(hcanx, &rns);
		//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
		RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b10100101, (float) fwd_omni, (float) roboconPID);
		RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000 * 3.142, 1.0, 0.05 / 4000 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
		RNSSet(&rns, RNS_F_KCD_PTD, 6615.0/ 3854.0, (float)(0.125 * 3.142 / 6615.0));
		RNSSet(&rns, RNS_B_KCD_PTD, 6589.0/ 6046.0, (float)(0.125 * 3.142 / 6589.0));

		RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 3.27, 19999.0);
		RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 3.52, 19999.0);
		RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 3.86, 19999.0);
		RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 3.61, 19999.0);

		RNSSet(&rns, RNS_F_LEFT_VEL_PID,  1.0, 0.0, 0.0);
		RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.0, 0.0, 0.0);
		RNSSet(&rns, RNS_B_LEFT_VEL_PID,  1.0, 0.0, 0.0);
		RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 1.0, 0.0, 0.0);

		RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
		RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

		RNSSet(&rns, RNS_PPInit); //Path Planning
		RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
		RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
		RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
		RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
	}






