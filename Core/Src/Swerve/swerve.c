/*
 * swerve.c
 *
 *  Created on:
 *      Author:
 */

#include "swerve.h"
#include <math.h>

void SwerveInit(int swerve_type, int turn_mode, float maximum_turn, float width, float length, float swerve_gear_ratio)
{
	swerve.turnmode = turn_mode;
	swerve.maxturn = maximum_turn;

	if (swerve_type == FWD_SWERVE)
	{
		swerve.type = FWD_SWERVE;

		MODNRobotBaseVelInit(MODN_FWD_SWERVE, width, length, &modn);
		MODNRobotConInit(&x_vel, &y_vel, &w_vel, &modn);
		MODNWheelVelInit(&v1, &v2, &v3, &v4, &modn);
		MODNWheelDirInit(&d1, &d2, &d3, &d4, &modn);

		VESCInit(VESC1, VESC3, VESC2, VESC4, &vesc); //FL,FR,BL,BR

		RBMS_Init(&rbms1, &hcan2, RBMS_1234);
		RBMS_Config(&rbms1, RBMS1, C610, swerve_gear_ratio);//FL
		RBMS_Config(&rbms1, RBMS2, C610, swerve_gear_ratio);//FR
		RBMS_Config(&rbms1, RBMS3, C610, swerve_gear_ratio);//BL
		RBMS_Config(&rbms1, RBMS4, C610, swerve_gear_ratio);//BR
		RBMS_PID_Init(&rbms1);
		RBMS_Set_Control_Mode(&rbms1, RBMS1, POSITION);
		RBMS_Set_Control_Mode(&rbms1, RBMS2, POSITION);
		RBMS_Set_Control_Mode(&rbms1, RBMS3, POSITION);
		RBMS_Set_Control_Mode(&rbms1, RBMS4, POSITION);
		rbms1.motor[RBMS1].config.vel_limit = 300;
		rbms1.motor[RBMS2].config.vel_limit = 300;
		rbms1.motor[RBMS3].config.vel_limit = 300;
		rbms1.motor[RBMS4].config.vel_limit = 300;
		rbms1.motor[RBMS1].config.POS_P = 200;
		rbms1.motor[RBMS2].config.POS_P = 200;
		rbms1.motor[RBMS3].config.POS_P = 200;
		rbms1.motor[RBMS4].config.POS_P = 200;
	}

	PIDSourceInit(&pid_angle_error, &pid_angle_output, &imu_rotate);
	PIDGainInit(0.005, 0.75, 0.5, 0.5, 0.45, 0.0, 0.0, 10.0, &imu_rotate);
	PIDDelayInit(&imu_rotate);

}

void SwerveRun(float max_speed, int mode, float brake_current, int brake_time, float target_angle)
{


	if (swerveA.alligning_status == SWERVE_ALLIGNED
			&& swerveB.alligning_status == SWERVE_ALLIGNED
			&& swerveC.alligning_status == SWERVE_ALLIGNED
			&& swerveD.alligning_status == SWERVE_ALLIGNED)
	{
		if (mode == manualnolock) {
			PIDDelayInit(&imu_rotate);
			x_vel = ps4.joyL_x;
			y_vel = ps4.joyL_y;
			w_vel = joy_w_vel;
		}
		else if (mode == manuallock)
		{
			//			PID(&imu_rotate);
			pid_angle_error = z_target_angle - IMU.real_z;

			x_vel = 0.0;
			y_vel = joy_y_vel;
			w_vel = (-0.35)*pid_angle_output;
		}
		else if (mode == perspective)
		{
			pid_angle_error = z_target_angle - IMU.real_z;

			x_vel = joy_x_vel;				/*(ps4.joyL_x*cos(-IMU.real_zrad)-ps4.joyL_y*sin(-IMU.real_zrad));*/
			y_vel = joy_y_vel;				/*(ps4.joyL_x*sin(-IMU.real_zrad)+ps4.joyL_y*cos(-IMU.real_zrad));*/


			if(ps4.button != R1){

				if(fabs(joy_w_vel) > 0.05){

					w_vel = joy_w_vel;
					z_target_angle = IMU.real_z;

				}else if(fabs(joy_y_vel) + fabs(joy_x_vel) > 0.05){

					w_vel = (-0.125)*pid_angle_output;


				}else{

					w_vel = 0.0;

				}

			}else{

				z_target_angle = reciver_heading;
				w_vel = (-0.2)*pid_angle_output;

			}

		}
		else if (mode == pathplan)
		{
			pid_angle_error = z_target_angle - IMU.real_z;
			x_vel = -(vx*cos(-IMU.real_zrad)-vy*sin(-IMU.real_zrad));
			y_vel = (vx*sin(-IMU.real_zrad)+vy*cos(-IMU.real_zrad));
			w_vel = (-1)*pid_angle_output;

		}


		if (swerve.turnmode == halfturn)
		{
			if (swerve.run)
			{
				swerve.run = 0;

				if (swerve.type == FWD_SWERVE)
				{
					swerve.sign[0] = (d1 > 90.0 || d1 < -90.0) ? -1.0 : 1.0 ;
					swerve.sign[1] = (d2 > 90.0 || d2 < -90.0) ? -1.0 : 1.0 ;
					swerve.sign[2] = (d3 > 90.0 || d3 < -90.0) ? -1.0 : 1.0 ;
					swerve.sign[3] = (d4 > 90.0 || d4 < -90.0) ? -1.0 : 1.0 ;
					d1 = (d1 > 90.0) ? d1 - 180 : (d1 < -90.0) ? d1 + 180 : d1 ;
					d2 = (d2 > 90.0) ? d2 - 180 : (d2 < -90.0) ? d2 + 180 : d2 ;
					d3 = (d3 > 90.0) ? d3 - 180 : (d3 < -90.0) ? d3 + 180 : d3 ;
					d4 = (d4 > 90.0) ? d4 - 180 : (d4 < -90.0) ? d4 + 180 : d4 ;

					swerve.fvel[0] = v1 * swerve.sign[0] * max_speed * fabs(cos((swerve.finalang[0]-d1) * (M_PI / 180.0))) ;
					swerve.fvel[1] = v2 * swerve.sign[1] * max_speed * fabs(cos((swerve.finalang[1]-d2) * (M_PI / 180.0))) ;
					swerve.fvel[2] = v3 * swerve.sign[2] * max_speed * fabs(cos((swerve.finalang[2]-d3) * (M_PI / 180.0))) ;
					swerve.fvel[3] = v4 * swerve.sign[3] * max_speed * fabs(cos((swerve.finalang[3]-d4) * (M_PI / 180.0))) ;

					swerve.finalang[0] = d1;
					swerve.finalang[1] = d2;
					swerve.finalang[2] = d3;
					swerve.finalang[3] = d4;

					VESCPDC(swerve.fvel[0], swerve.fvel[1], swerve.fvel[2], swerve.fvel[3], &vesc);


					RBMS_Set_Target_Position(&rbms1, RBMS4, swerve.finalang[0] / 360.0);
					RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.finalang[1] / 360.0);
					RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.finalang[2] / 360.0);
					RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.finalang[3] / 360.0);
				}

				swerve.braketimer = 0;
			}
			else
			{

				if (swerve.type == FWD_SWERVE)
				{
					swerve.fvel[0] = 0.0;
					swerve.fvel[1] = 0.0;
					swerve.fvel[2] = 0.0;
					swerve.fvel[3] = 0.0;

					if (swerve.braketimer < brake_time)
					{
						swerve.braketimer++;

						VESCCurrBrake(brake_current, &vesc);

						RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.finalang[0] / 360.0);
						RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.finalang[1] / 360.0);
						RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.finalang[2] / 360.0);
						RBMS_Set_Target_Position(&rbms1, RBMS4, swerve.finalang[3] / 360.0);
					}
					else
					{
						RBMS_Set_Target_Position(&rbms1, RBMS1, 0);
						RBMS_Set_Target_Position(&rbms1, RBMS2, 0);
						RBMS_Set_Target_Position(&rbms1, RBMS3, 0);
						RBMS_Set_Target_Position(&rbms1, RBMS4, 0);

						for(int i = 0; i < 4; i++)
						{
							swerve.sign[i] = 0.0;
							swerve.finalang[i] = 0.0;
						}
					}
				}
				else if (swerve.type == TRI_SWERVE)
				{
					swerve.fvel[0] = 0.0;
					swerve.fvel[1] = 0.0;
					swerve.fvel[2] = 0.0;

					if (swerve.braketimer < brake_time)
					{
						swerve.braketimer++;

						VESCCurrBrake(brake_current, &vesc);

						RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.finalang[0] / 360.0);
						RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.finalang[1] / 360.0);
						RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.finalang[2] / 360.0);
					}
					else
					{
						RBMS_Set_Target_Position(&rbms1, RBMS1, 0);
						RBMS_Set_Target_Position(&rbms1, RBMS2, 0);
						RBMS_Set_Target_Position(&rbms1, RBMS3, 0);

						for(int i = 0; i < 4; i++)
						{
							swerve.sign[i] = 0.0;
							swerve.finalang[i] = 0.0;
						}
					}
				}
			}
		}
		else if (swerve.turnmode == unlimitedturn)
		{
			if (swerve.run)
			{
				swerve.timeout = 0;
				swerve.run     = 0;

				if (swerve.type == FWD_SWERVE)
				{
					swerve.ang[0] = d1;
					swerve.ang[1] = d2;
					swerve.ang[2] = d3;
					swerve.ang[3] = d4;

					for(int i = 0; i < 4; i++)
					{
						if(swerve.ang[i] - swerve.angprev[i] < -180.0)
						{
							swerve.addvalue[i] += 360.0;
						}
						else if(swerve.ang[i] - swerve.angprev[i] > 180.0)
						{
							swerve.addvalue[i] -= 360.0;
						}

						swerve.fang[i] = swerve.ang[i] + swerve.addvalue[i];

						swerve.ang1[i] = swerve.fang[i] - swerve.fangprev[i];
						swerve.ang2[i] = (swerve.ang1[i] >= 0)? swerve.ang1[i] - 180.0 : swerve.ang1[i] + 180.0;
						swerve.angprev[i] = swerve.ang[i];
						swerve.fangprev[i] = swerve.fang[i];

						if(swerve.state[i] == 0)
						{
							swerve.finalang[i] = swerve.finalangprev[i] + swerve.ang1[i];
							swerve.sign[i] = 1;
							if(fabs(swerve.ang1[i]) > fabs(swerve.ang2[i]))
							{
								swerve.state[i] = 1;
								swerve.finalang[i] = swerve.finalangprev[i] + swerve.ang2[i];
								swerve.sign[i] = -1;
							}
						}
						else if(swerve.state[i] == 1)
						{
							swerve.finalang[i] = swerve.finalangprev[i] + swerve.ang1[i];
							swerve.sign[i] = -1;
							if(fabs(swerve.ang1[i]) > fabs(swerve.ang2[i]))
							{
								swerve.state[i] = 0;
								swerve.finalang[i] = swerve.finalangprev[i] + swerve.ang2[i];
								swerve.sign[i] = 1;
							}
						}

						swerve.finalangdif[i] = swerve.finalang[i]-swerve.finalangprev[i];
						swerve.finalangprev[i] = swerve.finalang[i];
					}

					swerve.fvel[0] = v1 * swerve.sign[0] * max_speed * fabs(cos((swerve.finalangdif[0]) * (M_PI / 180.0))) ;
					swerve.fvel[1] = v2 * swerve.sign[1] * max_speed * fabs(cos((swerve.finalangdif[1]) * (M_PI / 180.0))) ;
					swerve.fvel[2] = v3 * swerve.sign[2] * max_speed * fabs(cos((swerve.finalangdif[2]) * (M_PI / 180.0))) ;
					swerve.fvel[3] = v4 * swerve.sign[3] * max_speed * fabs(cos((swerve.finalangdif[3]) * (M_PI / 180.0))) ;

					//					VESCPDC(swerve.fvel[0], swerve.fvel[1], swerve.fvel[2], swerve.fvel[3], &vesc);
					VESCRPM(swerve.fvel[0]*6720*4, swerve.fvel[1]*6720*4, swerve.fvel[2]*6720*4, swerve.fvel[3]*6720*4, &vesc);


					RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.finalang[0] / 360.0 + swerve.closestzero[0]);
					RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.finalang[1] / 360.0 + swerve.closestzero[1]);
					RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.finalang[2] / 360.0 + swerve.closestzero[2]);
					RBMS_Set_Target_Position(&rbms1, RBMS4, swerve.finalang[3] / 360.0 + swerve.closestzero[3]);

				}
				swerve.getclosestzero = 0;
				swerve.braketimer = 0;
				swerve.returnzero = 1;
			}
			else
			{


					if (swerve.type == FWD_SWERVE)
					{
						swerve.fvel[0] = 0.0;
						swerve.fvel[1] = 0.0;
						swerve.fvel[2] = 0.0;
						swerve.fvel[3] = 0.0;

						if (swerve.braketimer < brake_time)
						{
							swerve.braketimer++;

							VESCCurrBrake(brake_current, &vesc);

							RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.finalang[0] / 360.0 + swerve.closestzero[0]);
							RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.finalang[1] / 360.0 + swerve.closestzero[1]);
							RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.finalang[2] / 360.0 + swerve.closestzero[2]);
							RBMS_Set_Target_Position(&rbms1, RBMS4, swerve.finalang[3] / 360.0 + swerve.closestzero[3]);
						}

						if(++swerve.timeout > 500){
							swerve.timeout = 0;

							if (swerve.returnzero == 1)
							{
								RBMS_Set_Target_Position(&rbms1, RBMS1, 0);
								RBMS_Set_Target_Position(&rbms1, RBMS2, 0);
								RBMS_Set_Target_Position(&rbms1, RBMS3, 0);
								RBMS_Set_Target_Position(&rbms1, RBMS4, 0);

								for(int i = 0; i < 4; i++)
								{
									swerve.ang[i] = 0.0;
									swerve.angprev[i] = 0.0;
									swerve.addvalue[i] = 0.0;
									swerve.fang[i] = 0.0;
									swerve.fangprev[i] = 0.0;
									swerve.finalang[i] = 0.0;
									swerve.finalangprev[i] = 0.0;
									swerve.closestzero[i] = 0.0;
									swerve.state[i] = 0;
								}
							}
							else
							{
								if (swerve.getclosestzero == 1)
								{
									swerve.closestzero[0] = roundf(swerve.finalang[0] / 360.0) + swerve.closestzero[0];
									swerve.closestzero[1] = roundf(swerve.finalang[1] / 360.0) + swerve.closestzero[1];
									swerve.closestzero[2] = roundf(swerve.finalang[2] / 360.0) + swerve.closestzero[2];
									swerve.closestzero[3] = roundf(swerve.finalang[2] / 360.0) + swerve.closestzero[3];
									swerve.getclosestzero = 0;
								}
								RBMS_Set_Target_Position(&rbms1, RBMS4, swerve.closestzero[0]);
								RBMS_Set_Target_Position(&rbms1, RBMS2, swerve.closestzero[1]);
								RBMS_Set_Target_Position(&rbms1, RBMS3, swerve.closestzero[2]);
								RBMS_Set_Target_Position(&rbms1, RBMS1, swerve.closestzero[3]);

								for(int i = 0; i < 4; i++)
								{
									swerve.ang[i] = 0.0;
									swerve.angprev[i] = 0.0;
									swerve.addvalue[i] = 0.0;
									swerve.fang[i] = 0.0;
									swerve.fangprev[i] = 0.0;
									swerve.finalang[i] = 0.0;
									swerve.finalangprev[i] = 0.0;
									swerve.state[i] = 0;
								}
							}

						VESCReleaseMotor(&vesc);
						VESCStop(&vesc);
					}


				}


			}
		}
	}
}


void SwerveEnq(UART_HandleTypeDef* huartx)
{

}

void swerve_enc_init(GPIO_TypeDef * encGPIOx, uint16_t encGPIO_Pin){
	GPIOPinsInit(encGPIOx, encGPIO_Pin, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL);
	uint32_t prior = encGPIO_Pin - 1;
	ExtixInit(encGPIO_Pin, 9, prior, &ExtiPin);
}


void swerve_init(swerve_allign_t* swerve, float swerve_gear_ratio, uint8_t rbmsid, float * enc){
//	swerve_v2.turnmode = unlimitedturn;
	swerve->RBMSID = rbmsid - 1;

	RBMS_Config(&rbms1, swerve->RBMSID, C610, swerve_gear_ratio); //FL
	RBMS_PID_Init(&rbms1);

	swerve->enc = enc;

	PIDSourceInit(&swerve->enc_target_err, &swerve->enc_output, &swerve->enc_align_pid);
	PIDGainInit(0.005, 1.0, 1.0/180.0, 1.0, 5.0, 0.0, 0.001, 60, &swerve->enc_align_pid);

	swerve->swerve_init = 1;
//	PIDGainInit(ts, sat, ke, ku, kp, ki, kd, kn, pid);
}


//credit to Mr. VinCent
void swerve_allign(swerve_allign_t* swerve, GPIO_TypeDef * hallGPIOx, uint16_t hallGPIO_Pin, float offset, float enc_target){


	switch(swerve->alligning_status){
	case SWERVE_CLOCKWISE:
		swerve->alligning_angle1 = 0.0;
		swerve->alligning_angle2 = 0.0;
		swerve->cur_enc_angle = 0.0;
		swerve->enc_output = 0.0;
		swerve->enc_target_err = 0.0;
		swerve->prev_enc_angle = 0.0;
		swerve->turns = 0;
		swerve->enc_target = enc_target;

		rbms1.motor[swerve->RBMSID].config.vel_limit = 500; //high speed will caz detector didnt detect
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);

		if (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, -500.0);
			while (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
				RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, -500.0);
				swerve->alligning_angle2 = rbms1.motor[swerve->RBMSID].pos;

			}
		}
		RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0.0);
		swerve->alligning_status = SWERVE_ANTICLOCKWISE;
		break;

	case SWERVE_ANTICLOCKWISE:
		osDelay(100);
		if (fabs(swerve->alligning_angle2) < 0.45) { //Mr V use 0.25 caz he got 2 hall XD
			swerve->alligning_angle1 = (swerve->alligning_angle2 - rbms1.motor[swerve->RBMSID].pos);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, swerve->alligning_angle1);
			osDelay(250);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			swerve->alligning_status = SWERVE_ALLIGN_ENC;
		}

		else {
//			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 500.0);
			osDelay(100); //uncomment if ur rbms speed limit is slow

			while (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
				RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 500.0);
				swerve->alligning_angle2 = rbms1.motor[swerve->RBMSID].pos;
			}

			swerve->alligning_angle1 = (swerve->alligning_angle2 - rbms1.motor[swerve->RBMSID].pos);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, swerve->alligning_angle1);
			osDelay(250);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			swerve->alligning_status = SWERVE_ALLIGN_ENC;

			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0.0);
			osDelay(100);

		}
		break;



	case SWERVE_ALLIGN_ENC:
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);

		uint32_t swervecount = 0;
		while((swervecount<500000 || HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) || swerve->enc_target_err > 0.5){
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, swerve->enc_output*100.0);
			swervecount++;
		}
		RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0);
		osDelay(100);
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
		osDelay(100);

		//if alignment fail, it will keep restart.
		if(HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1){
			swerve->alligning_status = SWERVE_CLOCKWISE;
		}else{
			rbms1.motor[swerve->RBMSID].config.vel_limit = 500;
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, offset);
			osDelay(1000);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			osDelay(500);
			swerve->alligning_status = SWERVE_ALLIGNED;

		}

//		swerve->alligning_status = SWERVE_ALLIGNED;


		break;

	case SWERVE_ALLIGNED:


		break;
	}
}

void SwerveCalcAngle(swerve_allign_t* swerve){
//	if(swerve == &swerveA){PWMEncoder_Angle_Update(&enc1); swerve->raw_enc_angle = enc1.Angle;}
//	if(swerve == &swerveB){PWMEncoder_Angle_Update(&enc2); swerve->raw_enc_angle = enc2.Angle;}
//	if(swerve == &swerveC){PWMEncoder_Angle_Update(&enc3); swerve->raw_enc_angle = enc3.Angle;}
//	if(swerve == &swerveD){PWMEncoder_Angle_Update(&enc4); swerve->raw_enc_angle = enc4.Angle;}

//	PWMEncoder_Angle_Update(swerve->enc);
	swerve->raw_enc_angle = *(swerve->enc);


	if (swerve->swerve_init == 1) {

		/*
		 * The value after correction too weird, sus the magnetic enc jumping, need KF
		 *
		 if (swerve->raw_enc_angle - swerve->prev_enc_angle < -350) {
		 swerve->turns++;
		 } else if (swerve->raw_enc_angle - swerve->prev_enc_angle > 350) {
		 swerve->turns--;
		 }
		 swerve->cur_enc_angle = (float)swerve->turns * 360.0 + swerve->raw_enc_angle;
		 swerve->prev_enc_angle = swerve->raw_enc_angle;


		 */

		swerve->cur_enc_angle = swerve->raw_enc_angle;
		swerve->enc_target_err = swerve->enc_target - swerve->cur_enc_angle;
		PID(&swerve->enc_align_pid);
	}
}
