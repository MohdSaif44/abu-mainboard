/*
 * swerve.h
 *
 *  Created on:
 *      Author:
 */

#ifndef SRC_SWERVE_SWERVE_H_
#define SRC_SWERVE_SWERVE_H_

#include "../VESC_CAN/vesc_interface.h"
#include "../Robomaster/Robomaster.h"
#include "../MODN/MODN.h"
#include "../PID/PID.h"
#include "../EXTI/EXTI.h"

enum{
	FWD_SWERVE,
	TRI_SWERVE,
	manualnolock,
	manuallock,
	perspective,
	pathplan,
	halfturn,
	unlimitedturn,
};


typedef enum{
	SWERVE_CLOCKWISE,
	SWERVE_ANTICLOCKWISE,
	SWERVE_ALLIGN_ENC,
	SWERVE_ALLIGNED,
}swerve_aligning_status;

typedef struct{
	swerve_aligning_status alligning_status;
	uint8_t RBMSID;
	float alligning_angle2;
	float alligning_angle1;
	float enc_target_err;
	float enc_output;
	PID_t enc_align_pid;
	uint8_t swerve_init;
	float enc_target;
	float cur_enc_angle;
	float prev_enc_angle;
	int turns;
	float raw_enc_angle;
	float * enc;

}swerve_allign_t;

struct {
	int type;
	int turnmode;
	int aligned;
	int run;
	int braketimer;
	int returnzero;
	int getclosestzero;
	float sign[4];
	float fvel[4];
	float ang[4];
	float fang[4];
	float angprev[4];
	float fangprev[4];
	float ang1[4];
	float ang2[4];
	int state[4];
	float finalang[4];
	float finalangprev[4];
	float finalangdif[4];
	float addvalue[4];
	float closestzero[4];
	float valuecheck[4];
	int aligndone[4];
	float alignerror[4];
	float angleplus[4];
	int done[4];
	int anothersign[4];
	float rotation;
	float maxturn;
	float aligntolerance;
	float alignspeed;
	float alignangle;
	uint16_t timeout;
} swerve;

/***prototype function***/
swerve_allign_t swerveA, swerveB, swerveC, swerveD;
void swerve_enc_init(GPIO_TypeDef * encGPIOx, uint16_t encGPIO_Pin);
void swerve_init(swerve_allign_t* swerve, float swerve_gear_ratio, uint8_t rbmsid, float * enc);
void swerve_allign(swerve_allign_t* swerve, GPIO_TypeDef * hallGPIOx, uint16_t hallGPIO_Pin, float offset, float enc_target);

void SwerveInit(int swerve_type, int turn_mode, float maximum_turn, float width, float length, float swerve_gear_ratio);
void SwerveRun(float max_speed, int mode, float brake_current, int brake_time, float target_angle);
void SwerveEnq(UART_HandleTypeDef* huartx);

//void SwerveAligninit(uint16_t A_GPIO_Pin, uint16_t B_GPIO_Pin, uint16_t C_GPIO_Pin, uint16_t D_GPIO_Pin);
//void SwerveAlign(float A_angle, float B_angle, float C_angle, float D_angle, GPIO_TypeDef * AhallsGPIOx, uint16_t AhallsGPIO_Pin, GPIO_TypeDef * BhallsGPIOx, uint16_t BhallsGPIO_Pin, GPIO_TypeDef * ChallsGPIOx, uint16_t ChallsGPIO_Pin, GPIO_TypeDef * DhallsGPIOx, uint16_t DhallsGPIO_Pin);

#endif /* SRC_SWERVE_SWERVE_H_ */
