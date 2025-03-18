
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common.h"
#include "../adapter.h"
#include "../MICROROS/microros.h"

#define STOP_CONDITION fabs(x_vel) + fabs(y_vel) + fabs(w_vel) < 0.05
//#define CURRENT_X_POS  -((QEIRead(QEI4)*cos(IMU.real_zrad) + QEIRead(QEI1)*sin(IMU.real_zrad)) * 0.05 * M_PI/ 8192.0)
//#define CURRENT_Y_POS   ((QEIRead(QEI1)*cos(IMU.real_zrad) - QEIRead(QEI4)*sin(IMU.real_zrad)) * 0.05 * M_PI/ 8192.0)
//#define CURRENT_X_POS -(QEIRead(QEI4)* 0.05 * M_PI/ 8192.0)
//#define CURRENT_Y_POS  (QEIRead(QEI1)* 0.05 * M_PI/ 8192.0)

void MainTask(void *argument);
void SecondaryTask (void *argument);
void SwerveATask (void *argument);
void SwerveBTask (void *argument);
void SwerveCTask (void *argument);
void SwerveDTask (void *argument);
void Calculation(void *argument);
void Microros(void *argument);
void subscription1_callback (const void * msgin);

void demo2_pathplan(float target_x, float target_y, float tolerance_x, float tolerance_y);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
