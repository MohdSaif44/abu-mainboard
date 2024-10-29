
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common.h"
#include "../adapter.h"

#define STOP_CONDITION fabs(x_vel) + fabs(y_vel) + fabs(w_vel) < 0.05

void MainTask(void *argument);
void ServoTask (void *argument);
void Calculation(void *argument);

void update_param(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
