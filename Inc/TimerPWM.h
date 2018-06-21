/*
 * TimerPWM.h
 *
 *  Created on: 19 thg 12, 2017
 *      Author: Minh Tien
 */

#ifndef APPLICATION_USER_TIMERPWM_H_
#define APPLICATION_USER_TIMERPWM_H_

#include "stm32f4xx_hal.h"

void MX_GPIO_Init(void);
void MX_TIM3_Init(void);
void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


#endif /* APPLICATION_USER_TIMERPWM_H_ */
