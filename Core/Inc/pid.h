/*
 * pid.h
 *
 *  Created on: Nov 13, 2026
 *      Author: Lera
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
void pidRun(I2C_HandleTypeDef* hpi2c, TIM_HandleTypeDef* hptim3, UART_HandleTypeDef* hpuart2);

#endif /* INC_PID_H_ */

