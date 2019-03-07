/*
 * StartDefaultTask_FreeRTOS.h
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: zilkov
 */

#ifndef STARTDEFAULTTASK_FREERTOS_H_
#define STARTDEFAULTTASK_FREERTOS_H_

#include "own_defines.h"
#include "adani_can_medical_protocol.h"
#include "cmsis_os.h"
#include "main.h"

extern IWDG_HandleTypeDef hiwdg1;
void StartDefaultTask(void const * argument);
void ADC_Task(void const * argument);

#endif /* STARTDEFAULTTASK_FREERTOS_H_ */
