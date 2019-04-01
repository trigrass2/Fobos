/*
 * own_defines.h
 *
 *  Created on: 7 дек. 2018 г.
 *      Author: zilkov
 */
#include "stm32h7xx_hal.h"

#ifndef OWN_DEFINES_H_
#define OWN_DEFINES_H_

#define LED_VD5(state)		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, state)
#define LED_VD6(state)		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, state)
#define LED_VD7(state)		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, state)

#define SPI_SS_ADC(state)			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, state); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, !state)
#define SPI_SS_ISO_BRD_EN(state)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, state)
#define SPI_SSI_nRE(state)			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, state)
#define SPI_SSI_DE(state)			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, state)

#define DIG_OUT1(state) 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, state)		//X30
#define DIG_OUT2(state) 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, state)		//X31
#define DIG_OUT3(state) 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, state)	//X32
#define DIG_OUT4(state) 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, state)	//X33

#define DIG_OUT1_READ		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)
#define DIG_OUT2_READ		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)
#define DIG_OUT3_READ		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)
#define DIG_OUT4_READ		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)

#define DIG_IN1 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)				//X34
#define DIG_IN2 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)				//X35
#define DIG_IN3 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)				//X36
#define DIG_IN4 			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)				//X37
#define DIG_IN5 			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)				//X38
#define DIG_IN6 			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)				//X39
#define DIG_IN7 			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)				//X40
#define DIG_IN8 			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)				//X41

struct queue_t{
	uint8_t data[8];
};

uint32_t check_adr_func();
char can_tx_func(FDCAN_HandleTypeDef *hfdcan, unsigned int ID, uint32_t data_lenght, uint8_t *data, uint32_t can_buf_num);

#endif /* OWN_DEFINES_H_ */
