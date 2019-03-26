/*
 * fobos_eth.h
 *
 *  Created on: 4 ���. 2019 �.
 *      Author: zilkov
 */

#ifndef FOBOS_ETH_H_
#define FOBOS_ETH_H_

#define DATA_READ_CMD 	0x00
#define DATA_WRITE_CMD	0x80

#define PIN_nINT		 	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)
#define PIN_nCS1(state) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, state)
#define PIN_PWDN(state) 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, state)
#define PIN_nRESET(state) 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, state)

#define LED_VD1(state)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, state)
#define LED_VD2(state)		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, state)

#define SOCKET_NMBR		0
#define SOCKET0 		0
#define SOCKET1 		1
#define SOCKET5 		5

void EthernetTask_func(void const * argument);
#endif /* FOBOS_ETH_H_ */
