/*
 * BoardComTask_FreeRTOS.h
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: zilkov
 */

#ifndef BOARDCOMTASK_FREERTOS_H_
#define BOARDCOMTASK_FREERTOS_H_

#include "own_defines.h"
#include "adani_can_medical_protocol.h"
#include "cmsis_os.h"
#include "main.h"

extern IWDG_HandleTypeDef hiwdg1;
extern SemaphoreHandle_t xSemaphore_dig_pins, xSemaphore_rs485;

extern  QueueHandle_t 	xQueue_digital_ports, xQueue_digital_ports_fb,
						xQueue_rs485_tx_to_device, xQueue_rs485_rx_from_device,
						xQueue_digital_ports_2,
						xQueue_to_adc_func, xQueue_from_adc_func;

extern osThreadId rs485_com_rxHandle;
extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart6;
extern xSemaphoreHandle xSemaphore_usb_mutex, xSemaphore_rs485_mutex;

void board_com_task(void const * argument);
void rs485_tx_task(void const * argument);
void rs485_rx_task(void const * argument);
#define RS485_PACKET_SIZE	21
typedef struct{
	uint8_t data[RS485_PACKET_SIZE]; 	//посылка, описывается в DATA1..7
	int		lenght;						//длина посылки, описывается в DATA0
}rs485_buf_t;

typedef struct digIO{
	uint8_t digital_outputs; 	//посылка, описывается в DATA1..7
	uint8_t digital_inputs;		//длина посылки, описывается в DATA0
}digital_ports;



#endif /* BOARDCOMTASK_FREERTOS_H_ */
