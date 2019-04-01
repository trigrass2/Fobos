/*
 * StartDefaultTask_FreeRTOS.c
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: Zilkov
 */
#include "own_defines.h"
#include "adani_can_medical_protocol.h"
#include "StartDefaultTask_FreeRTOS.h"
#include "BoardComTask_FreeRTOS.h"
#include "usb_device.h"
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
extern SPI_HandleTypeDef hspi4;

void adc_measure(uint8_t address){
	SPI_SS_ADC(RESET);
	uint8_t adc_measure_buf[3] = {0};
	uint8_t spi_transmit = 0x08;
	uint16_t adc_buf = 0, adc_buf_result = 0;

#define MEASURE_ITERATIONS	20
	HAL_SPI_Transmit(&hspi4, &spi_transmit, 1, 5);
	HAL_SPI_Receive(&hspi4, adc_measure_buf, 3, 5);
	adc_measure_buf[1] <<= 4;
	adc_measure_buf[1] |= adc_measure_buf[2];
	SPI_SS_ADC(SET);
	can_tx_func(&hfdcan2, address + 0x60 + 0x600 + 0x100, 2, adc_measure_buf, FDCAN_TX_BUFFER3);
}

/* USER CODE END Header_StartDefaultTask */

void ADC_Task(void const * argument)
{
  /* USER CODE BEGIN ADC_Task */
	vTaskDelay(1);
	uint8_t adc_buf[3] = {0};
	int address;
	address = check_adr_func();
	uint16_t adc_func_params = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(xQueue_to_adc_func, &adc_func_params, portMAX_DELAY)){
		  taskENTER_CRITICAL();
		  adc_measure(address);
		  taskEXIT_CRITICAL();
	  }
	  taskYIELD();
  }
  /* USER CODE END ADC_Task */
}

void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  volatile digital_ports ports;
  int address;
  address = check_adr_func();
  osDelay(5);
  SPI_SS_ADC(SET);
  uint8_t spi_transmit[2] = {0x08, 0x01}, spi_receive[3] = {0};

  //SPI_SS_ISO_BRD_EN(SET);
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(xQueue_digital_ports, &ports, portMAX_DELAY))
	  {
			DIG_OUT1(ports.digital_outputs & 1);
			DIG_OUT2(ports.digital_outputs & 2);
			DIG_OUT3(ports.digital_outputs & 4);
			DIG_OUT4(ports.digital_outputs & 8);
			ports.digital_outputs = 0;
			ports.digital_inputs = 0;
#define QUANTITY_ITERATIONS		200
			unsigned char checking_digOut_fb[QUANTITY_ITERATIONS] = {0}, checking_digIn_fb[QUANTITY_ITERATIONS] = {0}; // dig out FB [0], dig in FB [1]
			for(int i=0; i<QUANTITY_ITERATIONS; i++)
			{
				//osDelay(1);
				checking_digIn_fb[i] = 0;
				checking_digOut_fb[i] = 0;
				ports.digital_outputs = (DIG_OUT4_READ << 3) | (DIG_OUT3_READ << 2)
										| (DIG_OUT2_READ << 1) | DIG_OUT1_READ;

				ports.digital_inputs = (DIG_IN8<<7)|(DIG_IN7<<6)|(DIG_IN6<<5)|(DIG_IN5<<4)
										|(DIG_IN4<<3)|(DIG_IN3<<2)|(DIG_IN2<<1)|(DIG_IN1);

				checking_digOut_fb[i] |= ports.digital_outputs;
				checking_digIn_fb[i]  |= ports.digital_inputs;

				checking_digOut_fb[0] &= checking_digOut_fb[i];
				checking_digIn_fb[0]  &= checking_digIn_fb[i];
			}

			if(checking_digIn_fb[0] != ports.digital_inputs)
				ports.digital_inputs = 0xFF;

			if(checking_digIn_fb[0] != ports.digital_inputs)
				ports.digital_inputs = 0xFF;
			uint8_t tx_data[2];
			tx_data[0] = ports.digital_outputs;
			tx_data[1] = ports.digital_inputs;
			can_tx_func(&hfdcan2, ((address&0x1F)+0x100+0x620), 2, tx_data, FDCAN_TX_BUFFER1);
			static char a=1;
			LED_VD5(a^=1);
	  }//*/
	  taskYIELD();

  }
  /* USER CODE END 5 */
}
