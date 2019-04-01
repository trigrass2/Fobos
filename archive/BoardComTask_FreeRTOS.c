/*
 * BoardComTask_FreeRTOS.c
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: Zilkov
 */
/* USER CODE BEGIN Header_board_com_task */
/**
* @brief Function implementing the board_com thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_board_com_task */
#include "BoardComTask_FreeRTOS.h"

char can_tx_func(FDCAN_HandleTypeDef *hfdcan, unsigned int ID, uint32_t data_lenght, uint8_t *data, uint32_t can_buf_num)
{
	/* Prepare Tx Header */
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = ID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = (data_lenght<<16);
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_TX_EVENT;
	TxHeader.MessageMarker = 0;

	HAL_FDCAN_AddMessageToTxBuffer(hfdcan, &TxHeader, data, can_buf_num);
	HAL_FDCAN_EnableTxBufferRequest(hfdcan, can_buf_num);
	  return 0xFF;
}

uint32_t check_adr_func(){
	uint32_t address_buf=0;
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))//1
		address_buf |= 1;
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))//2
			address_buf |= 2;
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))//3
			address_buf |= 4;
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15))//4
			address_buf |= 8;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))//5
			address_buf |= 16;
	/*if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))//6
			address_buf += 6;*/
	return address_buf;
}

void FDCAN_Config(uint32_t adr)
{
  /* Configure Rx filter */
	FDCAN_FilterTypeDef sFilterConfig, sFilterConfig1;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	sFilterConfig.FilterID1 = adr;
	sFilterConfig.FilterID2 = 0x01F;

	sFilterConfig1.IdType = FDCAN_STANDARD_ID;
	sFilterConfig1.FilterIndex = 2;
	sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
	sFilterConfig1.FilterID1 = 0x080;
	sFilterConfig1.FilterID2 = 0x0FF;
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig1) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
}

void rs485_tx_task(void const * argument){
	/* USER CODE BEGIN rs485_tx_task */
	vTaskDelay(5);
	rs485_buf_t buf_tx = {{0}, 0};
	int32_t CAN_ID = check_adr_func();
	char led_state=0;
	for(;;){
		if(xQueueReceive(xQueue_rs485_tx_to_device, &buf_tx, portMAX_DELAY)){
			LED_VD7(led_state^=1);
			taskENTER_CRITICAL();
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
			if(HAL_UART_Transmit(&huart6, buf_tx.data, buf_tx.lenght, buf_tx.lenght*10) == HAL_OK)
				buf_tx.data[0] = 0;
			else
				buf_tx.data[0] = 0xFF;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
			taskEXIT_CRITICAL();
			can_tx_func(&hfdcan2, CAN_ID + 0x100 + 0x660, 1, buf_tx.data, FDCAN_TX_BUFFER0);
			for(int i=0; i<buf_tx.lenght; i++)
			buf_tx.data[i] = 0;
			buf_tx.lenght = 0;
		}
	}
	/* USER CODE END rs485_tx_task */
}
static volatile int rs_485_length = 8;
void rs485_rx_task(void const * argument)
{
  /* USER CODE BEGIN rs485_rx_task */
	//standart baudrate is 9600
	int32_t CAN_ID = check_adr_func();
	//standart rs485 length. Must be less than 21!!!
	volatile rs485_buf_t buf_rx = {{0}, rs_485_length};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
  /* Infinite loop */
  for(;;)
  {
	  if(buf_rx.lenght != rs_485_length)
		  buf_rx.lenght = rs_485_length;

	  if(HAL_UART_Receive(&huart6, buf_rx.data, buf_rx.lenght, 100) == HAL_OK)
	  {
		  {static a=0;
		  LED_VD7(a^=1);}

		  {
		  		  if(buf_rx.lenght <= 7 && buf_rx.lenght > 0)
		  		  {
		  			  uint8_t buf[buf_rx.lenght];
		  			  buf[0] = buf_rx.lenght;
		  			  for(int i=1; i <= buf_rx.lenght; i++)
		  				  buf[i] = buf_rx.data[i-1];
		  			  can_tx_func(&hfdcan2, 0x7A0+CAN_ID, 8, buf, FDCAN_TX_BUFFER0);
		  		  }
		  		  else if(buf_rx.lenght > 7)
		  		  {
		  			volatile int count_packs = 1+(buf_rx.lenght/7), count_pack_num = 0;
		  			volatile uint8_t can_tx_data[count_packs][8];
		  			  for(int i=0; count_pack_num < count_packs; i++)
		  			  {
		  				  if(i==0)
		  				  {
		  					  can_tx_data[count_pack_num][i++] = buf_rx.lenght | ((uint8_t)count_pack_num << 4);
		  				  }

		  				  can_tx_data[count_pack_num][i] = buf_rx.data[i + count_pack_num * 8 - (count_pack_num + 1)];

		  				  if(i==7)
		  				  {
		  					  i = -1;
		  					  count_pack_num++;
		  				  }
		  			  }
		  			  count_pack_num = 0;
		  			  uint32_t can_buf_num = FDCAN_TX_BUFFER0;
		  			  while(count_pack_num < count_packs)
		  			  {
		  				  can_tx_func(&hfdcan2, 0x6A0+0x100+CAN_ID, 8, can_tx_data[count_pack_num], can_buf_num);
		  				  can_buf_num <<= 1;
		  				  count_pack_num++;
		  			  }
		  		  }
		  	  }
	  }
	  vTaskSuspend(rs485_com_rxHandle);
  }
  /* USER CODE END rs485_rx_task */
}

#define CANOPEN_4BYTES_TX	0x43
#define CANOPEN_CMD_SYNC	0x1005
void board_com_task(void const * argument)//communicates with CAN-bus
{
  /* USER CODE BEGIN board_com_task */
	FDCAN_RxHeaderTypeDef RxHeader, RxHeader_rst_msg;
	RxHeader_rst_msg.FilterIndex = 1;
	uint8_t Rx_Can_Data[8] = {0};
	*Rx_Can_Data = 1;

	uint8_t Rx_Can_Data1[8] = {0};

	rs485_buf_t buf_tx = {{0},0};
	int32_t CAN_ID = check_adr_func();
	FDCAN_Config(CAN_ID);
	{
		char adr_buf = (uint8_t)CAN_ID;
		uint8_t buf[2] = {adr_buf, '1'};
		can_tx_func(&hfdcan2, 0, 2, &buf[0], FDCAN_TX_BUFFER0);
	}
	volatile digital_ports ports;
extern UART_HandleTypeDef huart6;
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader_rst_msg, Rx_Can_Data1) == HAL_OK){
		  uint32_t DataLength = RxHeader_rst_msg.DataLength >> 16;
		  //uint32_t address = RxHeader.Identifier;
		  if(DataLength == 8 && Rx_Can_Data1[0] == CANOPEN_4BYTES_TX)//0x43 - 4 байта передачи в CANopen
		  {
			  int cmd = Rx_Can_Data1[2]<<8 | Rx_Can_Data1[1];
			  if(cmd == CANOPEN_CMD_SYNC)
			  HAL_IWDG_Refresh(&hiwdg1);
		  }

	  }

	  if(can_protocol_data_analyzing(&hfdcan2, &RxHeader, Rx_Can_Data))
	  {

		  uint32_t address = RxHeader.Identifier;
		  uint32_t DataLength = RxHeader.DataLength >> 16;
			switch(address & 0x7E0)
			{
			case 0x620://Dig pins
			{
				if(DataLength)
				{
					ports.digital_inputs = 0;
					ports.digital_outputs = Rx_Can_Data[0];
					xQueueSendToFront(xQueue_digital_ports, &ports, 5);
				}
				else
				{
						uint8_t tx_data[2] = {0};//data0 - digout, data1 - digin pins
						tx_data[0] = (DIG_OUT4_READ << 3) | (DIG_OUT2_READ << 2)
									| (DIG_OUT2_READ << 1) | DIG_OUT1_READ;

						tx_data[1] = (DIG_IN8<<7)|(DIG_IN7<<6)|(DIG_IN6<<5)|(DIG_IN5<<4)
									|(DIG_IN4<<3)|(DIG_IN3<<2)|(DIG_IN2<<1)|(DIG_IN1);

						can_tx_func(&hfdcan2, ((address&0x1F)+0x620+0x100), 2, tx_data, FDCAN_TX_BUFFER4);
				}
			}
				break;

			case 0x640://rs485 tx to device
			{
				buf_tx.lenght = Rx_Can_Data[0] & 0x1F;
				if(buf_tx.lenght > 0)
				{
					if(Rx_Can_Data[0] == 0xF0){//если DATA0 = 0xF0, то изменить длину принимаемой посылки по rs485 на DATA1 (<21)
						uint8_t tx_data[2] = {0};
						taskENTER_CRITICAL();
						if(Rx_Can_Data[1] <= RS485_PACKET_SIZE){
							rs_485_length = Rx_Can_Data[1];
							tx_data[0] = 0;
							tx_data[1] = (uint8_t)rs_485_length;
						}
						else{
							tx_data[0] = 0xF0;
							tx_data[1] = 0;
						}
						taskEXIT_CRITICAL();

						can_tx_func(&hfdcan2, ((address&0x1F)+0x640+0x100), 2, tx_data, FDCAN_TX_BUFFER4);
					}
					else if(buf_tx.lenght <= 7)
					{
						memcpy(buf_tx.data, &Rx_Can_Data[1], buf_tx.lenght);
						xQueueSend(xQueue_rs485_tx_to_device, &buf_tx, 5);
					}
					else if(buf_tx.lenght > 7 && buf_tx.lenght <= RS485_PACKET_SIZE)
					{
						static int cnt_bytes = 0;
						memcpy(&buf_tx.data[cnt_bytes*7], &Rx_Can_Data[1], 7);
						cnt_bytes++;
						if((cnt_bytes * 7) >= buf_tx.lenght)
						{
							xQueueSend(xQueue_rs485_tx_to_device, &buf_tx, 10);
							cnt_bytes = 0;
						}
					}
				}
			}//case 0x640 ends
			break;

			case 0x660://ADC request measure
			{
				uint16_t data_to_adc = 0;
				xQueueSend(xQueue_to_adc_func, &data_to_adc, portMAX_DELAY);
			}
			break;

			case 0x720:
			{
				ports.digital_inputs = Rx_Can_Data[1];
				xQueueSend(xQueue_digital_ports_2, &ports, 10);
			}//0x720
			break;
		}//switch
	  }
	  vTaskResume(rs485_com_rxHandle);
  }
}

  /* USER CODE END board_com_task */


