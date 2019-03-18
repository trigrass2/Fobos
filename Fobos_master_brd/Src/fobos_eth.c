/*
 * fobos_eth.c
 *
 *  Created on: 4 мар. 2019 г.
 *      Author: zilkov
 */
#include "fobos_eth.h"
#include "Universal_controller_PC_protocol.h"
#include "ethernet/socket.h"
#include "ethernet/w5200.h"
#include "ethernet/wizchip_conf.h"
#include "own_defines.h"
#include "adani_can_medical_protocol.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "internet/dhcp.h"

uint8_t wiznet820_read_data(SPI_HandleTypeDef *hspi, unsigned short address);
void wiznet820_send_data(SPI_HandleTypeDef *hspi, unsigned short address, uint8_t data_value);
void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf);
void eth_cmds_analysis(fobos_protocol_buf_u *fobos_eth_buf);
void feedback_params(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy, uint8_t tx_bytes);
uint8_t confirmation(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy);//ethernet params confirmation
void vTimerCallback(TimerHandle_t);
/* USER CODE BEGIN Header_EthernetTask_func */
/**
  * @brief  Function implementing the EthTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_EthernetTask_func */
static uint16_t socket_port = 15000;
static uint8_t ip_gateway_adr[4] = {192,168,100,2};
static uint8_t subnet_mask_adr[4] = {255,255,255,0};
static uint8_t source_hardware_adr[6] = {0x5A,0x69,0x4C,0x6B,0x6F,0x76};
static uint8_t ip_source_adr[4] = {192,168,100,1};
//static uint8_t ip_source_adr[4] = {192,168,101,170};
static uint8_t ip_destination_adr[4] = {192,168,100,2};
static uint32_t timeout_period = 1000;
static wiz_NetInfo wiz_NetData;

typedef void * TimerHandle_t;
uint8_t dhcp_buf[580] = {0};
void EthernetTask_func(void const * argument)
{
  /* USER CODE BEGIN 5 */


	PIN_PWDN(RESET);
	PIN_nRESET(SET);
	PIN_nCS1(SET);
	//osDelay(200);
	vTaskDelay(200);

	wiz_NetData.dhcp = 2;
	memcpy(wiz_NetData.sn, subnet_mask_adr, 4);
	memcpy(wiz_NetData.ip, ip_source_adr, 4);
	memcpy(wiz_NetData.mac, source_hardware_adr,6);
	wizchip_setnetinfo(&wiz_NetData);

	setsockopt(SOCKET0, SO_DESTPORT, &socket_port);
	setsockopt(SOCKET0, SO_DESTIP, ip_destination_adr);
	setRTR(timeout_period);

	//DHCP>>>>
	taskENTER_CRITICAL();
	DHCP_init(SOCKET5, dhcp_buf);
	taskEXIT_CRITICAL();
	//DHCP====

	extern IWDG_HandleTypeDef hiwdg1;

	TimerHandle_t xTimer_period_reset;
	xTimer_period_reset = xTimerCreate("Period timer", 200, pdTRUE, (void*)0, vTimerCallback);
	xTimerStart(xTimer_period_reset, 0);
	vTaskDelay(2000);
  /* Infinite loop */
  for(;;)
  {

	  taskENTER_CRITICAL();
	  if(DHCP_run() == DHCP_IP_LEASED)
	    LED_VD5(RESET);
	  taskEXIT_CRITICAL();
	  LED_VD1(SET);
	  if(!PIN_nINT)
	    {
		  LED_VD1(SET);
		  disconnect(SOCKET0);
		  setSn_IR(SOCKET0, getSn_IR(SOCKET0));
	    }

	  switch (getSn_SR(SOCKET0)){
	  case SOCK_ESTABLISHED:
	  {
	    if(xTimerIsTimerActive(xTimer_period_reset) != pdFALSE)
	    xTimerStop(xTimer_period_reset, 0);
		  LED_VD2(SET);
		  fobos_protocol_buf_u fobos_eth_buf;
		  recv(SOCKET0,fobos_eth_buf.data_to_transmit, 258);
		  LED_VD1(SET);
		  eth_cmds_analysis(&fobos_eth_buf);
		  uint8_t buf[] = {0x43, 0x05, 0x10,0,0,0,0,0};
		  extern FDCAN_HandleTypeDef hfdcan2;
		  can_tx_func(&hfdcan2, 0x80, 8, buf, FDCAN_TX_BUFFER3);
		  LED_VD1(RESET);
	  }
		  break;
	  case SOCK_CLOSE_WAIT:
		  xTimerStart(xTimer_period_reset, 0);
		  disconnect(SOCKET0);
		  LED_VD2(RESET);
		  break;
	  case SOCK_CLOSED:
	    /*if(xTimerIsTimerActive(xTimer_period_reset) == pdFALSE)
	    xTimerStart(xTimer_period_reset, 0);*/
	    socket(SOCKET0,Sn_MR_TCP,socket_port,0x00);
	    LED_VD2(RESET);
		  break;
	  case SOCK_INIT:
	    listen(SOCKET0);
	    LED_VD2(RESET);
		  break;
	  }
	  HAL_IWDG_Refresh(&hiwdg1);
	  vTaskDelay(150);
  /* USER CODE END 5 */
}
}

void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf){//убрать строку с [0]=0!
	fobos_eth_buf->fobos_protocol_buf_t.CMD = CMD;
	fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = bytes_in_packet_N;
	send(SOCKET0,fobos_eth_buf->data_to_transmit,bytes_in_packet_N+2);
}


void eth_cmds_analysis(fobos_protocol_buf_u *fobos_eth_buf){
  extern FDCAN_HandleTypeDef hfdcan2;
	switch(fobos_eth_buf->fobos_protocol_buf_t.CMD)
	{
	case FOBOS_ETH_ECHO:
		send(SOCKET0,
			fobos_eth_buf->data_to_transmit,
			fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
		break;
	case  FOBOS_ETH_GET_MAC:
	  {
	    uint8_t mac_adr[6] = {0};
	    getSHAR(mac_adr);
	    fobos_eth_buf->fobos_protocol_buf_t.CMD = FOBOS_ETH_GET_MAC;
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    memcpy(fobos_eth_buf->fobos_protocol_buf_t.data+1, mac_adr, 6);
	    fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 6+1;
	    fobos_eth_protocol_send(FOBOS_ETH_GET_MAC, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
	  }
	  break;
	case  FOBOS_ETH_RST:
		fobos_eth_protocol_send(FOBOS_ETH_RST, 1, fobos_eth_buf);
		while(1);
		break;
	case FOBOS_SENSORS_STATE:
		{
			uint8_t sensors_state = 0, temp_lim_switches = 0;
			can_tx_func(&hfdcan2, 0x620+2, 0, &sensors_state, FDCAN_TX_BUFFER1);
			uint8_t can_data_buf[8] = {0};
			fobos_eth_buf->fobos_protocol_buf_t.data[2] = temp_lim_switches;//DATA2?
			FDCAN_RxHeaderTypeDef RxHeader;
			fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			if(TABLE_LOCK_SENSOR_LEFT)
				sensors_state |= 0x01;
			if(EMERGENCY_LIMIT_SW1)
				sensors_state |= 0x02;
			if(EMERGENCY_LIMIT_SW2)
				sensors_state |= 0x04;
			if(TABLE_LOCK_SENSOR_RIGHT)
				sensors_state |= 0x08;
			fobos_eth_buf->fobos_protocol_buf_t.data[3] = sensors_state;

			while(RxHeader.Identifier != 0x722)
			  can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_buf);
			if(can_data_buf[1] & 0b11000011)
			  temp_lim_switches = can_data_buf[1] & 0b11000011;//S1,S2 ... S4,S3 в соответствии с единицами в байте.
			fobos_eth_buf->fobos_protocol_buf_t.data[1] = temp_lim_switches;
			fobos_eth_protocol_send(FOBOS_SENSORS_STATE, 4, fobos_eth_buf);
		}
		break;
	case FOBOS_GENERATOR_STATE:
	  {
	    uint8_t can_data_buf[8];
	    FDCAN_RxHeaderTypeDef RxHeader;
	    can_tx_func(&hfdcan2, 0x620+2, 0, can_data_buf, FDCAN_TX_BUFFER1);
	    uint8_t interlock_generator_state = 0, interlock_door_and_generator_state = 0;
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;

	    if(DIG_IN8)
	      interlock_door_and_generator_state = 0xFF;

	    while(RxHeader.Identifier != 0x722)
	      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_buf);
	    if(can_data_buf[1] & 0x04)
	      interlock_generator_state = 0xFF;

	    fobos_eth_buf->fobos_protocol_buf_t.data[1] = interlock_generator_state & interlock_door_and_generator_state;

	    fobos_eth_protocol_send(FOBOS_GENERATOR_STATE, 2, fobos_eth_buf);
	  }
		break;
	case FOBOS_SERVOMOTOR_PLACEMENT:
	  {
	    uint8_t can_data_buf[8], temp_lim_switches = 0;
	    FDCAN_RxHeaderTypeDef RxHeader;
	    can_tx_func(&hfdcan2, 0x620+2, 0, can_data_buf, FDCAN_TX_BUFFER1);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    //запрос к двигателю линейного перемещения о положении ->
	    int servo_pos = 1619;//запрос к двигателю о его положении
	    fobos_eth_buf->fobos_protocol_buf_t.data[1] = servo_pos>>8;
	    fobos_eth_buf->fobos_protocol_buf_t.data[2] = servo_pos;
	    if(servo_pos > 100)
	      fobos_eth_buf->fobos_protocol_buf_t.data[3] = 0;
	    else
	      fobos_eth_buf->fobos_protocol_buf_t.data[3] = 0xFF;

	    while(RxHeader.Identifier != 0x720+2)
	      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_buf);
	    if(can_data_buf[1] & 0b11000011)
	      temp_lim_switches = can_data_buf[1] & 0b11000011;//S1,S2 ... S4,S3 в соответствии с единицами в байте.
	    if(temp_lim_switches && temp_lim_switches < 3)
	      fobos_eth_buf->fobos_protocol_buf_t.data[3] = temp_lim_switches;
	    fobos_eth_protocol_send(FOBOS_SERVOMOTOR_PLACEMENT, 4, fobos_eth_buf);
	  }
		break;
	case FOBOS_STATEMENT:
	  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	  fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;			//не готово устройство
	  fobos_eth_protocol_send(FOBOS_STATEMENT, 2, fobos_eth_buf);
		break;
	case FOBOS_CMD_BASING:
	  {
	    //поворот в обратную сторону (к нижнему концевику)
	    uint8_t can_data_buf[16] = {0x08, 0x05, 0x4B, 0x08, 0x00, 0x83, 0xDE, 0x00, 0x08, 0x47}, temp_lim_switches = 0;
	    FDCAN_RxHeaderTypeDef RxHeader;
	    can_tx_func(&hfdcan2, 0x640+2, 8, &can_data_buf[0], FDCAN_TX_BUFFER1);
	    can_tx_func(&hfdcan2, 0x640+2, 8, &can_data_buf[8], FDCAN_TX_BUFFER2);
	    while(RxHeader.Identifier != 0x740+2)
	      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_buf);
	    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	    //запрос к линейному приводу о положении
	    //uint8_t motor_data_can_buf[8] = {0x43, 0x7A, 0x60, 0x00, 0,0,0,0};//not sure
	    //can_tx_func(&hfdcan2, 0x280+1??, 8, motor_data_can_buf, FDCAN_TX_BUFFER1);
	    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	    fobos_eth_protocol_send(FOBOS_CMD_BASING, 1, fobos_eth_buf);
	  }
		break;
	case FOBOS_CMD_WORK:
	  {
	    uint8_t can_data_buf[16] = {0x08, 0x05, 0x4B, 0x08, 0x00, 0x83, 0xDE, 0x00, 0x08, 0x47}, temp_lim_switches = 0;
	    FDCAN_RxHeaderTypeDef RxHeader;
	    if(fobos_eth_buf->fobos_protocol_buf_t.data[1] && fobos_eth_buf->fobos_protocol_buf_t.data[1] <= 3){
		//передать значения в другой поток
	    }
	    else
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	    can_tx_func(&hfdcan2, 0x640+2, 8, &can_data_buf[0], FDCAN_TX_BUFFER1);
	    can_tx_func(&hfdcan2, 0x640+2, 8, &can_data_buf[8], FDCAN_TX_BUFFER2);
	    while(RxHeader.Identifier != 0x740+2)
	      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_buf);
	    fobos_eth_protocol_send(FOBOS_CMD_WORK, 1, fobos_eth_buf);
	  }
		break;
	case FOBOS_CMD_BARRIER:
	  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	  fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
	  fobos_eth_protocol_send(FOBOS_CMD_BARRIER, 2, fobos_eth_buf);
		break;
	case FOBOS_EMB_SOFT_VER:
	{
		char string_data[] = {"Fobos embedded software version 0.14"};
		int length = strlen(string_data)+1;
		fobos_eth_buf->fobos_protocol_buf_t.CMD = FOBOS_EMB_SOFT_VER;
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		memcpy(fobos_eth_buf->fobos_protocol_buf_t.data+1, string_data, length);
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = length;
		send(SOCKET0,fobos_eth_buf->data_to_transmit,length+2);
	}
		break;


		//Команды на изменение сетевых параметров устройства
	case FOBOS_ETH_CHANGE_IP:
		if(confirmation(fobos_eth_buf, ip_source_adr))
			setSIPR(ip_source_adr);
		break;
	case FOBOS_ETH_CHANGE_MASK:
		if(confirmation(fobos_eth_buf, subnet_mask_adr))
			setSUBR(subnet_mask_adr);
		break;
	case FOBOS_ETH_CHANGE_PORT:
		socket_port = (fobos_eth_buf->fobos_protocol_buf_t.data[1]<<8)|fobos_eth_buf->fobos_protocol_buf_t.data[2];
		if(confirmation(fobos_eth_buf, &socket_port))
		  setSn_PORT(SOCKET0, socket_port);
		break;
	case FOBOS_CHANGE_TIMEOUT:
		if(confirmation(fobos_eth_buf, &timeout_period))
			setRTR(timeout_period);
		break;

	//GET ethernet statements >>>>
	case FOBOS_ETH_GET_IP:
	  {
	    uint8_t ip[4];
	    getSIPR(ip);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, ip, 4+1);
	  }
	  break;
	case FOBOS_ETH_DHCP:
	  {
	    uint8_t dhcp_state = fobos_eth_buf->fobos_protocol_buf_t.data[1];
	    wiz_NetData.dhcp = dhcp_state;
	    if(dhcp_state == 1 || dhcp_state == 2)
	    {
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		feedback_params(fobos_eth_buf, &dhcp_state, 1+1);
	    }
	    else
	    {
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;
		send(SOCKET0,fobos_eth_buf->data_to_transmit,fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
	    }
	  }
	  break;
	case FOBOS_ETH_GET_MASK:
	  {
	    uint8_t mask[4]={0};
	    getSUBR(mask);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, mask, 4+1);
	  }
	  break;
	case FOBOS_ETH_GET_PORT:
		  {
		    uint8_t port[2]={0};
		    socket_port = getSn_PORT(SOCKET0);
		    port[0] = socket_port>>8;
		    port[1] = socket_port;
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		    feedback_params(fobos_eth_buf, port, 2+1);
		  }
		  break;
	case FOBOS_ETH_GET_TIMEOUT:
		  {
		    uint8_t timeout[2];
		    uint16_t timeout_temp1;
		    timeout_temp1 = getRTR();
		    timeout[0] = timeout_temp1>>8;
		    timeout[1] = timeout_temp1;
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		    feedback_params(fobos_eth_buf, timeout, 2+1);
		  }
		  break;
	case FOBOS_ETH_GET_DHCP_STATE:
	  {
	    uint8_t dhcp_state = 0;
	    dhcp_state = wiz_NetData.dhcp;
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, &dhcp_state, 1+1);
	  }
	  break;
	default:
	  fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;
	  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_CMD;
	  fobos_eth_protocol_send(fobos_eth_buf->fobos_protocol_buf_t.CMD, 1, fobos_eth_buf);
	  break;
	}
}

void feedback_params(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy, uint8_t tx_bytes){
  fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = tx_bytes;
  memcpy(&fobos_eth_buf->fobos_protocol_buf_t.data[1], data_for_copy, tx_bytes-1);
  send(SOCKET0,
      fobos_eth_buf->data_to_transmit,
      fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
}

uint8_t confirmation(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy){
	if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 4)
	{
		memcpy(data_for_copy,
		fobos_eth_buf->fobos_protocol_buf_t.data,
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N);
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;

		send(SOCKET0,
			fobos_eth_buf->data_to_transmit,
			fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
		return 1;
	}
	else if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 2
			&& (fobos_eth_buf->fobos_protocol_buf_t.CMD == FOBOS_ETH_CHANGE_PORT
					|| fobos_eth_buf->fobos_protocol_buf_t.CMD == FOBOS_CHANGE_TIMEOUT)){
				*data_for_copy = fobos_eth_buf->fobos_protocol_buf_t.data[1];
				*(data_for_copy + 1) = fobos_eth_buf->fobos_protocol_buf_t.data[0];

				fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
				fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;

				send(SOCKET0,
					fobos_eth_buf->data_to_transmit,
					fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
				return 1;
	}
	else
	{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;
		send(SOCKET0,
			fobos_eth_buf->data_to_transmit,
			fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
		return 0;
	}
}

uint8_t wiznet820_read_data(SPI_HandleTypeDef *hspi, unsigned short address){
		PIN_nCS1(RESET);
		uint8_t data[5] = {0};
		unsigned short adr, data_lenght = 1;
		uint8_t data_rcv=0;
		adr = address;
		data[0] = (uint8_t)(adr>>8);
		data[1] = (uint8_t)adr;
		data[2] = (uint8_t)(DATA_READ_CMD|(data_lenght>>8));
		data[3] = (uint8_t)data_lenght;
		HAL_SPI_Transmit(hspi, data, 4, 10);
		HAL_SPI_Receive(hspi, &data_rcv, 1, 10);
		PIN_nCS1(SET);
		return data_rcv;
}

void wiznet820_send_data(SPI_HandleTypeDef *hspi, unsigned short address, uint8_t data_value){
	PIN_nCS1(RESET);
	HAL_Delay(10);
	unsigned short adr, data_lenght = 1;
	adr = address;
	uint8_t data[5] = {0};
	data[0] = (uint8_t)(adr>>8);
	data[1] = (uint8_t)adr;
	data[2] = (uint8_t)(DATA_WRITE_CMD|(data_lenght>>8));
	data[3] = (uint8_t)data_lenght;
	data[4] = data_value;
	HAL_SPI_Transmit(hspi, data, 5, 10);
	PIN_nCS1(SET);
}

void vTimerCallback(TimerHandle_t Timer){
  	extern IWDG_HandleTypeDef hiwdg1;
	HAL_IWDG_Refresh(&hiwdg1);
	static a=0;
	LED_VD6(a^=1);
	uint8_t buf[] = {0x43, 0x05, 0x10,0,0,0,0,0};
	extern FDCAN_HandleTypeDef hfdcan2;
	can_tx_func(&hfdcan2, 0x80, 8, buf, FDCAN_TX_BUFFER4);
}
