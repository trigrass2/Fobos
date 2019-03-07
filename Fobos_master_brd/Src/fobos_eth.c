/*
 * fobos_eth.c
 *
 *  Created on: 4 ���. 2019 �.
 *      Author: zilkov
 */
#include "fobos_eth.h"
#include "Universal_controller_PC_protocol.h"
#include "ethernet/socket.h"
#include "ethernet/w5200.h"
#include "ethernet/wizchip_conf.h"


uint8_t wiznet820_read_data(SPI_HandleTypeDef *hspi, unsigned short address);
void wiznet820_send_data(SPI_HandleTypeDef *hspi, unsigned short address, uint8_t data_value);
void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf);
void eth_cmds_analysis(fobos_protocol_buf_u *fobos_eth_buf);
uint8_t confirmation(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy);//ethernet params confirmation
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
static uint8_t source_hardware_adr[6] = {0x00,0x08,0xDC,0x01,0x02,0x03};
static uint8_t ip_source_adr[4] = {192,168,100,1};
static uint8_t ip_destination_adr[4] = {192,168,100,2};
static uint32_t timeout_period = 1000;
void EthernetTask_func(void const * argument)
{
  /* USER CODE BEGIN 5 */


	PIN_PWDN(RESET);
	PIN_nRESET(SET);
	PIN_nCS1(SET);
	//osDelay(200);
	vTaskDelay(200);

	setSUBR(subnet_mask_adr);
	//setGAR(ip_gateway_adr);
	//setSHAR(source_hardware_adr);
	setSIPR(ip_source_adr);

	//setsockopt(SOCKET0, SO_DESTPORT, &socket_port);
	//setsockopt(SOCKET0, SO_DESTIP, ip_destination_adr);
	setRTR(timeout_period);
	extern IWDG_HandleTypeDef hiwdg1;
  /* Infinite loop */
  for(;;)
  {
	  HAL_IWDG_Refresh(&hiwdg1);
	  if(!PIN_nINT){
		  LED_VD1(SET);
		  disconnect(SOCKET0);
		  setSn_IR(SOCKET0, getSn_IR(SOCKET0));
	  }
	  switch (getSn_SR(SOCKET0)){
	  case SOCK_ESTABLISHED:
	  {
		  LED_VD2(SET);
		  fobos_protocol_buf_u fobos_eth_buf;
			recv(SOCKET0,fobos_eth_buf.data_to_transmit, 258);
			LED_VD1(SET);
			eth_cmds_analysis(&fobos_eth_buf);
			LED_VD1(RESET);
	  }
		  break;
	  case SOCK_CLOSE_WAIT:
		  disconnect(SOCKET0);
		  LED_VD2(RESET);
		  break;
	  case SOCK_CLOSED:
		  socket(SOCKET0,Sn_MR_TCP,socket_port,0x00);
		  LED_VD2(RESET);
		  break;
	  case SOCK_INIT:
		  listen(SOCKET0);
		  LED_VD2(RESET);
		  break;
	  }
	  vTaskDelay(4);
  /* USER CODE END 5 */
}
}

void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf){//������ ������ � [0]=0!
	fobos_eth_buf->fobos_protocol_buf_t.CMD = CMD;
	fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = bytes_in_packet_N;
	for(int i=0; i<bytes_in_packet_N; i++)
	fobos_eth_buf->fobos_protocol_buf_t.data[i] = (uint8_t)(rand()%7);
	fobos_eth_buf->fobos_protocol_buf_t.data[0]=0;//!!!@@@###
	send(SOCKET0,fobos_eth_buf->data_to_transmit,bytes_in_packet_N+2);
}


void eth_cmds_analysis(fobos_protocol_buf_u *fobos_eth_buf){
	switch(fobos_eth_buf->fobos_protocol_buf_t.CMD)
	{
	case FOBOS_ETH_ECHO:
			//fobos_eth_protocol_send(FOBOS_ETH_ECHO, getSn_RX_RSR(SOCKET0), fobos_eth_buf);
		send(SOCKET0,
			fobos_eth_buf->data_to_transmit,
			fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
		break;
	case  FOBOS_ETH_RST:
		fobos_eth_protocol_send(FOBOS_ETH_RST, 1, fobos_eth_buf);
		while(1);
		break;
	case FOBOS_SENSORS_STATE:
		fobos_eth_protocol_send(FOBOS_SENSORS_STATE, 4, fobos_eth_buf);
		break;
	case FOBOS_GENERATOR_STATE:
		fobos_eth_protocol_send(FOBOS_GENERATOR_STATE, 2, fobos_eth_buf);
		break;
	case FOBOS_SERVOMOTOR_PLACEMENT:
		fobos_eth_protocol_send(FOBOS_SERVOMOTOR_PLACEMENT, 4, fobos_eth_buf);
		break;
	case FOBOS_STATEMENT:
		fobos_eth_protocol_send(FOBOS_STATEMENT, 2, fobos_eth_buf);
		break;
	case FOBOS_CMD_BASING_SERVO:
		fobos_eth_protocol_send(FOBOS_CMD_BASING_SERVO, 1, fobos_eth_buf);
		break;
	case FOBOS_CMD_WORK:
		fobos_eth_protocol_send(FOBOS_CMD_WORK, 1, fobos_eth_buf);
		break;
	case FOBOS_CMD_BARRIER:
		fobos_eth_protocol_send(FOBOS_CMD_BARRIER, 2, fobos_eth_buf);
		break;
	case FOBOS_EMB_SOFT_VER:
	{
		char string_data[] = {"Fobos embedded software version 0.1"};
		int length = strlen(string_data)+1;
		fobos_eth_buf->fobos_protocol_buf_t.CMD = FOBOS_EMB_SOFT_VER;
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		memcpy(fobos_eth_buf->fobos_protocol_buf_t.data+1, string_data, length);
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = length;
		send(SOCKET0,fobos_eth_buf->data_to_transmit,length+2);
	}
		break;
	case FOBOS_ETH_CHANGE_IP:
		if(confirmation(fobos_eth_buf, ip_source_adr))
			setSIPR(ip_source_adr);
		break;
	case FOBOS_ETH_CHANGE_MASK:
		if(confirmation(fobos_eth_buf, subnet_mask_adr))
			setSUBR(subnet_mask_adr);
		break;
	case FOBOS_ETH_CHANGE_PORT:
		if(confirmation(fobos_eth_buf, &socket_port))
					setSUBR(subnet_mask_adr);
		break;
	case FOBOS_CHANGE_TIMEOUT:
		if(confirmation(fobos_eth_buf, &timeout_period))
			setRTR(timeout_period);
		break;
	}
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