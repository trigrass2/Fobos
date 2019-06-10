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
#include "queue.h"
#include "semphr.h"
#include "internet/dhcp.h"

/*#include "internet/httpServer/httpParser.h"
#include "internet/httpServer/httpServer.h"
#include "internet/httpServer/httpUtil.h"*/

#define	SDO_1BYTE_REQ	0x2F
#define	SDO_2BYTES_REQ	0x2B
#define	SDO_3BYTES_REQ	0x27
#define	SDO_4BYTES_REQ	0x23

#define SDO_REQUEST	0x40

uint8_t wiznet820_read_data(SPI_HandleTypeDef *hspi, unsigned short address);
void wiznet820_send_data(SPI_HandleTypeDef *hspi, unsigned short address, uint8_t data_value);
void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf);
void eth_cmds_analysis(volatile fobos_protocol_buf_u *fobos_eth_buf);
void feedback_params(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy, uint8_t tx_bytes);
uint8_t confirmation(fobos_protocol_buf_u *fobos_eth_buf, uint8_t *data_for_copy);//ethernet params confirmation
void vTimerCallback(TimerHandle_t);
void vFobos_Start_Process();
static void homing_process();
static void position_mode_process(uint8_t );
uint8_t can_protocol_data_analyzing(FDCAN_HandleTypeDef *hfdcan,
				FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData);

/* USER CODE BEGIN Header_EthernetTask_func */
extern FDCAN_HandleTypeDef hfdcan2;
extern IWDG_HandleTypeDef hiwdg1;
/**
  * @brief  Function implementing the EthTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_EthernetTask_func */
static uint16_t socket_port = 15000;
static uint8_t ip_gateway_adr[4] = {192,168,100,2};
static uint8_t ip_dns_adr[4] = {192,168,100,1};
static uint8_t subnet_mask_adr[4] = {255,255,255,0};
static uint8_t source_hardware_adr[6] = {0x00,0x08,0xDC,0x0F,0x00,0x01};
static uint8_t ip_source_adr[4] = {192,168,100,1};
//static uint8_t ip_source_adr[4] = {192,168,101,170};
static uint8_t ip_destination_adr[4] = {192,168,100,2};
static uint32_t timeout_period = 1000;
static wiz_NetInfo wiz_NetData;
volatile static uint8_t C_rack_statement;//положение С-рамы
uint8_t canopen_transmit(uint16_t COB_ID, uint8_t control_field, uint16_t index, uint8_t subindex, uint8_t *data);
static char can_tx_func(FDCAN_HandleTypeDef *hfdcan, unsigned int ID, uint32_t data_lenght, uint8_t *data);
typedef void * TimerHandle_t;
uint8_t dhcp_buf[580] = {0};

static uint8_t motor_state_indication = 0;

typedef union canopen{
  struct{
    uint8_t COB_ID_high;
    uint8_t COB_ID_low;
    uint8_t control_field;
    uint8_t index_low;
    uint8_t index_high;
    uint8_t subindex;
    uint8_t data[4];
  }bytes_t;
  struct{
    uint16_t COB_ID;
    uint8_t control_field;
    uint16_t index;
    uint8_t subindex;
    uint8_t data[4];
  }values_t;
  uint8_t canopen_mass[10];
}canopen_u;
canopen_u* canopen_receive(canopen_u* canopen_rcv);
canopen_u* canopen_req_resp_sdo(uint16_t COB_ID, uint8_t control_field, uint16_t index, uint8_t subindex, uint8_t *data, canopen_u *canopen_rcv);
QueueHandle_t xQueue_Scanning_start = NULL;

SemaphoreHandle_t Mutex_Eth = NULL;

void EthernetTask_func(void const * argument)
{
  /* USER CODE BEGIN 5 */
  PIN_PWDN(RESET);
  PIN_nRESET(SET);
  PIN_nCS1(SET);

  Mutex_Eth = xSemaphoreCreateMutex();
  if(Mutex_Eth == NULL)
    Error_Handler();

  xQueue_Scanning_start = xQueueCreate(1, sizeof(uint8_t));
  if(xQueue_Scanning_start == NULL)
    Error_Handler();
	TimerHandle_t xTimer_period_reset;
  	xTimer_period_reset = xTimerCreate("Period timer", 150, pdTRUE, (void*)0, vTimerCallback);
  	xTimerStart(xTimer_period_reset, 0);
	vTaskDelay(300);

	DIG_OUT3(SET);//ENABLE signal

	wiz_NetData.dhcp = 2;
	memcpy(wiz_NetData.sn, subnet_mask_adr, 4);
	memcpy(wiz_NetData.ip, ip_source_adr, 4);
	memcpy(wiz_NetData.mac, source_hardware_adr,6);
	//memcpy(wiz_NetData.dns, ip_dns_adr, 4);
	wizchip_setnetinfo(&wiz_NetData);

	setsockopt(SOCKET0, SO_DESTPORT, &socket_port);
	//setsockopt(SOCKET0, SO_DESTIP, ip_destination_adr);
	setRTR(timeout_period);

	//DHCP>>>
	/*taskENTER_CRITICAL();
	DHCP_init(SOCKET5, dhcp_buf);
	taskEXIT_CRITICAL();*/
	//DHCP===*/
	{
	    /*uint8_t nmt_msg[2] = {0x81, 1};
	    can_tx_func(&hfdcan2, 0, 2, nmt_msg);*/
	    uint8_t data[4] = {0x06,0,0,0};
	    canopen_transmit(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, data);
	}
	extern IWDG_HandleTypeDef hiwdg1;
	{
	  uint8_t can_data_rx1[8] = {0};
	  /*FDCAN_RxHeaderTypeDef RxHeader;
	  while(RxHeader.Identifier != 0x701)
	    {
	      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_rx1);
	  HAL_IWDG_Refresh(&hiwdg1);
	  }*/

	  {
	    canopen_u canopen_rcv;
	    uint8_t can_data_tx[4] = {6,0,0,0};
	    /*while(canopen_rcv.values_t.COB_ID != 0x580+1 && canopen_rcv.values_t.index != 0x6040)
	    {
		vTaskDelay(50);
		HAL_IWDG_Refresh(&hiwdg1);
		canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
		uint8_t can_open_tx[8] = {0x2B,0x40,0x60,0,0x06,0,0,0};
	    }*/
	  }//*/
	}

	DIG_OUT4(SET);
	LED_VD1(SET);

	//>>>RS485
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
	extern UART_HandleTypeDef huart6;
	//<<<RS485
  /* Infinite loop */
  for(;;)
  {
	  /*//DHCP>>>
	  taskENTER_CRITICAL();
	  if(DHCP_run() == DHCP_IP_LEASED)
	    LED_VD5(RESET);
	  taskEXIT_CRITICAL();
	  //DHCP===*/
      FDCAN_RxHeaderTypeDef rxHeader;
      uint8_t temp_data[8];
	  if(!PIN_nINT)
	    {
	      LED_VD1(SET);
	      disconnect(SOCKET0);
	      setSn_IR(SOCKET0, getSn_IR(SOCKET0));
	    }

	  switch (getSn_SR(SOCKET0)){
	  case SOCK_ESTABLISHED:
	  {
	    DIG_OUT3(SET);//ENABLE signal for KOLLARMORGEN's drive
	    if(xTimerIsTimerActive(xTimer_period_reset) != pdFALSE)
	    xTimerStop(xTimer_period_reset, 0);

		  LED_VD2(SET);
		  LED_VD1(SET);
		  uint8_t buf[8] = {0x43, 0x05, 0x10,0,0,0,0,0};
		  can_tx_func(&hfdcan2, 0x622, 0, buf);

		  volatile fobos_protocol_buf_u fobos_eth_buf;
		  for(int i=0; i<10; i++)
		    fobos_eth_buf.fobos_protocol_buf_t.data[i] = 0;
		  uint16_t eth_pkg_length;
		  taskENTER_CRITICAL();
		  recv(SOCKET0,fobos_eth_buf.data_to_transmit, 258);
		  taskEXIT_CRITICAL();
		  eth_cmds_analysis(&fobos_eth_buf);

		  LED_VD1(RESET);
		  HAL_IWDG_Refresh(&hiwdg1);
		  uint8_t can_tx_data[8] = {0x40, 0x02, 0x10, 0,0,0,0,0};
		  canopen_u canopen_rcv_data;
		  canopen_req_resp_sdo(0x600+1, 0x40, 0x1002, 0, can_tx_data, &canopen_rcv_data);
		  break;
	  case SOCK_CLOSE_WAIT:
		  //xTimerStart(xTimer_period_reset, 0);
		  DIG_OUT3(RESET);//ENABLE signal for KOLLARMORGEN's drive
		  disconnect(SOCKET0);
		  LED_VD2(RESET);
		  break;

	  case SOCK_CLOSED:
	    /*if(xTimerIsTimerActive(xTimer_period_reset) == pdFALSE)
	    xTimerStart(xTimer_period_reset, 0);*/
	    {
	      /*static uint8_t a=0;
	      if(a == 0){
		  a++;*/
		  //uint8_t nmt_msg[2] = {0x81, 1};
		  //can_tx_func(&hfdcan2, 0, 2, nmt_msg);
		  uint8_t data[4] = {0x06,0,0,0};
		  canopen_transmit(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, data);
		  motor_state_indication = 0;
	      //}
	    }
	    DIG_OUT3(RESET);//ENABLE signal for KOLLARMORGEN's drive
	    LED_VD1(SET);
	    socket(SOCKET0,Sn_MR_TCP,socket_port,0x00);
	    LED_VD2(RESET);
		  break;

	  case SOCK_INIT:
	    listen(SOCKET0);
	    LED_VD2(RESET);
		  break;
	  }
	  vTaskDelay(80);
	  can_protocol_data_analyzing(&hfdcan2, &rxHeader, temp_data);
  /* USER CODE END 5 */
}
}}

void fobos_eth_protocol_send(uint8_t CMD, uint8_t bytes_in_packet_N, fobos_protocol_buf_u *fobos_eth_buf){
	fobos_eth_buf->fobos_protocol_buf_t.CMD = CMD;
	fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = bytes_in_packet_N;
	send(SOCKET0,fobos_eth_buf->data_to_transmit,bytes_in_packet_N+2);
}

BaseType_t xHoming = NULL, xPosition_func = NULL;
TaskHandle_t xHoming_Handle = NULL, xPosition_Handle = NULL;

volatile uint8_t motor_emergency = 0, terminals_statements = 0;

static void position_mode_process_left();
static void position_mode_process_right();
static void position_mode_process_calibration();

void eth_cmds_analysis(volatile fobos_protocol_buf_u *fobos_eth_buf){
  extern FDCAN_HandleTypeDef hfdcan2;
	switch(fobos_eth_buf->fobos_protocol_buf_t.CMD)
	{
	  static uint8_t basing_point = 0;
	case FOBOS_ETH_ECHO:
		send(SOCKET0,
			fobos_eth_buf->data_to_transmit,
			fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N+2);
		break;
	case  FOBOS_ETH_GET_MAC:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	    uint8_t mac_adr[6] = {0};
	    getSHAR(mac_adr);
	    fobos_eth_buf->fobos_protocol_buf_t.CMD = FOBOS_ETH_GET_MAC;
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    memcpy(fobos_eth_buf->fobos_protocol_buf_t.data+1, mac_adr, 6);
	    fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 6+1;
	    fobos_eth_protocol_send(FOBOS_ETH_GET_MAC, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
	  }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
		fobos_eth_protocol_send(FOBOS_ETH_GET_MAC, 2, fobos_eth_buf);
	      }
	  break;

	case  FOBOS_ETH_RST:
		fobos_eth_protocol_send(FOBOS_ETH_RST, 1, fobos_eth_buf);
		while(1);
		break;

	case FOBOS_SENSORS_STATE:
#define SENSOR_STATE
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
		{
			for(int i=0; i<9; i++)
			fobos_eth_buf->fobos_protocol_buf_t.data[i] = 0;
			uint8_t sensors_state = 0;

			fobos_eth_buf->fobos_protocol_buf_t.data[1] = terminals_statements & 0xC3;//S1,S2 ... S4,S3 в соответствии с единицами в байте. //Концевики С-рамы
			if(TABLE_LOCK_SENSOR_LEFT)
				sensors_state |= 0x08;
			if(TABLE_LOCK_SENSOR_RIGHT)
				sensors_state |= 0x10;
			fobos_eth_buf->fobos_protocol_buf_t.data[2] = sensors_state;//срабатывание датчиков стола
			if(motor_emergency == 0x0F)
			fobos_eth_buf->fobos_protocol_buf_t.data[2] |= 0b00000110;//сработал один из аварийных концевиков линейного мотора

			//>>>>>>>>>>>> Рама сбазирована data[3]
			{
			    if(motor_state_indication)
			    {
				uint8_t can_data_tx[4] = {6,0,0,0};//can_data_tx[0] младший байт
				canopen_u canopen_rcv;
				if(basing_point){
				    fobos_eth_buf->fobos_protocol_buf_t.data[3] = basing_point;
				}
				else
				{
				  canopen_req_resp_sdo(0x600+1, 0x40,0x6061,0,can_data_tx, &canopen_rcv);
				  if(canopen_rcv.values_t.data[0] == 6)
				  {
				      canopen_req_resp_sdo(0x600+1, 0x40,0x6041,0,can_data_tx, &canopen_rcv);
				      if((canopen_rcv.values_t.data[1]&0x16) == 0x16 && (canopen_rcv.values_t.data[0]&0xB7) == 0xB7)
				      {
					  basing_point = 0xFF;
					  fobos_eth_buf->fobos_protocol_buf_t.data[3] = basing_point;
				      }
				      else{
					  fobos_eth_buf->fobos_protocol_buf_t.data[3] = 0;
				      }
				  }
				  else
				  {
				    fobos_eth_buf->fobos_protocol_buf_t.data[3] = basing_point;
				  }
				}//else from if(basing_point)
			  }
			  else{
			      fobos_eth_buf->fobos_protocol_buf_t.data[3] = 0;
			  }
		      }
			//<<<<<<<<<<<< Рама сбазирована?
			//>>>>>>>>>>>> Линейный мотор достиг заданной точки data[4]
			{
			      uint8_t can_data_tx[8] = {0};//can_data_tx[0] младший байт
			       canopen_u canopen_rcv;

			       if(basing_point == 0 /*&& motor_state_indication == 0*/){
				   fobos_eth_buf->fobos_protocol_buf_t.data[4] = 0;//контроль достижения заданной точки
			       }
			       else {
				   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
			       if(canopen_rcv.values_t.data[0] == 0x01){
				      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
				      fobos_eth_buf->fobos_protocol_buf_t.data[4] = 0;//контроль достижения заданной точки
				      if(canopen_rcv.values_t.data[1] == 0x16)
				      fobos_eth_buf->fobos_protocol_buf_t.data[4] = 1;
				  }
			       }
			}
			//<<<<<<<<<<<< Линейный мотор достиг заданной точки data[4]
			//>>>>>>>>>>>> Положение сервопривода data[5-8]
			{
			  canopen_u canopen_rcv = {0};
			  uint8_t can_data_tx[8] = {0}, temp_lim_switches = 0;
			  if(motor_state_indication){
			  canopen_req_resp_sdo(0x600+1, SDO_REQUEST, 0x6064, 0, can_data_tx, &canopen_rcv);

			  fobos_eth_buf->fobos_protocol_buf_t.data[5] = canopen_rcv.values_t.data[3];
			  fobos_eth_buf->fobos_protocol_buf_t.data[6] = canopen_rcv.values_t.data[2];
			  fobos_eth_buf->fobos_protocol_buf_t.data[7] = canopen_rcv.values_t.data[1];
			  fobos_eth_buf->fobos_protocol_buf_t.data[8] = canopen_rcv.values_t.data[0];
			  }
			  else{
			      for(int i=5; i<=8;i++)
			      fobos_eth_buf->fobos_protocol_buf_t.data[i] = 0;
			  }
			}
			//<<<<<<<<<<<< Положение сервопривода data[5-8]
			//>>>>>>>>>>>> Состояние готовности мотора data[9]
			fobos_eth_buf->fobos_protocol_buf_t.data[9] = motor_state_indication;
			//<<<<<<<<<<<< Состояние готовности мотора data[9]
			fobos_eth_protocol_send(FOBOS_SENSORS_STATE, 10, fobos_eth_buf);
		}
		else{
		      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		      fobos_eth_protocol_send(FOBOS_SENSORS_STATE, 1, fobos_eth_buf);
		    }
		break;
	case FOBOS_GENERATOR_STATE:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	    uint8_t can_data_buf[8];
	    FDCAN_RxHeaderTypeDef RxHeader;
	    can_tx_func(&hfdcan2, 0x620+2, 0, can_data_buf);
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
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
		fobos_eth_protocol_send(FOBOS_GENERATOR_STATE, 2, fobos_eth_buf);
	      }
		break;
	case FOBOS_SERVOMOTOR_PLACEMENT:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0 /*&& motor_state_indication*/)
	  {
	    canopen_u canopen_rcv = {0};
	    uint8_t can_data_tx[8] = {0}, temp_lim_switches = 0;
	    fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 5;
	    if(motor_state_indication){
	    canopen_req_resp_sdo(0x600+1, SDO_REQUEST, 0x6064, 0, can_data_tx, &canopen_rcv);

	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;

	    fobos_eth_buf->fobos_protocol_buf_t.data[1] = canopen_rcv.values_t.data[3];
	    fobos_eth_buf->fobos_protocol_buf_t.data[2] = canopen_rcv.values_t.data[2];
	    fobos_eth_buf->fobos_protocol_buf_t.data[3] = canopen_rcv.values_t.data[1];
	    fobos_eth_buf->fobos_protocol_buf_t.data[4] = canopen_rcv.values_t.data[0];
	    }
	    else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = 0;//FOBOS_ETH_ERR_NR;
	    }
	    //uint32_t encoder_val;//um
	    /*encoder_val = (fobos_eth_buf->fobos_protocol_buf_t.data[1]<<24) | (fobos_eth_buf->fobos_protocol_buf_t.data[2]<<16)
			| (fobos_eth_buf->fobos_protocol_buf_t.data[3]<<8) | fobos_eth_buf->fobos_protocol_buf_t.data[4];

	    fobos_eth_buf->fobos_protocol_buf_t.data[5] = basing_point;*/

	    fobos_eth_protocol_send(FOBOS_SERVOMOTOR_PLACEMENT,
				    fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N,
				    fobos_eth_buf);
	  }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
		fobos_eth_protocol_send(FOBOS_SERVOMOTOR_PLACEMENT, 2, fobos_eth_buf);
	      }
		break;
	case FOBOS_STATEMENT://готовность мотора к работе
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	    {
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	      fobos_eth_buf->fobos_protocol_buf_t.data[1] = motor_state_indication;			//не готово устройство
	      fobos_eth_protocol_send(FOBOS_STATEMENT, 2, fobos_eth_buf);
	    }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
		fobos_eth_protocol_send(FOBOS_STATEMENT, 2, fobos_eth_buf);
	      }
		break;
	case FOBOS_CMD_BASING://homing process
#define CMD_HOMING
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0 /*&& motor_state_indication*/)
	  {
	    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	    //запрос к линейному приводу о положении
	      /*vTaskDelay(50);
	      uint8_t can_data_buf[8] = {0x10,0};
	      can_tx_func(&hfdcan2, 0x620+2, 2, can_data_buf);*/
	      vTaskDelay(100);
	      if(basing_point == 0)
	      {
	      if(xHoming == NULL)
	    xHoming = xTaskCreate(homing_process, "homing", 128, (void*)0, tskIDLE_PRIORITY, &xHoming_Handle);
	  }
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	      fobos_eth_protocol_send(FOBOS_CMD_BASING, 1, fobos_eth_buf);
	      /*else{
		  if(xPosition_func == NULL)
      xPosition_func = xTaskCreate(position_mode_process_right, "MotorRIGHT", 128,(void*)0, 0, &xPosition_Handle);
		  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		  fobos_eth_protocol_send(FOBOS_CMD_BASING, 1, fobos_eth_buf);
	      }*/
	  }
	  else{
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	      fobos_eth_protocol_send(FOBOS_CMD_BASING, 1, fobos_eth_buf);
	  }

		break;

	case FOBOS_CMD_WORK://21 rotation
	#define CMD_WORK
		  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 1)
		  {
		    uint8_t temp_lim_switches = 0;
		    FDCAN_RxHeaderTypeDef RxHeader;
		    if(fobos_eth_buf->fobos_protocol_buf_t.data[0] && fobos_eth_buf->fobos_protocol_buf_t.data[0] <= 2)
		      {
			vTaskDelay(100);
			if(fobos_eth_buf->fobos_protocol_buf_t.data[0] == 1)
			{
			    uint8_t data[2] = {0x10,0};
			    can_tx_func(&hfdcan2, 0x620+2,2,data);
			    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		fobos_eth_protocol_send(FOBOS_CMD_WORK, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
			}
			else if(fobos_eth_buf->fobos_protocol_buf_t.data[0] == 2)
			  {
			    uint8_t data[2] = {0x20,0};
			    can_tx_func(&hfdcan2, 0x620+2,2,data);
			    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			    fobos_eth_protocol_send(FOBOS_CMD_WORK, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
			  }
		    }
		    else{
		      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		      fobos_eth_protocol_send(FOBOS_CMD_WORK, 1, fobos_eth_buf);
		    }
		  }
		  else{
			  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
			  fobos_eth_protocol_send(FOBOS_CMD_WORK, 1, fobos_eth_buf);
		      }
			break;
	case FOBOS_CMD_LAMP://22 cmd
		  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 1){
		    if(fobos_eth_buf->fobos_protocol_buf_t.data[0])
		      {
			XRAY_GEN_START(SET);
		      }
		    else
		      {
			XRAY_GEN_START(RESET);
		      }

		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;

		  }
		  else{
		      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		  }
		  fobos_eth_protocol_send(FOBOS_CMD_LAMP, 1, fobos_eth_buf);
		  break;

	case FOBOS_CMD_BASING_STATEMENT://23
#define CMD_BASING_STATEMENT
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	      if(motor_state_indication)
		{
		    uint8_t can_data_tx[4] = {6,0,0,0};//can_data_tx[0] младший байт
		    canopen_u canopen_rcv;
		    if(basing_point){
			fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			fobos_eth_buf->fobos_protocol_buf_t.data[1] = basing_point;
		    }
		    else
		    {
		      canopen_req_resp_sdo(0x600+1, 0x40,0x6061,0,can_data_tx, &canopen_rcv);
		      if(canopen_rcv.values_t.data[0] == 6)
		      {
			  canopen_req_resp_sdo(0x600+1, 0x40,0x6041,0,can_data_tx, &canopen_rcv);
			  if((canopen_rcv.values_t.data[1]&0x16) == 0x16 && (canopen_rcv.values_t.data[0]&0xB7) == 0xB7)
			  {
			      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			      basing_point = 0xFF;
			      fobos_eth_buf->fobos_protocol_buf_t.data[1] = basing_point;
			  }
			  else{
			      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			      fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
			  }
		      }
		      else
		      {
			fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
			fobos_eth_buf->fobos_protocol_buf_t.data[1] = basing_point;
		      }
		    }//else from if(basing_point)
	      }
	      else{
		  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		  fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
	      }
	  }
	  else{
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	      fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;
	  }
	  fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 2;
	  fobos_eth_protocol_send(FOBOS_CMD_BASING_STATEMENT, 2, fobos_eth_buf);
	break;

	case FOBOS_CMD_START://24, линейное перемещение мотора
#define CMD_START
	  /*if(xHoming == NULL)
	    xHoming = xTaskCreate(homing_process, "homing", 128, (void*)0, tskIDLE_PRIORITY, &xHoming_Handle);*/
	  {
	    if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 1 && motor_state_indication)
	      {
		if(fobos_eth_buf->fobos_protocol_buf_t.data[0] == 1){
		    if(xPosition_func == NULL)
		      xPosition_func = xTaskCreate(position_mode_process_calibration, "CALIBRATION", 128,(void*)0, 0, &xPosition_Handle);
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		}
	    else if(fobos_eth_buf->fobos_protocol_buf_t.data[0] == 2)//left position
		  {
		    //position_mode_process_left();
		    if(xPosition_func == NULL)
		    xPosition_func = xTaskCreate(position_mode_process_left, "MotorLEFT", 128,(void*)0, 0, &xPosition_Handle);
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		  }
		else if(fobos_eth_buf->fobos_protocol_buf_t.data[0] == 3)//right position
		  {
		    if(xPosition_func == NULL)
		    xPosition_func = xTaskCreate(position_mode_process_right, "MotorRIGHT", 128,(void*)0, 0, &xPosition_Handle);
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		  }
		else
		  {
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NR;
		  }
	      }
	    else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	    }
	    fobos_eth_protocol_send(FOBOS_CMD_START, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
	  }
	  break;

	case FOBOS_CMD_START_STATUS: //25 cmd
#define CMD_START_STATUS
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	      fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 2;
	      uint8_t can_data_tx[8] = {0};//can_data_tx[0] младший байт
	         canopen_u canopen_rcv;

	         if(basing_point == 0 /*&& motor_state_indication == 0*/){
	             fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	             fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;//контроль достижения заданной точки
	         }
	         else {
	             canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
	         if(canopen_rcv.values_t.data[0] != 0x01){
	        	fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	            }
	            else{
	        	canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
	        	fobos_eth_buf->fobos_protocol_buf_t.data[1] = 0;//контроль достижения заданной точки
	        	if(canopen_rcv.values_t.data[1] == 0x16)
	        	fobos_eth_buf->fobos_protocol_buf_t.data[1] = 1;
	            }
	         }
	  }
	  else
	    {
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	    }

	  fobos_eth_protocol_send(FOBOS_CMD_START_STATUS, fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N, fobos_eth_buf);
	  break;

	case FOBOS_CMD_STOP://30 cmd
	  {
	    uint8_t can_data_tx[4] = {0x06,0,0,0};//can_data_tx[0] младший байт
	    canopen_u canopen_rcv;

	    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ,0x6040,0,can_data_tx, &canopen_rcv);
	    can_data_tx[0] = 0x30;
	    can_tx_func(&hfdcan2,0x622,2,can_data_tx);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    fobos_eth_protocol_send(FOBOS_CMD_STOP, 1, fobos_eth_buf);
	  }
		break;

	case FOBOS_EMB_SOFT_VER:
	{
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0){
		char string_data[] = {"Fobos embedded software version 16.0"};
		int length = strlen(string_data)+1;
		fobos_eth_buf->fobos_protocol_buf_t.CMD = FOBOS_EMB_SOFT_VER;
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		memcpy(fobos_eth_buf->fobos_protocol_buf_t.data+1, string_data, length);
		fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = length;
		send(SOCKET0,fobos_eth_buf->data_to_transmit,length+2);
	  }
	  else{
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	      fobos_eth_protocol_send(FOBOS_EMB_SOFT_VER, 1, fobos_eth_buf);
	  }
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
		if(socket_port){
		if(confirmation(fobos_eth_buf, &socket_port))
		  setSn_PORT(SOCKET0, socket_port);
		}
		else
		  {
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		    fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;
		    confirmation(fobos_eth_buf, &socket_port);
		}
		break;
	case FOBOS_CHANGE_TIMEOUT:
	  timeout_period = (fobos_eth_buf->fobos_protocol_buf_t.data[1]<<8)|fobos_eth_buf->fobos_protocol_buf_t.data[2];
	  timeout_period *= 10;
	  if(timeout_period > 800){
		if(confirmation(fobos_eth_buf, (void*)0))
			setRTR(timeout_period);
	  }
	  else {
	      fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 0;
	      confirmation(fobos_eth_buf, (void*)0);
	  }

		break;

	//GET ethernet statements >>>>
	case FOBOS_ETH_GET_IP:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	    uint8_t ip[4];
	    getSIPR(ip);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, ip, 4+1);
	  }
	  else{
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	      fobos_eth_protocol_send(FOBOS_ETH_GET_IP, 1, fobos_eth_buf);
	  }
	  break;
	case FOBOS_ETH_DHCP:
	  {
	    uint8_t dhcp_state = fobos_eth_buf->fobos_protocol_buf_t.data[1];
	    wiz_NetData.dhcp = dhcp_state;
	    if(dhcp_state == 1 || dhcp_state == 2)//Не активная фича!
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
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	    uint8_t mask[4]={0};
	    getSUBR(mask);
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, mask, 4+1);
	  }
	  else{
	      fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
	      fobos_eth_protocol_send(FOBOS_ETH_GET_MASK, 1, fobos_eth_buf);
	      }
	  break;
	case FOBOS_ETH_GET_PORT:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
		  {
		    uint8_t port[2]={0};
		    socket_port = getSn_PORT(SOCKET0);
		    port[0] = socket_port>>8;
		    port[1] = socket_port;
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		    feedback_params(fobos_eth_buf, port, 2+1);
		  }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_protocol_send(FOBOS_ETH_GET_PORT, 1, fobos_eth_buf);
	      }
		  break;
	case FOBOS_ETH_GET_TIMEOUT:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
		  {
		    uint8_t timeout[2];
		    uint16_t timeout_temp1;
		    timeout_temp1 = getRTR();
		    timeout[0] = timeout_temp1>>8;
		    timeout[1] = timeout_temp1;
		    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
		    feedback_params(fobos_eth_buf, timeout, 2+1);
		  }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_protocol_send(FOBOS_ETH_GET_TIMEOUT, 1, fobos_eth_buf);
	      }
		  break;
	case FOBOS_ETH_GET_DHCP_STATE:
	  if(fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N == 0)
	  {
	    uint8_t dhcp_state = 0;
	    dhcp_state = wiz_NetData.dhcp;
	    fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_NO;
	    feedback_params(fobos_eth_buf, &dhcp_state, 1+1);
	  }
	  else{
		fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_PA;
		fobos_eth_protocol_send(FOBOS_ETH_GET_DHCP_STATE, 1, fobos_eth_buf);
	    }
	  break;
	default:
	  fobos_eth_buf->fobos_protocol_buf_t.bytes_in_packet_N = 1;
	  fobos_eth_buf->fobos_protocol_buf_t.data[0] = FOBOS_ETH_ERR_CMD;
	  fobos_eth_protocol_send(fobos_eth_buf->fobos_protocol_buf_t.CMD, 1, fobos_eth_buf);
	  break;
	}
	xSemaphoreGive(Mutex_Eth);
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

uint8_t canopen_transmit(uint16_t COB_ID, uint8_t control_field, uint16_t index, uint8_t subindex, uint8_t *data){
  uint8_t temp_buf[8] = {0};
  temp_buf[0] = control_field;

  temp_buf[1] = (uint8_t)index;
  temp_buf[2] = (uint8_t)(index>>8);
  temp_buf[3] = subindex;

  temp_buf[4] = *(data+0);	//low byte
  temp_buf[5] = *(data+1);
  temp_buf[6] = *(data+2);
  temp_buf[7] = *(data+3);		//high byte
  return can_tx_func(&hfdcan2, COB_ID, 8, temp_buf);
}

canopen_u* canopen_receive(canopen_u* canopen_rcv){
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t can_data_rcv[8] = {0};
  uint16_t ID_temp = canopen_rcv->values_t.COB_ID-0x80;// special for SDO!
  uint16_t index = canopen_rcv->values_t.index;
  uint8_t subindex = canopen_rcv->values_t.subindex;
  while(RxHeader.Identifier != ID_temp
      && index != can_data_rcv[1]
      && (index>>8) != can_data_rcv[2]
      /*&& subindex != can_data_rcv[3]*/)
      can_protocol_data_analyzing(&hfdcan2, &RxHeader, can_data_rcv);
  canopen_rcv->values_t.COB_ID = RxHeader.Identifier;
  canopen_rcv->values_t.control_field = can_data_rcv[0];
  canopen_rcv->values_t.index = can_data_rcv[1] | (can_data_rcv[2]<<8);
  canopen_rcv->values_t.subindex = can_data_rcv[3];
  canopen_rcv->values_t.data[0] = can_data_rcv[4];
  canopen_rcv->values_t.data[1] = can_data_rcv[5];
  canopen_rcv->values_t.data[2] = can_data_rcv[6];
  canopen_rcv->values_t.data[3] = can_data_rcv[7];
  return canopen_rcv;
}

canopen_u* canopen_req_resp_sdo(uint16_t COB_ID, uint8_t control_field, uint16_t index, uint8_t subindex, uint8_t *data,
				canopen_u *canopen_rcv){

 // while(canopen_rcv->values_t.COB_ID != (COB_ID - 0x80)
      /*|| index != canopen_rcv.values_t.index
      || subindex != canopen_rcv.values_t.subindex)*/
    {
      canopen_transmit(COB_ID, control_field, index, subindex, data);
      canopen_rcv->values_t.COB_ID = COB_ID;
      canopen_rcv->values_t.index = index;
      canopen_rcv->values_t.subindex = subindex;
      canopen_receive(canopen_rcv);
    }
  return canopen_rcv;
}

void vFobos_Start_Process(){
uint8_t scan_types = 0, can_data_rx[8] = {0};
FDCAN_RxHeaderTypeDef RxHeader;
  for(;;){
      can_data_rx[5] = 0;
      if(xQueueReceive(xQueue_Scanning_start, &scan_types, 10)){
	  position_mode_process(scan_types);
      }
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

char can_tx_func(FDCAN_HandleTypeDef *hfdcan, unsigned int ID, uint32_t data_lenght, uint8_t *data)
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

	static a=0;
	LED_VD6(a^=1);
	uint32_t buf_num = FDCAN_TX_BUFFER1;
	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, buf_num)){
	    if(buf_num == FDCAN_TX_BUFFER4)
	      buf_num = FDCAN_TX_BUFFER1;
	    else
	      buf_num = buf_num << 1;
	}
	if(HAL_FDCAN_AddMessageToTxBuffer(hfdcan, &TxHeader, data, buf_num) == HAL_OK)
	if(HAL_FDCAN_EnableTxBufferRequest(hfdcan, buf_num) == HAL_OK)
	  return 0xFF;

	return 0;
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

	HAL_IWDG_Refresh(&hiwdg1);
	extern FDCAN_HandleTypeDef hfdcan2;
	terminals_statements = (DIG_IN8<<7)|(DIG_IN7<<6)|(DIG_IN6<<5)|(DIG_IN5<<4)
			      |(DIG_IN4<<3)|(DIG_IN3<<2)|(DIG_IN2<<1)|(DIG_IN1);
	uint8_t buf[8];
	can_tx_func(&hfdcan2, 0x622, 0, buf);
}

static void position_mode_process_calibration(){
  uint8_t can_data_tx[4] = {0};//can_data_tx[0] младший байт
   canopen_u canopen_rcv;
   HAL_IWDG_Refresh(&hiwdg1);

   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
   if(canopen_rcv.values_t.data[0] != 0x01){
   can_data_tx[0] = 6;//shutdown
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
   can_data_tx[0] = 1;//profile position mode
   canopen_req_resp_sdo(0x600+1, SDO_1BYTE_REQ,0x6060,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

   while((canopen_rcv.values_t.data[0] & 0b00000111) != 1){
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv); HAL_IWDG_Refresh(&hiwdg1);
   }

   can_data_tx[0] = 0x07;//switch on
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   HAL_IWDG_Refresh(&hiwdg1);
   while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

     can_data_tx[0] = 0x0F;//operation EN
     canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
     canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
     while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }
   else{
 	can_data_tx[0] = 0x07;//switch on
         canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
         canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);

         while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
           canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

 	can_data_tx[0] = 0x0F;//operation EN
 	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
 	canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);
 	while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }

   	can_data_tx[0] = 0x01;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

   	can_data_tx[0] = 0x3F;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
   	  xPosition_func = NULL;
   	  vTaskDelete(xPosition_Handle);
}

static void position_mode_process_right(){
  uint8_t can_data_tx[4] = {0};//can_data_tx[0] младший байт
   canopen_u canopen_rcv;
   HAL_IWDG_Refresh(&hiwdg1);

   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
   if(canopen_rcv.values_t.data[0] != 0x01){
   can_data_tx[0] = 6;//shutdown
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
   can_data_tx[0] = 1;//profile position mode
   canopen_req_resp_sdo(0x600+1, SDO_1BYTE_REQ,0x6060,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

   while((canopen_rcv.values_t.data[0] & 0b00000111) != 1){
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv); HAL_IWDG_Refresh(&hiwdg1);
   }

   can_data_tx[0] = 0x07;//switch on
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   HAL_IWDG_Refresh(&hiwdg1);
   while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

     can_data_tx[0] = 0x0F;//operation EN
     canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
     canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
     while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }
   else{
 	can_data_tx[0] = 0x07;//switch on
         canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
         canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);

         while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
           canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

 	can_data_tx[0] = 0x0F;//operation EN
 	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
 	canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);
 	while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }

   	can_data_tx[0] = 0x03;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

   	can_data_tx[0] = 0x3F;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
   	  xPosition_func = NULL;
   	  vTaskDelete(xPosition_Handle);
}

static void position_mode_process_left(){
  uint8_t can_data_tx[4] = {0};//can_data_tx[0] младший байт
   canopen_u canopen_rcv;
   HAL_IWDG_Refresh(&hiwdg1);

   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
   if(canopen_rcv.values_t.data[0] != 0x01){
   can_data_tx[0] = 6;//shutdown
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
   can_data_tx[0] = 1;//profile position mode
   canopen_req_resp_sdo(0x600+1, SDO_1BYTE_REQ,0x6060,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

   while((canopen_rcv.values_t.data[0] & 0b00000111) != 1){
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv); HAL_IWDG_Refresh(&hiwdg1);
   }

   can_data_tx[0] = 0x07;//switch on
   canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
   HAL_IWDG_Refresh(&hiwdg1);
   while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
       canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

     can_data_tx[0] = 0x0F;//operation EN
     canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
     canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
     while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }
   else{
 	can_data_tx[0] = 0x07;//switch on
         canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
         canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);

         while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
           canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

 	can_data_tx[0] = 0x0F;//operation EN
 	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
 	canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);
 	while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
 	  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
   }

   	can_data_tx[0] = 0x02;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

   	can_data_tx[0] = 0x3F;
   	canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
   	  xPosition_func = NULL;
   	  vTaskDelete(xPosition_Handle);
}

static void position_mode_process(uint8_t scan_types){
  uint8_t can_data_tx[4] = {0};//can_data_tx[0] младший байт
  canopen_u canopen_rcv;
  HAL_IWDG_Refresh(&hiwdg1);

  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 1 - profile position mode
  if(canopen_rcv.values_t.data[0] != 0x01){
  can_data_tx[0] = 6;//shutdown
  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
  can_data_tx[0] = 1;//profile position mode
  canopen_req_resp_sdo(0x600+1, SDO_1BYTE_REQ,0x6060,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

  while((canopen_rcv.values_t.data[0] & 0b00000111) != 1){
      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv); HAL_IWDG_Refresh(&hiwdg1);
  }

  can_data_tx[0] = 0x07;//switch on
  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  HAL_IWDG_Refresh(&hiwdg1);
  while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

    can_data_tx[0] = 0x0F;//operation EN
    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
    canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
    while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

    can_data_tx[0] = 0x3F; //Operation start (process)
    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
  }
  else{
	can_data_tx[0] = 0x07;//switch on
        canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
        canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);

        while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
          canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

	can_data_tx[0] = 0x0F;//operation EN
	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
	canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);
	while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
	  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

	can_data_tx[0] = 0x3F; //Operation start (process)
	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  }

  switch(scan_types){
    case 1:
      {
	uint8_t can_tx_to_rs485_buf[16] = {8,5,0x4B,8,0,3,0xAA,0,8,0xFB,0,0,0,0,0,0};
	can_tx_func(&hfdcan2, 0x640+2, 8, &can_tx_to_rs485_buf[0]);
	can_tx_func(&hfdcan2, 0x640+2, 8, &can_tx_to_rs485_buf[8]);
	scan_types = 0;
	can_data_tx[0] = 0x0F;
	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

	while(canopen_rcv.values_t.data[0]&0b00000111 != 0x07)
	  canopen_req_resp_sdo(0x600+1, 0x40,0x6041,0,can_data_tx, &canopen_rcv);

	can_data_tx[0] = 0x02;
	canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

	while(DIG_IN1 == 0)
	  HAL_IWDG_Refresh(&hiwdg1);

	can_data_tx[0] = 0x3F;
	canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
      }
      break;
    case 2:
      {
	uint8_t can_tx_to_rs485_buf[16] = {8,5,0x4B,8,0,0x83,0xAA,0,8,0x7B,0,0,0,0,0,0};
	can_tx_func(&hfdcan2, 0x640+2, 8, &can_tx_to_rs485_buf[0]);
	can_tx_func(&hfdcan2, 0x640+2, 8, &can_tx_to_rs485_buf[8]);
	scan_types = 0;
	can_data_tx[0] = 0x0F;
	canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
	while(canopen_rcv.values_t.data[0] != 0xB7)
	  canopen_req_resp_sdo(0x600+1, 0x40,0x6041,0,can_data_tx, &canopen_rcv);

	can_data_tx[0] = 0x03;
	canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

	can_data_tx[0] = 0x3F;
	canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
      }
      break;
    case 3:
      {
	  scan_types = 0;
	  canopen_u canopen_rcv = {0};
	  can_data_tx[0] = 0x0F;
	  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
	  while(canopen_rcv.values_t.data[1] != 0x06 && canopen_rcv.values_t.data[0] != 0xB7)
	  {
	    vTaskDelay(20);
	    canopen_req_resp_sdo(0x600+1, 0x40,0x6041,0,can_data_tx, &canopen_rcv);
	  }
	  can_data_tx[0] = 0x02;
	  canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

	  can_data_tx[0] = 0x3F;
	  canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
	  vTaskDelay(500);
	  canopen_req_resp_sdo(0x600+1, 0x40, 0x6041, 0, can_data_tx, &canopen_rcv);
	  while(canopen_rcv.values_t.data[1] == 0x12 && canopen_rcv.values_t.data[0] == 0xB7){
		      vTaskDelay(100);
		      canopen_req_resp_sdo(0x600+1, 0x40, 0x6041, 0, can_data_tx, &canopen_rcv);
		  }


	  can_data_tx[0] = 0x0F;
	  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);

	  can_data_tx[0] = 0x03;
	  canopen_req_resp_sdo(0x600+1, 0x2B,0x2080,0,can_data_tx, &canopen_rcv);

	  can_data_tx[0] = 0x3F;
	  canopen_req_resp_sdo(0x600+1, 0x2B,0x6040,0,can_data_tx, &canopen_rcv);
	  while(canopen_rcv.values_t.data[1] == 0x12 && canopen_rcv.values_t.data[0] == 0xB7){
		    vTaskDelay(100);
		    canopen_req_resp_sdo(0x600+1, 0x40, 0x6041, 0, can_data_tx, &canopen_rcv);
		}
      }
      break;
    default:
      ;
  }
}

static void homing_process(){
  uint8_t can_data_tx[4] = {0};//can_data_tx[0] младший байт
  canopen_u canopen_rcv;
  HAL_IWDG_Refresh(&hiwdg1);
  vTaskDelay(100);
  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6061,0,can_data_tx, &canopen_rcv);//mode request: 6 - homing mode

  if(canopen_rcv.values_t.data[0] != 0x06){
  can_data_tx[0] = 6;//shutdown
  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  canopen_req_resp_sdo(0x600+1, SDO_1BYTE_REQ,0x6060,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);

  while((canopen_rcv.values_t.data[0] & 0b00000111) != 1){
      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv); HAL_IWDG_Refresh(&hiwdg1);
  }
  vTaskDelay(100);
  HAL_IWDG_Refresh(&hiwdg1);

  can_data_tx[0] = 0x07;//switch on
  canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  HAL_IWDG_Refresh(&hiwdg1);
  while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23)
      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

    can_data_tx[0] = 0x0F;//operation EN
    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
    canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
    while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27)
	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);

    can_data_tx[0] = 0x1F; //Operation start (homing)
    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
  }
  else
  	      {
      vTaskDelay(100);
  		canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  		if(canopen_rcv.values_t.data[1] == 0x02 && canopen_rcv.values_t.data[0] == 0xB7)
  		  ;
  		else{
  		  can_data_tx[0] = 0x07;//switch on
  		    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);	HAL_IWDG_Refresh(&hiwdg1);
  		    canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);

  		    while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x23){
  		        canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
  		      HAL_IWDG_Refresh(&hiwdg1);
  		    }
  		  vTaskDelay(100);
  		      can_data_tx[0] = 0x0F;//operation EN
  		      canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv); 	HAL_IWDG_Refresh(&hiwdg1);
  		      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);		HAL_IWDG_Refresh(&hiwdg1);
  		      while((canopen_rcv.values_t.data[0] & 0b01100111) != 0x27){
  			HAL_IWDG_Refresh(&hiwdg1);
  		  	      canopen_req_resp_sdo(0x600+1, SDO_REQUEST,0x6041,0,can_data_tx, &canopen_rcv);
  		      }
  		    HAL_IWDG_Refresh(&hiwdg1);
  		      can_data_tx[0] = 0x1F; //Operation start (homing)
  		      canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
  		    HAL_IWDG_Refresh(&hiwdg1);
  	      }
  	      }
  xHoming = NULL;
  vTaskDelete(xHoming_Handle);
}

uint8_t can_protocol_data_analyzing(FDCAN_HandleTypeDef *hfdcan,
				FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData)
{
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, pRxHeader, pRxData) == HAL_OK)
	{
		volatile uint32_t address = pRxHeader->Identifier;
		/*if(address == 0x81){
		  motor_emergency = 0x0F;
		}
		else */if(address == 0x722){
		    volatile uint8_t buf[8];
		    memcpy(buf,pRxData,2);
		    terminals_statements = buf[1];
		    //terminals_statements = *(pRxData+1);
		}
		else if(address == 0x701)
		{
		    uint8_t can_data_tx[4] = {0x80,0,0,0};//can_data_tx[0] младший байт
		    canopen_u canopen_rcv;
		    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
		    can_data_tx[0] = 0x06;
		    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);
		    can_data_tx[0] = 0x07;
		    canopen_req_resp_sdo(0x600+1, SDO_2BYTES_REQ, 0x6040, 0, can_data_tx, &canopen_rcv);//*/
		    /*uint8_t can_open_tx[8] = {0x2B,0x40,0x60,0,0x06,0,0,0};
		    can_tx_func(hfdcan, 0x601,8,can_open_tx);*/
		    vTaskDelay(300);
		    motor_state_indication = 0xFF;
		}
		else if(address == 0x581)
		{
		    motor_state_indication = 0xFF;
		    uint8_t can_data[8];
		    memcpy(can_data, pRxData, 8);
		    canopen_u canopen_rcv_data;
		    canopen_rcv_data.values_t.index = (can_data[2]<<8) | can_data[1];
		    for(int i=0;i<4;i++)
		      canopen_rcv_data.values_t.data[i] = can_data[i+4];
		    if(canopen_rcv_data.values_t.index == 0x1002 /*&& canopen_rcv.values_t.control_field == SDO_4BYTES_REQ+0x20*/){
			static a=1;
			LED_VD2(a^=1);
			if(canopen_rcv_data.values_t.data[1]&0x20)
			  {
			    motor_emergency = 0x0F;
			  }
			else{
			    motor_emergency = 0;
			}
		    }
		}
		static char a=0;
		  LED_VD6(a^=1);
		return 0xFF;
	}
	else
		return 0;
}
