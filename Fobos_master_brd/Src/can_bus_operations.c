/*
 * can_bus_operations.c
 *
 *  Created on: 2 мая 2019 г.
 *      Author: zilkov
 */
#include "can_bus_operations.h"
extern FDCAN_HandleTypeDef hfdcan2;
uint8_t can_bus_tx(uint16_t ID, uint8_t length, uint8_t *data){

  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = ID;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = (length<<16);
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_TX_EVENT;
  TxHeader.MessageMarker = 0;

  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data) == HAL_OK)
    return 0xFF;

  return 0;
}

void can_bus_rcv_task(){
  uint8_t a;
  for(;;){
      uint8_t can_rx_data[8] = {0};
      FDCAN_RxHeaderTypeDef RxHeader;
      while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0)){
	if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, can_rx_data) == HAL_OK){
	    switch (RxHeader.Identifier) {
	      case 0x722:

		break;
	      default:
		break;
	    }
	}
      }
      LED_VD7(RESET);
      //taskYIELD();
  }
}
