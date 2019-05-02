/*
 * can_bus_operations.c
 *
 *  Created on: 2 мая 2019 г.
 *      Author: zilkov
 */
#include "stm32h7xx_hal.h"
uint8_t can_bus_tx(uint16_t ID, uint8_t length, uint8_t *data){
  extern FDCAN_HandleTypeDef hfdcan2;
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
